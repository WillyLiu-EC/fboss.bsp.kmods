// SPDX-License-Identifier: GPL-2.0
/*
 * DS250DF810 / DS2x0DFxx0 Retimer I2C Driver
 *
 * Based on TI DS2x0DFxx0 Programmer's Guide (SNLU182H)
 * Supports: DS280DF810, DS250DF810, DS250DF410, DS250DF230, DS250DF210
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* Global Registers */
#define REG_DEVICE_ID       0xF1
#define REG_VENDOR_ID       0xFE
#define REG_CHAN_SEL        0xFC
#define REG_PAGE_SEL        0xFF

/* Channel Registers */
#define REG_CH_RESET        0x00
#define REG_CDR_CTRL        0x0A
#define REG_RATE            0x2F
#define REG_ADAPT_MODE      0x31
#define REG_DFE_CTRL        0x1E
#define REG_FIR_MAIN        0x3D
#define REG_FIR_PRE         0x3E
#define REG_FIR_POST        0x3F
#define REG_CDR_STATUS      0x02
#define REG_SD_LOCK_STATUS  0x78
#define REG_HEO             0x27
#define REG_VEO             0x28
#define REG_CTLE_OBS        0x8F

/* Standard Rate Settings (Reg_0x2F[7:4]) */
enum ds250_rate {
	RATE_10G3125  = 0x00,
	RATE_10G9375  = 0x10,
	RATE_12G5     = 0x20,
	RATE_25G0     = 0x40,
	RATE_25G78125 = 0x50,  /* Default */
	RATE_28G125   = 0x70,  /* DS280DF810 only */
};

/* TX FIR presets per data rate */
struct ds250_fir_preset {
	u8 main_mag;    /* Main-cursor magnitude (0-31) */
	u8 main_sign;   /* 0=positive, 1=negative */
	u8 pre_mag;     /* Pre-cursor magnitude (0-15) */
	u8 pre_sign;    /* 0=positive, 1=negative */
	u8 post_mag;    /* Post-cursor magnitude (0-15) */
	u8 post_sign;   /* 0=positive, 1=negative */
};

/* Default FIR presets for each standard rate (based on Section 5.1 Backplane config) */
static const struct ds250_fir_preset fir_for_rate[] = {
	[RATE_10G3125 >> 4]  = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 2,   .pre_sign = 1,
				 .post_mag = 1,  .post_sign = 1 },
	[RATE_10G9375 >> 4]  = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 2,   .pre_sign = 1,
				 .post_mag = 1,  .post_sign = 1 },
	[RATE_12G5 >> 4]     = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 2,   .pre_sign = 1,
				 .post_mag = 1,  .post_sign = 1 },
	[3]                  = { 0 }, /* Reserved */
	[RATE_25G0 >> 4]     = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 3,   .pre_sign = 1,
				 .post_mag = 0,  .post_sign = 1 },
	[RATE_25G78125 >> 4] = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 3,   .pre_sign = 1,
				 .post_mag = 0,  .post_sign = 1 },
	[6]                  = { 0 }, /* Reserved */
	[RATE_28G125 >> 4]   = { .main_mag = 15, .main_sign = 0,
				 .pre_mag = 3,   .pre_sign = 1,
				 .post_mag = 0,  .post_sign = 1 },
};

static const struct ds250_fir_preset *ds250_get_fir_for_rate(u8 rate_val)
{
	u8 idx = (rate_val >> 4) & 0x07;
	return &fir_for_rate[idx];
}

struct ds250_channel_config {
	u8 rate_reg_val;
	struct ds250_fir_preset fir;
	bool fir_override;  /* true = user set FIR via sysfs, don't use default */
	u8 adapt_mode;  /* 0-3 */
	bool dfe_en;
};

struct ds250_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 num_channels;  /* 2, 4, or 8 */
	u8 device_id;
	struct ds250_channel_config ch_cfg[8];
};

/* Low-level I2C helpers */
static int ds250_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "Failed to read reg 0x%02x: %d\n", reg, ret);
	return ret;
}

static int ds250_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "Failed to write reg 0x%02x=0x%02x: %d\n",
			reg, val, ret);
	return ret;
}

/* Read-Modify-Write: only modify bits specified by mask */
static int ds250_rmw(struct i2c_client *client, u8 reg, u8 val, u8 mask)
{
	int orig;
	u8 new_val;

	orig = ds250_read_reg(client, reg);
	if (orig < 0)
		return orig;

	new_val = (orig & ~mask) | (val & mask);
	return ds250_write_reg(client, reg, new_val);
}

/* Channel Selection */
static int ds250_select_channel(struct ds250_data *data, int ch)
{
	struct i2c_client *client = data->client;
	int ret;

	/* Enable channel register access */
	ret = ds250_write_reg(client, REG_PAGE_SEL, 0x01);
	if (ret)
		return ret;

	/* Select specific channel */
	ret = ds250_write_reg(client, REG_CHAN_SEL, BIT(ch));
	return ret;
}

static int ds250_select_broadcast(struct ds250_data *data)
{
	struct i2c_client *client = data->client;
	u8 ch_mask = (1 << data->num_channels) - 1;
	int ret;

	/* Select all channels */
	ret = ds250_write_reg(client, REG_CHAN_SEL, ch_mask);
	if (ret)
		return ret;

	/* Enable channel register access + broadcast write */
	ret = ds250_write_reg(client, REG_PAGE_SEL, 0x03);
	return ret;
}

/* Write FIR registers (caller must hold lock and have selected channel/broadcast) */
static int __ds250_write_fir(struct i2c_client *client,
			     const struct ds250_fir_preset *fir)
{
	int ret;

	/* 0x3D = enable[7] + main_sign[6] + main_mag[4:0] */
	ret = ds250_rmw(client, REG_FIR_MAIN,
			0x80 | (fir->main_sign ? 0x40 : 0x00) | (fir->main_mag & 0x1F),
			0xDF);
	if (ret)
		return ret;

	/* 0x3E = pre_sign[6] + pre_mag[3:0] */
	ret = ds250_rmw(client, REG_FIR_PRE,
			(fir->pre_sign ? 0x40 : 0x00) | (fir->pre_mag & 0x0F),
			0x4F);
	if (ret)
		return ret;

	/* 0x3F = post_sign[6] + post_mag[3:0] */
	ret = ds250_rmw(client, REG_FIR_POST,
			(fir->post_sign ? 0x40 : 0x00) | (fir->post_mag & 0x0F),
			0x4F);
	return ret;
}

/* TX FIR Configuration (broadcast to all channels, with CDR retrain) */
static int ds250_set_fir_all(struct ds250_data *data,
			     const struct ds250_fir_preset *fir)
{
	struct i2c_client *client = data->client;
	int ret, i;

	mutex_lock(&data->lock);

	ret = ds250_select_broadcast(data);
	if (ret)
		goto out;

	/* Assert CDR reset before changing FIR */
	ret = ds250_rmw(client, REG_CDR_CTRL, 0x0C, 0x0C);
	if (ret)
		goto out;

	ret = __ds250_write_fir(client, fir);
	if (ret)
		goto out;

	/* Release CDR reset to retrain */
	ret = ds250_rmw(client, REG_CDR_CTRL, 0x00, 0x0C);
	if (ret)
		goto out;

	/* Update software state for all channels */
	for (i = 0; i < data->num_channels; i++)
		data->ch_cfg[i].fir = *fir;

out:
	mutex_unlock(&data->lock);
	return ret;
}

/* Combined Rate + FIR Configuration (broadcast to all channels) */
static int ds250_configure_all_channels(struct ds250_data *data,
					u8 rate_val, const struct ds250_fir_preset *fir)
{
	struct i2c_client *client = data->client;
	int ret, i;

	mutex_lock(&data->lock);

	ret = ds250_select_broadcast(data);
	if (ret)
		goto out;

	/* Reset channel registers */
	ret = ds250_rmw(client, REG_CH_RESET, 0x04, 0x04);
	if (ret)
		goto out;
	usleep_range(1000, 2000);

	/* Assert CDR reset */
	ret = ds250_rmw(client, REG_CDR_CTRL, 0x0C, 0x0C);
	if (ret)
		goto out;

	/* Set rate */
	ret = ds250_rmw(client, REG_RATE, rate_val, 0xF0);
	if (ret)
		goto out;

	/* Set adapt mode 2 (CTLE + DFE + CTLE) */
	ret = ds250_rmw(client, REG_ADAPT_MODE, 0x40, 0x60);
	if (ret)
		goto out;

	/* Enable DFE + taps 3-5 */
	ret = ds250_write_reg(client, REG_DFE_CTRL, 0xE3);
	if (ret)
		goto out;

	/* Set TX FIR */
	ret = __ds250_write_fir(client, fir);
	if (ret)
		goto out;

	/* Release CDR reset */
	ret = ds250_rmw(client, REG_CDR_CTRL, 0x00, 0x0C);
	if (ret)
		goto out;

	/* Update software state for all channels */
	for (i = 0; i < data->num_channels; i++) {
		data->ch_cfg[i].rate_reg_val = rate_val;
		data->ch_cfg[i].fir = *fir;
	}

out:
	mutex_unlock(&data->lock);
	return ret;
}

/* Sysfs Attributes */
/*
 * rate - Set/Get data rate for all channels
 *   Write: "25g" | "10g" | "12g5" | "25g78"
 *   Read:  current rate string
 */
static ssize_t rate_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	const struct ds250_fir_preset *fir;
	u8 rate_val;
	int ret;

	if (sysfs_streq(buf, "10g3125"))
		rate_val = RATE_10G3125;
	else if (sysfs_streq(buf, "10g9375"))
		rate_val = RATE_10G9375;
	else if (sysfs_streq(buf, "12g5"))
		rate_val = RATE_12G5;
	else if (sysfs_streq(buf, "25g0"))
		rate_val = RATE_25G0;
	else if (sysfs_streq(buf, "25g78125"))
		rate_val = RATE_25G78125;
	else if (sysfs_streq(buf, "28g125")) {
		if (data->device_id != 0x13) {
			dev_err(dev, "28.125 Gbps only supported on DS280DF810\n");
			return -EINVAL;
		}
		rate_val = RATE_28G125;
	} else
		return -EINVAL;

	/* Use user-overridden FIR if set, otherwise default for rate */
	if (data->ch_cfg[0].fir_override)
		fir = &data->ch_cfg[0].fir;
	else
		fir = ds250_get_fir_for_rate(rate_val);

	/* Apply to all channels via broadcast */
	ret = ds250_configure_all_channels(data, rate_val, fir);
	if (ret)
		return ret;

	return count;
}

static ssize_t rate_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	const char *rate_str;

	switch (data->ch_cfg[0].rate_reg_val) {
	case RATE_10G3125:  rate_str = "10g3125"; break;
	case RATE_10G9375:  rate_str = "10g9375"; break;
	case RATE_12G5:     rate_str = "12g5"; break;
	case RATE_25G0:     rate_str = "25g0"; break;
	case RATE_25G78125: rate_str = "25g78125"; break;
	case RATE_28G125:   rate_str = "28g125"; break;
	default:            rate_str = "unknown"; break;
	}

	return sprintf(buf, "%s\n", rate_str);
}
static DEVICE_ATTR_RW(rate);

/*
 * tx_fir - Override TX FIR for all channels
 *   Write: "main,pre,post" using signed integers
 *          e.g., "11,-3,0" = main=+11, pre=-3, post=-0
 *          Bare 0 = negative, +0 = positive (per TI convention)
 *          Write "default" to clear override and use rate-based default
 *   Read:  current FIR settings + whether override is active
 */
static ssize_t tx_fir_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	struct ds250_fir_preset fir;
	int ret, i;
	int main_val, pre_val, post_val;
	const char *p;

	if (sysfs_streq(buf, "default")) {
		const struct ds250_fir_preset *def_fir;

		def_fir = ds250_get_fir_for_rate(data->ch_cfg[0].rate_reg_val);

		/* Apply default FIR to all channels via broadcast */
		ret = ds250_set_fir_all(data, def_fir);
		if (ret)
			return ret;

		mutex_lock(&data->lock);
		for (i = 0; i < data->num_channels; i++)
			data->ch_cfg[i].fir_override = false;
		mutex_unlock(&data->lock);
		return count;
	}

	/*
	 * Parse "main,pre,post" — each is a signed integer.
	 * Special case: bare "0" → negative, "+0" → positive.
	 * We scan the raw string to detect "+0" before sscanf.
	 */
	ret = sscanf(buf, "%d,%d,%d", &main_val, &pre_val, &post_val);
	if (ret != 3)
		return -EINVAL;

	/* Main: range -31..+31 */
	if (main_val > 31 || main_val < -31)
		return -EINVAL;
	if (main_val >= 0) {
		fir.main_sign = 0;
		fir.main_mag = main_val;
	} else {
		fir.main_sign = 1;
		fir.main_mag = -main_val;
	}

	/* Pre: range -15..+15 */
	if (pre_val > 15 || pre_val < -15)
		return -EINVAL;
	if (pre_val > 0) {
		fir.pre_sign = 0;
		fir.pre_mag = pre_val;
	} else if (pre_val < 0) {
		fir.pre_sign = 1;
		fir.pre_mag = -pre_val;
	} else {
		/* pre_val == 0: check for explicit '+' in input */
		p = strchr(buf, ',');
		if (p) {
			p++;
			while (*p == ' ')
				p++;
			fir.pre_sign = (*p == '+') ? 0 : 1;
		} else {
			fir.pre_sign = 1;
		}
		fir.pre_mag = 0;
	}

	/* Post: range -15..+15 */
	if (post_val > 15 || post_val < -15)
		return -EINVAL;
	if (post_val > 0) {
		fir.post_sign = 0;
		fir.post_mag = post_val;
	} else if (post_val < 0) {
		fir.post_sign = 1;
		fir.post_mag = -post_val;
	} else {
		/* post_val == 0: check for explicit '+' in second comma */
		p = strchr(buf, ',');
		if (p)
			p = strchr(p + 1, ',');
		if (p) {
			p++;
			while (*p == ' ')
				p++;
			fir.post_sign = (*p == '+') ? 0 : 1;
		} else {
			fir.post_sign = 1;
		}
		fir.post_mag = 0;
	}

	/* Main val == 0: check for explicit '+' at start */
	if (main_val == 0) {
		p = buf;
		while (*p == ' ')
			p++;
		fir.main_sign = (*p == '+') ? 0 : 1;
	}

	/* Apply to all channels via broadcast */
	ret = ds250_set_fir_all(data, &fir);
	if (ret)
		return ret;

	mutex_lock(&data->lock);
	for (i = 0; i < data->num_channels; i++)
		data->ch_cfg[i].fir_override = true;
	mutex_unlock(&data->lock);

	return count;
}

static ssize_t tx_fir_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	struct ds250_fir_preset *fir = &data->ch_cfg[0].fir;

	return sprintf(buf, "main=%c%u pre=%c%u post=%c%u [%s]\n",
		       fir->main_sign ? '-' : '+', fir->main_mag,
		       fir->pre_sign ? '-' : '+', fir->pre_mag,
		       fir->post_sign ? '-' : '+', fir->post_mag,
		       data->ch_cfg[0].fir_override ? "override" : "default");
}
static DEVICE_ATTR_RW(tx_fir);

/*
 * status - Read CDR lock and signal detect status for all channels
 */
static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret, val, i, len = 0;

	mutex_lock(&data->lock);
	for (i = 0; i < data->num_channels; i++) {
		ret = ds250_select_channel(data, i);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}

		val = ds250_read_reg(client, REG_SD_LOCK_STATUS);
		if (val < 0) {
			mutex_unlock(&data->lock);
			return val;
		}

		len += sprintf(buf + len, "ch%d: signal_detect=%d cdr_lock=%d\n",
			       i, (val >> 5) & 1, (val >> 4) & 1);
	}
	mutex_unlock(&data->lock);

	return len;
}
static DEVICE_ATTR_RO(status);

/*
 * eye - Read HEO/VEO for all channels
 */
static ssize_t eye_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ds250_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret, heo, veo, i, len = 0;

	mutex_lock(&data->lock);
	for (i = 0; i < data->num_channels; i++) {
		ret = ds250_select_channel(data, i);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}

		heo = ds250_read_reg(client, REG_HEO);
		veo = ds250_read_reg(client, REG_VEO);
		if (heo < 0 || veo < 0) {
			mutex_unlock(&data->lock);
			return -EIO;
		}

		len += sprintf(buf + len, "ch%d: HEO=%d/32UI VEO=%d*3.125mV\n",
			       i, heo, veo);
	}
	mutex_unlock(&data->lock);

	return len;
}
static DEVICE_ATTR_RO(eye);

static struct attribute *ds250_attrs[] = {
	&dev_attr_rate.attr,
	&dev_attr_tx_fir.attr,
	&dev_attr_status.attr,
	&dev_attr_eye.attr,
	NULL,
};

static const struct attribute_group ds250_attr_group = {
	.attrs = ds250_attrs,
};

static const struct attribute_group *ds250_attr_groups[] = {
	&ds250_attr_group,
	NULL,
};

/* Probe */
static int ds250_probe(struct i2c_client *client)
{
	struct ds250_data *data;
	int device_id, vendor_id;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);

	/* Verify device */
	vendor_id = ds250_read_reg(client, REG_VENDOR_ID);
	if (vendor_id < 0)
		return vendor_id;

	if (vendor_id != 0x03) {
		dev_err(&client->dev, "Invalid vendor ID: 0x%02x (expected 0x03)\n",
			vendor_id);
		return -ENODEV;
	}

	device_id = ds250_read_reg(client, REG_DEVICE_ID);
	if (device_id < 0)
		return device_id;

	switch (device_id) {
	case 0x10: /* DS250DF810/DF410/DF210 */
		data->num_channels = 8;
		break;
	case 0x13: /* DS280DF810 */
		data->num_channels = 8;
		break;
	case 0x15: /* DS250DF230 */
		data->num_channels = 2;
		break;
	default:
		dev_warn(&client->dev, "Unknown device ID: 0x%02x\n", device_id);
		data->num_channels = 8;
		break;
	}

	data->device_id = device_id;

	/* Set default config: 25.78125 Gbps */
	for (int i = 0; i < data->num_channels; i++) {
		data->ch_cfg[i].rate_reg_val = RATE_25G78125;
		data->ch_cfg[i].fir = *ds250_get_fir_for_rate(RATE_25G78125);
		data->ch_cfg[i].adapt_mode = 2;
		data->ch_cfg[i].dfe_en = true;
	}

	dev_info(&client->dev,
		 "DS2x0DFxx0 retimer detected (device_id=0x%02x, %d channels)\n",
		 device_id, data->num_channels);

	return 0;
}

static const struct i2c_device_id ds250_id[] = {
	{ "ds250df810", 0 },
	{ "ds280df810", 0 },
	{ "ds250df230", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds250_id);

static const struct of_device_id ds250_of_match[] = {
	{ .compatible = "ti,ds250df810" },
	{ .compatible = "ti,ds280df810" },
	{ .compatible = "ti,ds250df230" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds250_of_match);

static struct i2c_driver ds250_driver = {
	.driver = {
		.name = "ds250df810",
		.of_match_table = ds250_of_match,
		.dev_groups = ds250_attr_groups,
	},
	.probe = ds250_probe,
	.id_table = ds250_id,
};
module_i2c_driver(ds250_driver);

MODULE_AUTHOR("Accton Technology");
MODULE_DESCRIPTION("TI DS2x0DFxx0 Retimer I2C Driver");
MODULE_LICENSE("GPL");
