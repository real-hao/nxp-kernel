// SPDX-License-Identifier: GPL-2.0
/*
 * AP3216C Ambient Light and Infrared Proximity Sensor
 *
 * Copyright (c) 2019, Robert Eshleman.
 *
 * Datasheet: https://pdf-datasheet-datasheet.netdna-ssl.com/pdf-down/A/P/3/AP3216C-LITE-ON.pdf
 *
 * 7-bit I2C slave address 0x1E
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/bits.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#define AP3216C_DRV_NAME "ap3216c"

/* register addresses */
#define AP3216C_SYS 0x0
#define AP3216C_INT_STATUS 0x01
#define AP3216C_INT_CLR 0x02
#define AP3216C_IR_DATA_LO 0x0A
#define AP3216C_IR_DATA_HI 0x0B
#define AP3216C_ALS_DATA_LO 0x0C
#define AP3216C_ALS_DATA_HI 0x0D
#define AP3216C_PS_DATA_LO 0x0E
#define AP3216C_PS_DATA_HI 0x0F
#define AP3216C_ALS_CFG 0x10
#define AP3216C_ALS_CALIB 0x19
#define AP3216C_ALS_LO_THR_LO 0x1A
#define AP3216C_ALS_LO_THR_HI 0x1B
#define AP3216C_ALS_HI_THR_LO 0x1C
#define AP3216C_ALS_HI_THR_HI 0x1D
#define AP3216C_PS_CFG 0x20
#define AP3216C_PS_CALIB_LO 0x28
#define AP3216C_PS_CALIB_HI 0x29
#define AP3216C_PS_LO_THR_LO 0x2A
#define AP3216C_PS_LO_THR_HI 0x2B
#define AP3216C_PS_HI_THR_LO 0x2C
#define AP3216C_PS_HI_THR_HI 0x2D

/* SYS_MODE mask and config value */
#define AP3216C_SYS_MODE_ALS_PS 0x3

/* INT_STATUS masks and config value */
#define AP3216C_INT_STATUS_ALS_MASK BIT(0)
#define AP3216C_INT_STATUS_PS_MASK BIT(1)
#define AP3216C_INT_STATUS_CLR GENMASK(1, 0)
#define AP3216C_INT_ALS_CLR BIT(0)
#define AP3216C_INT_PS_CLR BIT(1)
#define AP3216C_INT_CLR_MANUAL 1

/* IR_DATA mask/shift */
#define AP3216C_IR_DATA_LO_MASK GENMASK(1, 0)
#define AP3216C_IR_DATA_HI_SHIFT 2

/* ALS_DATA shift and fractional helper */
#define AP3216C_ALS_DATA_HI_SHIFT 8
#define AP3216C_ALS_DATA_DENOM 100000

/* ALS_CALIB masks/shifts */
#define AP3216C_ALS_CALIB_INT_MASK GENMASK(7, 6)
#define AP3216C_ALS_CALIB_INT_SHIFT 6
#define AP3216C_ALS_CALIB_DEC_MASK GENMASK(5, 0)

/* PS_DATA shifts/masks/bits */
#define AP3216C_PS_DATA_LO_MASK GENMASK(3, 0)
#define AP3216C_PS_DATA_LO_IR_OF BIT(6)
#define AP3216C_PS_DATA_HI_MASK GENMASK(5, 0)
#define AP3216C_PS_DATA_HI_SHIFT 4
#define AP3216C_PS_DATA_HI_IR_OF BIT(6)

/* ALS_CFG masks */
#define AP3216C_ALS_CFG_GAIN_MASK GENMASK(5, 4)
#define AP3216C_ALS_CFG_GAIN_SHIFT 4

/* ALS_CALIB shifts */
#define AP3216C_ALS_CALIB_INT_SHIFT 6

/* ALS_HI_THR masks and shifts */
#define AP3216C_ALS_HI_THR_LO_MASK GENMASK(7, 0)
#define AP3216C_ALS_HI_THR_HI_SHIFT 8
#define AP3216C_ALS_HI_THR_HI_MASK GENMASK(16, 8)
#define AP3216C_ALS_HI_THR_HI_READ_MASK GENMASK(7, 0)
#define AP3216C_ALS_HI_THR_HI_SHIFT 8

#define AP3216C_ALS_THR_HI_SHIFT 8

/* ALS_LO_THR masks and shifts */
#define AP3216C_ALS_LO_THR_LO_MASK GENMASK(7, 0)
#define AP3216C_ALS_LO_THR_HI_MASK GENMASK(16, 8)
#define AP3216C_ALS_LO_THR_HI_SHIFT 8

/* PS_CFG reg mask/shift/bit values */
#define AP3216C_PS_CFG_GAIN_MASK GENMASK(3, 2)
#define AP3216C_PS_CFG_GAIN_SHIFT 2
#define AP3216C_PS_CFG_GAIN(val) \
	(1 << ((val & AP3216C_PS_CFG_GAIN_MASK) >> AP3216C_PS_CFG_GAIN_SHIFT))

/* PS_LO and PS_HI multiplier values */
#define AP3216C_PS_LO_THR_HI_MULT 4
#define AP3216C_PS_HI_THR_HI_MULT 4
#define AP3216C_PS_THR_HI_MULT 4

/* PS_THR_LO and PS_THR_HI masks/shifts */
#define AP3216C_PS_THR_LO_MASK GENMASK(1, 0)
#define AP3216C_PS_THR_HI_MASK GENMASK(10, 2)
#define AP3216C_PS_THR_HI_SHIFT 2

/* PS_CALIB_HI and PS_CALIB_LO shift/mask/bit values */
#define AP3216C_PS_CALIB_HI_SHIFT 1
#define AP3216C_PS_CALIB_HI_MASK GENMASK(8, 1)
#define AP3216C_PS_CALIB_LO_MASK BIT(0)

/* ALS fractional helper values */
#define AP3216C_ALS_INTEG_MULT 1000
#define AP3216C_ALS_FRACT_DIV 100

/* ALS_CALIB min/max values (0.0 <= ALS_CALIB <= 3.984375) */
#define AP3216C_ALS_CALIB_INT_MIN 0
#define AP3216C_ALS_CALIB_DEC_MIN 0
#define AP3216C_ALS_CALIB_INT_MAX 3
#define AP3216C_ALS_CALIB_DEC_MAX 984375

/* ALS_CALIB conversion denominator */
#define AP3216C_ALS_CALIB_DENOM 64

#define AP3216C_IIO_MULT 1000000
#define AP3216C_PS_THR_MAX (BIT(10) - 1)

/* See ALS configuration ALS_GAIN field in datasheet */
#define AP3216C_ALS_THR_MAX 20661

static const u16 ap3216c_gain_array[] = {
	3500,  /* 0.3500 lux resolution */
	 788,  /* 0.0788 lux resolution */
	 197,  /* 0.0197 lux resolution */
	  49,  /* 0.0049 lux resolution */
};

static const u16 ap3216c_als_max_thr_array[] = {
	20661,  /* max thr when gain is 0.3500 */
	 5162,  /* max thr when gain is 0.0788 */
	 1291,  /* max thr when gain is 0.0197 */
	  323,  /* max thr when gain is 0.0049*/
};

struct ap3216c_data {
	struct regmap *regmap;
	struct i2c_client *client;
	struct mutex lock;
	bool als_thresh_en;
	bool prox_thresh_en;
};

static const struct iio_event_spec ap3216c_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_shared_by_type = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec ap3216c_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_CALIBSCALE),
		.event_spec = ap3216c_event_spec,
		.num_event_specs = ARRAY_SIZE(ap3216c_event_spec),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS),
		.event_spec = ap3216c_event_spec,
		.num_event_specs = ARRAY_SIZE(ap3216c_event_spec),
	},
	{
		.type = IIO_LIGHT,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int ap3216c_read_gain(struct ap3216c_data *data, u16 *gain)
{
	unsigned int cfg;
	int ret;

	ret = regmap_read(data->regmap, AP3216C_ALS_CFG, &cfg);
	if (ret < 0)
		return ret;

	*gain = ap3216c_gain_array[(cfg & AP3216C_ALS_CFG_GAIN_MASK) >>
				    AP3216C_ALS_CFG_GAIN_SHIFT];
	return 0;
}

static int ap3216c_read_max_thr(struct ap3216c_data *data, u16 *max_thr)
{
	unsigned int cfg;
	int ret;

	ret = regmap_read(data->regmap, AP3216C_ALS_CFG, &cfg);
	if (ret < 0)
		return ret;

	/* max thr is derived from the gain bits of ALS_CFG */
	*max_thr = ap3216c_als_max_thr_array[
			(cfg & AP3216C_ALS_CFG_GAIN_MASK) >>
			 AP3216C_ALS_CFG_GAIN_SHIFT];
	return 0;
}

static int ap3216c_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	u8 vals[2];
	int integral, fractional;
	u16 thr, max_thr, gain;
	int ret;

	if (val < 0 || val2 < 0)
		return -EINVAL;

	switch (chan->type) {
	case IIO_LIGHT:
		ret = ap3216c_read_max_thr(data, &max_thr);
		if (ret < 0)
			return ret;

		if (val > max_thr)
			val = max_thr;

		ret = ap3216c_read_gain(data, &gain);
		if (ret < 0)
			return ret;

		integral = val * AP3216C_ALS_INTEG_MULT / gain;
		fractional = val2 / AP3216C_ALS_FRACT_DIV / gain;

		thr = integral + fractional;

		switch (dir) {
		case IIO_EV_DIR_RISING:
			vals[0] = thr & AP3216C_ALS_HI_THR_LO_MASK;
			vals[1] = (thr & AP3216C_ALS_HI_THR_HI_MASK) >>
				   AP3216C_ALS_HI_THR_HI_SHIFT;

			return regmap_bulk_write(data->regmap,
						 AP3216C_ALS_HI_THR_LO,
						 vals, 2);

		case IIO_EV_DIR_FALLING:
			vals[0] = thr & AP3216C_ALS_LO_THR_LO_MASK;
			vals[1] = (thr & AP3216C_ALS_LO_THR_HI_MASK) >>
				   AP3216C_ALS_LO_THR_HI_SHIFT;

			return regmap_bulk_write(data->regmap,
						 AP3216C_ALS_LO_THR_LO,
						 vals, 2);
		default:
			return -EINVAL;
		}


	case IIO_PROXIMITY:
		if (val > AP3216C_PS_THR_MAX)
			val = AP3216C_PS_THR_MAX;

		vals[0] = val & AP3216C_PS_THR_LO_MASK;
		vals[1] = (val & AP3216C_PS_THR_HI_MASK) >>
			   AP3216C_PS_THR_HI_SHIFT;

		switch (dir) {
		case IIO_EV_DIR_RISING:
			return regmap_bulk_write(data->regmap,
						 AP3216C_PS_HI_THR_LO,
						 vals, 2);

		case IIO_EV_DIR_FALLING:
			return regmap_bulk_write(data->regmap,
						 AP3216C_PS_LO_THR_LO,
						 vals, 2);
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int ap3216c_get_als_reg(enum iio_event_direction dir)
{
	switch (dir) {
	case IIO_EV_DIR_RISING:
		return AP3216C_ALS_HI_THR_LO;

	case IIO_EV_DIR_FALLING:
		return AP3216C_ALS_LO_THR_LO;

	default:
		return -EINVAL;
	}
}

static int ap3216c_get_ps_reg(enum iio_event_direction dir)
{
	switch (dir) {
	case IIO_EV_DIR_RISING:
		return AP3216C_PS_HI_THR_LO;

	case IIO_EV_DIR_FALLING:
		return AP3216C_PS_LO_THR_LO;

	default:
		return -EINVAL;
	}
}

static int ap3216c_read_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	u16 gain;
	u8 vals[2];
	int reg, ret;

	switch (chan->type) {
	case IIO_LIGHT:
		reg = ap3216c_get_als_reg(dir);
		if (reg < 0)
			return reg;

		ret = regmap_bulk_read(data->regmap, reg, vals, 2);
		if (ret < 0)
			return ret;

		ret = ap3216c_read_gain(data, &gain);
		if (ret < 0)
			return ret;

		*val = ((vals[1] << AP3216C_ALS_THR_HI_SHIFT) | vals[0]) *
			 gain / AP3216C_ALS_INTEG_MULT;
		return IIO_VAL_INT;

	case IIO_PROXIMITY:
		reg = ap3216c_get_ps_reg(dir);
		if (reg < 0)
			return reg;

		ret = regmap_bulk_read(data->regmap, reg, vals, 2);
		if (ret < 0)
			return ret;

		*val = (vals[1] << 2) | (vals[0] & AP3216C_PS_THR_LO_MASK);

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ap3216c_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	switch (chan->type) {
	case IIO_LIGHT:
		ret = data->als_thresh_en;
		break;

	case IIO_PROXIMITY:
		ret = data->prox_thresh_en;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int ap3216c_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir, int state)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	switch (chan->type) {
	case IIO_LIGHT:
		data->als_thresh_en = state;
		break;

	case IIO_PROXIMITY:
		data->prox_thresh_en = state;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static const struct regmap_config ap3216c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AP3216C_PS_HI_THR_HI,
};

/**
 * Returns integral part of decimal between 0.0 and 3.984275
 */
static int ap3216c_als_calibscale_int(int val)
{
	if (val > AP3216C_ALS_CALIB_INT_MAX)
		return AP3216C_ALS_CALIB_INT_MAX;

	if (val <= AP3216C_ALS_CALIB_INT_MIN)
		return AP3216C_ALS_CALIB_INT_MIN;

	return val;
}

/**
 * Returns decimal part of decimal number between 0.0 and 3.984275
 */
static int ap3216c_als_calibscale_dec(int val, int val2)
{
	/* Return max decimal if number exceeds calibscale max */
	if (val > AP3216C_ALS_CALIB_INT_MAX ||
	    (val == AP3216C_ALS_CALIB_INT_MAX &&
	     val2 > AP3216C_ALS_CALIB_DEC_MAX))
		return AP3216C_ALS_CALIB_DEC_MAX;

	/* Floor the decimal if integral below minimum */
	if (val <= AP3216C_ALS_CALIB_INT_MIN)
		return AP3216C_ALS_CALIB_DEC_MIN;

	/* Floor the decimal if decimal below minimum */
	if (val2 < AP3216C_ALS_CALIB_DEC_MIN)
		return AP3216C_ALS_CALIB_DEC_MIN;

	/* Return max decimal if decimal above maximum */
	if (val2 > AP3216C_ALS_CALIB_DEC_MAX)
		return AP3216C_ALS_CALIB_DEC_MAX;

	return val2;
}

static int ap3216c_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	int integral, decimal;
	u8 vals[2];
	int tmp;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			integral = ap3216c_als_calibscale_int(val);
			decimal = ap3216c_als_calibscale_dec(val, val2);

			tmp = integral << AP3216C_ALS_CALIB_INT_SHIFT;

			/*
			 * Reverse scaling by multiplying by the scaling
			 * denominator and dividing by IIO multiplier.
			 */
			tmp |= (decimal * AP3216C_ALS_CALIB_DENOM) /
				AP3216C_IIO_MULT;

			return regmap_write(data->regmap,
					    AP3216C_ALS_CALIB, tmp);

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_PROXIMITY:
			vals[0] = val & AP3216C_PS_CALIB_LO_MASK;
			vals[1] = (val & AP3216C_PS_CALIB_HI_MASK) >>
				   AP3216C_PS_CALIB_HI_SHIFT;

			return regmap_bulk_write(data->regmap,
						 AP3216C_PS_CALIB_LO,
						 vals, 2);

		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ap3216c_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ap3216c_data *data = iio_priv(indio_dev);
	u16 gain;
	u8 vals[2];
	int cfg, tmp;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBSCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = regmap_read(data->regmap,
					  AP3216C_ALS_CALIB, &tmp);
			if (ret < 0)
				return ret;

			/* Scale is multiplied by 1/64 */
			*val = tmp;
			*val2 = AP3216C_ALS_CALIB_DENOM;

			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_PROXIMITY:
			ret = regmap_bulk_read(data->regmap,
					       AP3216C_PS_CALIB_LO,
					       vals, 2);
			if (ret < 0)
				return ret;

			*val = (vals[0] & AP3216C_PS_CALIB_LO_MASK) |
			       ((vals[1] & AP3216C_PS_CALIB_HI_MASK) <<
				AP3216C_PS_CALIB_HI_SHIFT);

			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = regmap_bulk_read(data->regmap,
					       AP3216C_ALS_DATA_LO,
					       vals, 2);
			if (ret < 0)
				return ret;

			ret = ap3216c_read_gain(data, &gain);
			if (ret < 0)
				return ret;

			*val = ((vals[1] << AP3216C_ALS_DATA_HI_SHIFT) |
				 vals[0]) * gain;
			*val2 = AP3216C_ALS_DATA_DENOM;

			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			if (chan->channel2 != IIO_MOD_LIGHT_IR)
				return -EINVAL;

			ret = regmap_bulk_read(data->regmap,
					       AP3216C_IR_DATA_LO,
					       vals, 2);

			*val = (vals[0] & AP3216C_IR_DATA_LO_MASK) |
			       (vals[1] << AP3216C_IR_DATA_HI_SHIFT);


			return IIO_VAL_INT;

		case IIO_PROXIMITY:
			ret = regmap_bulk_read(data->regmap,
					       AP3216C_PS_DATA_LO,
					       vals, 2);
			if (ret < 0)
				return ret;

			ret = regmap_read(data->regmap,
					  AP3216C_PS_CFG, &cfg);
			if (ret < 0)
				return ret;

			tmp = vals[0] & AP3216C_PS_DATA_LO_MASK;
			tmp |= (vals[1] & AP3216C_PS_DATA_HI_MASK) <<
				AP3216C_PS_DATA_HI_SHIFT;
			*val = tmp * AP3216C_PS_CFG_GAIN(cfg);

			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_info ap3216c_info = {
	.read_raw = ap3216c_read_raw,
	.write_raw = ap3216c_write_raw,
	.read_event_value = ap3216c_read_event_value,
	.write_event_value = ap3216c_write_event_value,
	.read_event_config = ap3216c_read_event_config,
	.write_event_config = ap3216c_write_event_config,
};

static irqreturn_t ap3216c_event_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct ap3216c_data *data = iio_priv(indio_dev);
	int status;
	s64 timestamp;
	int ret;

	mutex_lock(&data->lock);
	ret = regmap_read(data->regmap, AP3216C_INT_STATUS, &status);
	if (ret) {
		dev_err(&data->client->dev, "error reading IRQ status\n");
		goto done;
	}

	/* The IRQ was not from this device */
	if (!status) {
		mutex_unlock(&data->lock);
		return IRQ_NONE;
	}

	timestamp = iio_get_time_ns(indio_dev);
	if (status & AP3216C_INT_STATUS_PS_MASK) {
		if (data->prox_thresh_en)
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY, 0,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);

		ret = regmap_write(data->regmap,
				   AP3216C_INT_STATUS, AP3216C_INT_PS_CLR);
		if (ret < 0)
			dev_err(&data->client->dev, "error clearing PS interrupt\n");
	}

	if (status & AP3216C_INT_STATUS_ALS_MASK) {
		if (data->als_thresh_en)
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_EITHER),
				       timestamp);

		ret = regmap_write(data->regmap,
				   AP3216C_INT_STATUS, AP3216C_INT_ALS_CLR);
		if (ret < 0)
			dev_err(&data->client->dev, "error clearing ALS interrupt\n");
	}

done:
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

static int ap3216c_probe(struct i2c_client *client)
{
	struct ap3216c_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &ap3216c_info;
	indio_dev->name = AP3216C_DRV_NAME;
	indio_dev->channels = ap3216c_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap3216c_channels);

	data->regmap = devm_regmap_init_i2c(client, &ap3216c_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	/* Default to thresh events disabled */
	data->als_thresh_en = false;
	data->prox_thresh_en = false;

	/*
	 * Require that that the interrupt is cleared only when the INT_STATUS
	 * register is written to, instead of when data is read.
	 */
	ret = regmap_write(data->regmap,
			   AP3216C_INT_CLR, AP3216C_INT_CLR_MANUAL);
	if (ret < 0)
		return ret;

	ret = regmap_write(data->regmap, AP3216C_INT_STATUS,
			   AP3216C_INT_PS_CLR | AP3216C_INT_ALS_CLR);
	if (ret < 0) {
		dev_err(&data->client->dev, "error clearing interrupts\n");
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, ap3216c_event_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_SHARED | IRQF_ONESHOT,
						client->name, indio_dev);
		if (ret)
			return ret;
	}

	/* Enable ALS and PS+IR */
	ret = regmap_write(data->regmap, AP3216C_SYS, AP3216C_SYS_MODE_ALS_PS);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id ap3216c_of_match[] = {
	{ .compatible = "liteon,ap3216c", },
	{ },
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static const struct i2c_device_id ap3216c_id[] = {
	{"ap3216c", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static struct i2c_driver ap3216c_driver = {
	.driver = {
		.name	= AP3216C_DRV_NAME,
	},
	.probe		= ap3216c_probe,
	.id_table	= ap3216c_id,
};
module_i2c_driver(ap3216c_driver);

MODULE_AUTHOR("Robert Eshleman <bobbyeshleman@gmail.com>");
MODULE_DESCRIPTION("AP3216C Ambient Light and Proximity Sensor");
MODULE_LICENSE("GPL v2");
