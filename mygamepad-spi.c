/************************************************************************
 * License-Identifier: GPL-2.0 	    									
 ************************************************************************
 * Copyright (C) 2023 JeffZen-TW <jeff810123@gmail.com>
 * 
 * The project is a simple kernel driver for adc joystick and keyboard.
 * And this project modify from two file in linux kernel after v6.5 as follow.
 * 
 ************************************************************************
 * linux/drivers/iio/adc/mcp320x.c
 ************************************************************************
 * Copyright (C) 2013 Oskar Andero <oskar.andero@gmail.com>
 * Copyright (C) 2014 Rose Technology
 * 	   Allan Bendorff Jensen <abj@rosetechnology.dk>
 *	   Soren Andersen <san@rosetechnology.dk>
 *
 * Driver for following ADC chips from Microchip Technology's:
 * 10 Bit converter
 * MCP3001
 * MCP3002
 * MCP3004
 * MCP3008
 * ------------
 * 12 bit converter
 * MCP3201
 * MCP3202
 * MCP3204
 * MCP3208
 * ------------
 *
 * Datasheet can be found here:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21293C.pdf  mcp3001
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21294E.pdf  mcp3002
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21295d.pdf  mcp3004/08
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21290D.pdf  mcp3201
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21034D.pdf  mcp3202
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf  mcp3204/08
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21700E.pdf  mcp3301
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ************************************************************************
 * linux/drivers/input/joystick/psxpad-spi.c
 * ************************************************************************
 * PlayStation 1/2 joypads via SPI interface Driver
 *
 * Copyright (C) 2017 Tomohiro Yoshidomi <sylph23k@gmail.com>
 *
 * PlayStation 1/2 joypad's plug (not socket)
 *  123 456 789
 * (...|...|...)
 *
 * 1: DAT -> MISO (pullup with 1k owm to 3.3V)
 * 2: CMD -> MOSI
 * 3: 9V (for motor, if not use N.C.)
 * 4: GND
 * 5: 3.3V
 * 6: Attention -> CS(SS)
 * 7: SCK -> SCK
 * 8: N.C.
 * 9: ACK -> N.C.
 */
#if 1
  #define dprint(fmt,s...) printk("mcp3008_my: %s,%d:"fmt,__FUNCTION__,__LINE__,##s)
#else
  #define dprint(fmt,s...)
#endif  

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>

enum {
	mcp3001,
	mcp3002,
	mcp3004,
	mcp3008,
	mcp3201,
	mcp3202,
	mcp3204,
	mcp3208,
	mcp3301,
};

struct mcp320x_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int resolution;
};

struct mcp320x {
	struct spi_device *spi;
	struct spi_message msg;
	struct spi_transfer transfer[2];
	struct input_dev *idev;
	int    device_index;
	struct regulator *reg;
	struct mutex lock;
	const struct mcp320x_chip_info *chip_info;

	u8 tx_buf ____cacheline_aligned;
	u8 rx_buf[2];
};

// typedef enum adckey ADCKey;
// enum adckey {
// 	S1,
// 	S2,
// 	S3,
// 	S4,
// 	S5,
// };

// typedef struct _point {
// 	uint x;
// 	uint y;
// } Point;

// typedef struct _mygamepad {
// 	Point axisRaw;
// 	ADCKey adcKey;
// } MyGamePad;

static int mcp320x_channel_to_tx_data(int device_index,
			const unsigned int channel, bool differential)
{
	int start_bit = 1;

	switch (device_index) {
	case mcp3001:
	case mcp3201:
	case mcp3301:
		return 0;
	case mcp3002:
	case mcp3202:
		return ((start_bit << 4) | (!differential << 3) |
							(channel << 2));
	case mcp3004:
	case mcp3204:
	case mcp3008:
	case mcp3208:
		return ((start_bit << 6) | (!differential << 5) |
							(channel << 2));
	default:
		return -EINVAL;
	}
}

static int mcp320x_adc_conversion(struct mcp320x *adc, u8 channel,
				  bool differential, int device_index)
{
	int ret;

	adc->rx_buf[0] = 0;
	adc->rx_buf[1] = 0;
	adc->tx_buf = mcp320x_channel_to_tx_data(device_index,
						channel, differential);

	if (device_index != mcp3001 && device_index != mcp3201 && device_index != mcp3301) {
		ret = spi_sync(adc->spi, &adc->msg);
		if (ret < 0)
			return ret;
	} else {
		ret = spi_read(adc->spi, &adc->rx_buf, sizeof(adc->rx_buf));
		if (ret < 0)
			return ret;
	}

	switch (device_index) {
	case mcp3001:
		return (adc->rx_buf[0] << 5 | adc->rx_buf[1] >> 3);
	case mcp3002:
	case mcp3004:
	case mcp3008:
		return (adc->rx_buf[0] << 2 | adc->rx_buf[1] >> 6);
	case mcp3201:
		return (adc->rx_buf[0] << 7 | adc->rx_buf[1] >> 1);
	case mcp3202:
	case mcp3204:
	case mcp3208:
		return (adc->rx_buf[0] << 4 | adc->rx_buf[1] >> 4);
	case mcp3301:
		return sign_extend32((adc->rx_buf[0] & 0x1f) << 8 | adc->rx_buf[1], 12);
	default:
		return -EINVAL;
	}
}


static int mcp320x_read_raw(struct input_dev *input,
			    struct iio_chan_spec const *channel, int *val)
{
	struct mcp320x *adc = input_get_drvdata(input);
	int ret = -EINVAL;
	int device_index = adc->device_index;

	mutex_lock(&adc->lock);

	ret = mcp320x_adc_conversion(adc, channel->address,
		channel->differential, device_index);

	if (ret < 0)
		goto out;

	*val = ret;
	ret = IIO_VAL_INT;

out:
	mutex_unlock(&adc->lock);

	return ret;
}


#define MCP320X_VOLTAGE_CHANNEL(num)				\
	{							\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = (num),				\
		.address = (num),				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

#define MCP320X_VOLTAGE_CHANNEL_DIFF(num)			\
	{							\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = (num * 2),				\
		.channel2 = (num * 2 + 1),			\
		.address = (num * 2),				\
		.differential = 1,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
	}

static const struct iio_chan_spec mcp3201_channels[] = {
	MCP320X_VOLTAGE_CHANNEL_DIFF(0),
};

static const struct iio_chan_spec mcp3202_channels[] = {
	MCP320X_VOLTAGE_CHANNEL(0),
	MCP320X_VOLTAGE_CHANNEL(1),
	MCP320X_VOLTAGE_CHANNEL_DIFF(0),
};

static const struct iio_chan_spec mcp3204_channels[] = {
	MCP320X_VOLTAGE_CHANNEL(0),
	MCP320X_VOLTAGE_CHANNEL(1),
	MCP320X_VOLTAGE_CHANNEL(2),
	MCP320X_VOLTAGE_CHANNEL(3),
	MCP320X_VOLTAGE_CHANNEL_DIFF(0),
	MCP320X_VOLTAGE_CHANNEL_DIFF(1),
};

static const struct iio_chan_spec mcp3208_channels[] = {
	MCP320X_VOLTAGE_CHANNEL(0),
	MCP320X_VOLTAGE_CHANNEL(1),
	MCP320X_VOLTAGE_CHANNEL(2),
	MCP320X_VOLTAGE_CHANNEL(3),
	MCP320X_VOLTAGE_CHANNEL(4),
	MCP320X_VOLTAGE_CHANNEL(5),
	MCP320X_VOLTAGE_CHANNEL(6),
	MCP320X_VOLTAGE_CHANNEL(7),
	MCP320X_VOLTAGE_CHANNEL_DIFF(0),
	MCP320X_VOLTAGE_CHANNEL_DIFF(1),
	MCP320X_VOLTAGE_CHANNEL_DIFF(2),
	MCP320X_VOLTAGE_CHANNEL_DIFF(3),
};

static const struct mcp320x_chip_info mcp320x_chip_infos[] = {
	[mcp3001] = {
		.channels = mcp3201_channels,
		.num_channels = ARRAY_SIZE(mcp3201_channels),
		.resolution = 10
	},
	[mcp3002] = {
		.channels = mcp3202_channels,
		.num_channels = ARRAY_SIZE(mcp3202_channels),
		.resolution = 10
	},
	[mcp3004] = {
		.channels = mcp3204_channels,
		.num_channels = ARRAY_SIZE(mcp3204_channels),
		.resolution = 10
	},
	[mcp3008] = {
		.channels = mcp3208_channels,
		.num_channels = ARRAY_SIZE(mcp3208_channels),
		.resolution = 10
	},
	[mcp3201] = {
		.channels = mcp3201_channels,
		.num_channels = ARRAY_SIZE(mcp3201_channels),
		.resolution = 12
	},
	[mcp3202] = {
		.channels = mcp3202_channels,
		.num_channels = ARRAY_SIZE(mcp3202_channels),
		.resolution = 12
	},
	[mcp3204] = {
		.channels = mcp3204_channels,
		.num_channels = ARRAY_SIZE(mcp3204_channels),
		.resolution = 12
	},
	[mcp3208] = {
		.channels = mcp3208_channels,
		.num_channels = ARRAY_SIZE(mcp3208_channels),
		.resolution = 12
	},
	[mcp3301] = {
		.channels = mcp3201_channels,
		.num_channels = ARRAY_SIZE(mcp3201_channels),
		.resolution = 13
	},
};




static void mygamepad_spi_poll(struct input_dev *input)
{
	struct mcp320x *adc = input_get_drvdata(input);
	u8 b_rsp3, b_rsp4;
	int err;

	int x,y

	err = mcp320x_read_raw(input)
	if (err) {
		dev_err(&pad->spi->dev,
			"%s: poll command failed mode: %d\n", __func__, err);
		return;
	}


	input_report_abs(input, ABS_X, REVERSE_BIT(pad->response[7]));
	input_report_abs(input, ABS_Y, REVERSE_BIT(pad->response[8]));
	input_report_key(input, BTN_X, b_rsp4 & BIT(3));
	input_report_key(input, BTN_A, b_rsp4 & BIT(2));
	input_report_key(input, BTN_B, b_rsp4 & BIT(1));
	input_report_key(input, BTN_Y, b_rsp4 & BIT(0));
		
	input_sync(input);
}

static int mygamepad_probe(struct spi_device *spi)
{
	struct input_dev *idev;
	struct mcp320x *adc;
	const struct mcp320x_chip_info *chip_info;
	int ret;
        
/*  The resource managed devm_input_allocate_device()/devm_iio_device_free()
    to automatically clean up any allocations made by Input drivers,
    thus leading to simplified Input drivers code.        
*/
	idev = devm_input_allocate_device(&spi->dev);
	if (!idev) {
		dev_err(&spi->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	adc = devm_kzalloc(&spi->dev, sizeof(struct mcp320x), GFP_KERNEL);

	/* input poll device settings */
	adc->idev = idev;
	adc->spi = spi;

	/* input device settings */
	input_set_drvdata(idev, adc);

	idev->name = "MyGamePad";
	idev->id.bustype = BUS_SPI;
	// idev->open = mygamepad_spi_poll_open;
	// idev->close = mygamepad_spi_poll_close;

	/* key/value map settings */
	input_set_abs_params(idev, ABS_X, 0, 255, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 255, 0, 0);
	input_set_capability(idev, EV_KEY, BTN_A);
	input_set_capability(idev, EV_KEY, BTN_B);
	input_set_capability(idev, EV_KEY, BTN_X);
	input_set_capability(idev, EV_KEY, BTN_Y);

	chip_info = &mcp320x_chip_infos[mcp3008];
    dprint("the index = %d\n",mcp3008);

	adc->device_id = mcp3008;
	adc->chip_info = chip_info;

	adc->transfer[0].tx_buf = &adc->tx_buf;
	adc->transfer[0].len = sizeof(adc->tx_buf);
	adc->transfer[1].rx_buf = adc->rx_buf;
	adc->transfer[1].len = sizeof(adc->rx_buf);

	spi_message_init_with_transfers(&adc->msg, adc->transfer,
					ARRAY_SIZE(adc->transfer));

	adc->reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->reg))
		return PTR_ERR(adc->reg);

	ret = regulator_enable(adc->reg);
	if (ret < 0)
		return ret;

	mutex_init(&adc->lock);

	err = input_setup_polling(idev, mygamepad_spi_poll);
	if (err) {
		dev_err(&spi->dev, "failed to set up polling: %d\n", err);
		return err;
	}

	/* poll interval is about 60fps */
	input_set_poll_interval(idev, 16);
	input_set_min_poll_interval(idev, 8);
	input_set_max_poll_interval(idev, 32);

	/* register input poll device */
	ret = input_register_device(idev);
	if (ret) {
		dev_err(&spi->dev,
			"failed to register input device: %d\n", ret);
		goto reg_disable;
	}
        
    dprint("success\n");
	return 0;

reg_disable:
	regulator_disable(adc->reg);

	return ret;
}

static int mygamepad_remove(struct input_dev *input)
{

	struct mcp320x *adc = input_get_drvdata(input);

	input_unregister_device (input);
	regulator_disable(adc->reg);

	return 0;
}

#if 0
static const struct of_device_id mcp320x_dt_ids[] = {
	{
		.compatible = "mcp3001",
		.data = &mcp320x_chip_infos[mcp3001],
	}, {
		.compatible = "mcp3002",
		.data = &mcp320x_chip_infos[mcp3002],
	}, {
		.compatible = "mcp3004",
		.data = &mcp320x_chip_infos[mcp3004],
	}, {
		.compatible = "mcp3008",
		.data = &mcp320x_chip_infos[mcp3008],
	}, {
		.compatible = "mcp3201",
		.data = &mcp320x_chip_infos[mcp3201],
	}, {
		.compatible = "mcp3202",
		.data = &mcp320x_chip_infos[mcp3202],
	}, {
		.compatible = "mcp3204",
		.data = &mcp320x_chip_infos[mcp3204],
	}, {
		.compatible = "mcp3208",
		.data = &mcp320x_chip_infos[mcp3208],
	}, {
		.compatible = "mcp3301",
		.data = &mcp320x_chip_infos[mcp3301],
	}, {
	}
};
MODULE_DEVICE_TABLE(of, mcp320x_dt_ids);
#else
static const struct of_device_id mcp320x_dt_ids[] = {
	{
		.compatible = "mygamepad-spi",
		.data = &mcp320x_chip_infos[mcp3008],
	}, {

	}
};
MODULE_DEVICE_TABLE(of, mcp320x_dt_ids);
#endif

//name , driver_data
#if 0
static const struct spi_device_id mcp320x_id[] = {
	{ "mcp3001", mcp3001 },
	{ "mcp3002", mcp3002 },
	{ "mcp3004", mcp3004 },
	{ "mcp3008", mcp3008 },
	{ "mcp3201", mcp3201 },
	{ "mcp3202", mcp3202 },
	{ "mcp3204", mcp3204 },
	{ "mcp3208", mcp3208 },
	{ "mcp3301", mcp3301 },
	{ }
};
#endif

static const struct spi_device_id mygamepad_id[] = {
	{ "mygamepad-spi", mcp3008 },
	{ }
};

MODULE_DEVICE_TABLE(spi, mygamepad_id);

static struct spi_driver mygamepad_spi_driver = {
	.driver = {
		.name = "mygamepad-spi",
		.of_match_table = of_match_ptr(mcp320x_dt_ids),
	},
	.probe = mygamepad_probe,
	.remove = mygamepad_remove,
	.id_table = mygamepad_id,
};
module_spi_driver(mygamepad_spi_driver);

MODULE_AUTHOR("JeffZen-TW <jeff810123@gmail.com>");
MODULE_DESCRIPTION("MyGamePad");
MODULE_LICENSE("GPL v2");
