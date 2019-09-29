/*
 * PlayStation 1/2 joypads via SPI interface Driver
 *
 * Copyright (C) 2017 Tomohiro Yoshidomi <sylph23k@gmail.com>
 * Licensed under the GPL-2 or later.
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

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
/*
#ifdef CONFIG_ARCH_MULTI_V7
#define BCM2708_PERI_BASE 0x3F000000
#else
#define BCM2708_PERI_BASE 0x20000000
#endif
*/
#define BCM2708_PERI_BASE 0xFE000000 //RPi 4

#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

#define GPIO_STATUS (*(gpio+13))

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
#define HAVE_TIMER_SETUP
#endif

#define REVERSE_BIT(x) ((((x) & 0x80) >> 7) | (((x) & 0x40) >> 5) | \
	(((x) & 0x20) >> 3) | (((x) & 0x10) >> 1) | (((x) & 0x08) << 1) | \
	(((x) & 0x04) << 3) | (((x) & 0x02) << 5) | (((x) & 0x01) << 7))

#define PSX_ACK_OR_DIE if(!psxpad_wait_ack()) { GPIO_SET = pin; udelay(40); return 0; }

#define PSX_GPIO_ACK 25
#define PSX_GPIO_ATT1 23
#define PSX_GPIO_ATT2 24

static const char *psx_name = "PSX controller";

#define PSX_REFRESH_TIME	HZ/60

/* PlayStation 1/2 joypad command and response are LSBFIRST. */

static const u8 PSX_CMD_POLL[] = {
/*	0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 */
	0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const u8 PSX_CMD_ENTER_CFG[] = {
/*	0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 */
	0xC2, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const u8 PSX_CMD_EXIT_CFG[] = {
/*	0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A */
	0xC2, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A
};
static const u8 PSX_CMD_ENABLE_MOTOR[]	= {
/*	0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF */
	0xB2, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF
};

struct psxpad {
	struct input_dev *dev;
	u8 dev_reg;
	u8 type;
	char phys[32];
	bool motor1enable;
	bool motor2enable;
	u8 motor1level;
	u8 motor2level;
	struct spi_device *spi;
};

struct psxdriver {
	struct spi_device *spi;
	struct psxpad* pads[2][4];
	struct timer_list timer;
	u8 sendbuf[50] ____cacheline_aligned;
	u8 response[50] ____cacheline_aligned;
};

static volatile unsigned *gpio = NULL;

/*
 * psx_xmit() sends the desired amount of bytes to the PSX controller via SPI. In most cases only 1 byte
 *  should be sent using this call unless transmitting at 1 Mhz, because the inter-byte delay is not enough
 *  time for the controller to recover and be ready for the next byte.
 */
static int psx_xmit(struct spi_device *spi, const u8 *inBytes, u8 *outBytes, u8 len, u32 baud_override)
{
	struct spi_transfer xfers = {
                .tx_buf         = inBytes,
                .rx_buf         = outBytes,
                .len            = len,
                .speed_hz       = baud_override,
		.delay_usecs	= 0
        };
        int err;
	u8 index;

        err = spi_sync_transfer(spi, &xfers, 1);
        if (err) {
                dev_err(&spi->dev,
                        "%s: failed to SPI xfers mode: %d\n",
                        __func__, err);
                return err;
        }
        for(index = 0; index < len; index++)
        {
                outBytes[index] = REVERSE_BIT(outBytes[index]);
        }

        return 0;
}

/*
 * psxpad_wait_ack() waits for ACK pin to go low, signaling a controller has acknowledged a data transmission.
 *  if ACK does not go low within ~50us, function returns non-zero, signaling the controller is not present.
 *
 * returns non-zero for success, zero for failure.
 */
static int psxpad_wait_ack(void)
{
        int i;
        for(i = 0; i < 50; i++)
        {
                udelay(1);
                if(!(GPIO_STATUS & (1 << PSX_GPIO_ACK)))
                {
                        udelay(6); /* sleep for one clock cycle to let controller settle */
                        return 1;
                }
        }
        return 0;
}

/*
 * psxpad_command() is the main function for sending a message to the PSX controller/multitap. In normal SPI
 *  the number of bytes sent dictate the number of bytes received, but PSX controllers require some dynamic
 *  handling. The digital controller returns less bytes than the analog for instance. If the multitap is
 *  connected, then it returns 35 bytes, and switches back and forth from 250khz to 1000khz.
 *
 * returns 0 for no controller connected, >0 for the number of bytes received, or <0 for SPI errors.
 */
static int psxpad_command(struct psxdriver *psx, const u8 sendcmdlen, u8 pin_num)
{
	unsigned long flags;
        int numBytesRecvd = 0;
	u8 num_pad_bytes = 8;
	int mtap;
	u8 ack_seen = 0;
        u8 mode = 0;
        const u8 wakeup = 0x80;//0x01;
        const u8 read_mode = 0x42;
	u8 index = 0;
        int pin = 1 << (pin_num == 0 ? PSX_GPIO_ATT1 : PSX_GPIO_ATT2);
        if(!sendcmdlen)
                return 0;
	local_irq_save(flags);

        mode = psx->sendbuf[0];
        GPIO_CLR = pin;

        udelay(100); // waiting 100 us after pulling ATT low is key to multitap support

        if(psx_xmit(psx->spi, &wakeup, psx->response, 1, 0))
        {
                GPIO_SET = pin;
		local_irq_restore(flags);
                return -1;
        }
	for(index = 0; index < 50; index++)
        {
                udelay(1);
                if(!(GPIO_STATUS & (1 << PSX_GPIO_ACK)))
                {
			ack_seen = 1; // saw ack... keep waiting though. need to wait 50 us
                }
        }

        numBytesRecvd++;
        psx_xmit(psx->spi, psx->sendbuf, psx->response + numBytesRecvd, 1, 0);
        numBytesRecvd++;
	if(psx->response[1] == 0xFF)
	{
		local_irq_restore(flags);
		return 0;
	}

        if(psx->response[1] == 0x80) // multitap
        {
		PSX_ACK_OR_DIE;
                if(psx_xmit(psx->spi, &wakeup, psx->response + numBytesRecvd, 1, 2000000))
                {
                        GPIO_SET = pin;
			local_irq_restore(flags);
			return -1;
                }
		PSX_ACK_OR_DIE;

                numBytesRecvd++;

                if(mode != read_mode && !psx->pads[pin_num][0])
                {
                        psx_xmit(psx->spi, &read_mode, psx->response + numBytesRecvd, 1, 0);
                }
                else psx_xmit(psx->spi, psx->sendbuf, psx->response + numBytesRecvd, 1, 0);
                numBytesRecvd++;

                mtap = 1;
        }
	if(mode == read_mode && psx->pads[pin_num][0] != NULL)
	{
		psx->sendbuf[3] = psx->pads[pin_num][0]->motor1enable ? psx->pads[pin_num][0]->motor1level : 0x00;
	        psx->sendbuf[4] = psx->pads[pin_num][0]->motor2enable ? psx->pads[pin_num][0]->motor2level : 0x00;
	}
	if(!mtap)
	{
		num_pad_bytes = 2 * (psx->response[1] & 0x0F) + 1;
	}
        for(index = 1; index < num_pad_bytes; index++)
        {
		PSX_ACK_OR_DIE;
                psx_xmit(psx->spi, psx->sendbuf + index, psx->response + numBytesRecvd, 1, 0);
                numBytesRecvd ++;
        }

        if(mtap)
        {
		PSX_ACK_OR_DIE;
		if(sendcmdlen <= 8) // if the user only supplied 8 command bytes, repeat them for the other controllers on the multitap
		{
			for(index = 1; index < 4; index++)
			{
				if(mode == read_mode && psx->pads[pin_num][index] != NULL)
			        {
			                psx->sendbuf[3] = psx->pads[pin_num][index]->motor1enable ? psx->pads[pin_num][index]->motor1level : 0x00;
        			        psx->sendbuf[4] = psx->pads[pin_num][index]->motor2enable ? psx->pads[pin_num][index]->motor2level : 0x00;
			        }
				if(psx->pads[pin_num][index]) psx->sendbuf[0] = mode;
				else psx->sendbuf[0] = read_mode;
		                psx_xmit(psx->spi, psx->sendbuf, psx->response + numBytesRecvd, 8, 2000000);
	        	        numBytesRecvd += 8;
			}
		}
		else
		{
			psx_xmit(psx->spi, psx->sendbuf + 8, psx->response + numBytesRecvd, sendcmdlen - 8, 2000000);
	                numBytesRecvd += (sendcmdlen - 8);
		}
        }
        GPIO_SET = pin;
	local_irq_restore(flags);
	return numBytesRecvd;
}

static int psx_open(struct input_dev *dev)
{
	return 0;
}

static void psx_close(struct input_dev *dev)
{
}

static const short psx_abs[] = {
	ABS_RX, ABS_RY, ABS_X, ABS_Y
};
static const short psx_btn[] = {
	BTN_TL2, BTN_TR2, BTN_TL, BTN_TR, BTN_X, BTN_A, BTN_B, BTN_Y,
	BTN_SELECT, BTN_THUMBL, BTN_THUMBR, BTN_START
};


#ifdef CONFIG_JOYSTICK_PSXPAD_SPI_FF
static void psxpad_control_motor(struct psxdriver* psx, u8 pin_num,
	bool motor1enable, bool motor2enable)
{
	int err;
	u8 pad_num = 0;
	for(pad_num = 0; pad_num < 3; pad_num++)
	{
		struct psxpad *pad = psx->pads[pin_num][pad_num];

		pad->motor1enable = motor1enable;
		pad->motor2enable = motor2enable;
	}

	memcpy(psx->sendbuf, PSX_CMD_ENTER_CFG, sizeof(PSX_CMD_ENTER_CFG));
	err = psxpad_command(psx, sizeof(PSX_CMD_ENTER_CFG), pin_num);
	if (err < 0) {
		dev_err(&psx->spi->dev,
			"%s: failed to enter config mode: %d\n",
			__func__, err);
		return;
	}
	memcpy(psx->sendbuf, PSX_CMD_ENABLE_MOTOR, sizeof(PSX_CMD_ENABLE_MOTOR));
	psx->sendbuf[3] = motor1enable ? 0x00 : 0xFF;
	psx->sendbuf[4] = motor2enable ? 0x80 : 0xFF;
	err = psxpad_command(psx, sizeof(PSX_CMD_ENABLE_MOTOR), pin_num);
	if (err < 0) {
		dev_err(&psx->spi->dev,
			"%s: failed to enable motor mode: %d\n",
			__func__, err);
		return;
	}
	memcpy(psx->sendbuf, PSX_CMD_EXIT_CFG, sizeof(PSX_CMD_EXIT_CFG));
	err = psxpad_command(psx, sizeof(PSX_CMD_EXIT_CFG), pin_num);
	if (err < 0) {
		dev_err(&psx->spi->dev,
			"%s: failed to exit config mode: %d\n",
			__func__, err);
		return;
	}
}

static void psxpad_set_motor_level(struct psxpad *pad,
	u8 motor1level, u8 motor2level)
{
	pad->motor1level = motor1level ? 0xFF : 0x00;
	pad->motor2level = REVERSE_BIT(motor2level);
}

static int psxpad_spi_play_effect(struct input_dev *idev,
	void *data, struct ff_effect *effect)
{
	struct psxpad *pad = input_get_drvdata(idev);

	switch (effect->type) {
	case FF_RUMBLE:
		psxpad_set_motor_level(pad,
			(effect->u.rumble.weak_magnitude >> 8) & 0xFFU,
			(effect->u.rumble.strong_magnitude >> 8) & 0xFFU);
		break;
	}

	return 0;
}

#else	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */

static void psxpad_control_motor(struct psxpad *pad,
	bool motor1enable, bool motor2enable)
{
}

static void psxpad_set_motor_level(struct psxpad *pad,
	u8 motor1level, u8 motor2level)
{
}

static inline int psxpad_spi_init_ff(struct psxpad *pad)
{
	return 0;
}
#endif	/* CONFIG_JOYSTICK_PSXPAD_SPI_FF */

static int psxpad_create_dev(struct psxdriver *psxdriver, int pin_num, int pad_num, u8 pad_type)
{
	struct input_dev *dev;
	int i;
	int err;
	//if (pad_type < 1 || pad_type >= GC_MAX) {
	//	pr_err("Pad type %d unknown\n", pad_type);
	//	return -EINVAL;
	//}

	if(!psxdriver->pads[pin_num][pad_num]) {
		psxdriver->pads[pin_num][pad_num] = kzalloc(sizeof(struct psxpad), GFP_KERNEL);
	}

	if(!psxdriver->pads[pin_num][pad_num]->dev) {
		dev = input_allocate_device();
		if (!dev) {
			pr_err("Not enough memory for input device\n");
			return -ENOMEM;
		}

		dev->name = psx_name;
		snprintf(psxdriver->pads[pin_num][pad_num]->phys, 0x20,
			"psxpad_%d_%d", pin_num, pad_num);
		dev->phys = psxdriver->pads[pin_num][pad_num]->phys;
		dev->id.bustype = BUS_SPI;
		dev->id.vendor = 0x0001;
		dev->id.product = pad_type;
		dev->id.version = 0x0100;

		input_set_drvdata(dev, psxdriver);

		dev->open = psx_open;
		dev->close = psx_close;

		dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

		input_set_abs_params(dev, ABS_HAT0X, -1, 1, 0, 0);
		input_set_abs_params(dev, ABS_HAT0Y, -1, 1, 0, 0);

		for (i = 0; i < 4; i++) {
			input_set_abs_params(dev, psx_abs[i], 0, 255, 0, 28);
		}
		for (i = 0; i < 12; i++) {
			__set_bit(psx_btn[i], dev->keybit);
		}
		input_set_capability(dev, EV_FF, FF_RUMBLE);

        	err = input_ff_create_memless(dev, NULL, psxpad_spi_play_effect);
	        if (err) {
			input_free_device(dev);
	                kfree(psxdriver->pads[pin_num][pad_num]);
			printk(KERN_ERR "Failed to create memless force-feedback for input device for controller on pin %d, port %d\n", pin_num, pad_num); 
		        return err;
	        }
		psxdriver->pads[pin_num][pad_num]->dev = dev;
	}

	if(!psxdriver->pads[pin_num][pad_num]->dev_reg) {
		err = input_register_device(dev);
		if(err) {
			input_free_device(dev);
			psxdriver->pads[pin_num][pad_num]->dev = NULL;
			printk(KERN_ERR "Failed to register input device for controller on pin %d, port %d\n", pin_num, pad_num);
			return err;
		}
		psxdriver->pads[pin_num][pad_num]->dev_reg = 1;
	}

	printk(KERN_INFO "psxpad_spi_ex: created device for controller on pin %d, port %d\n", pin_num, pad_num);

	return 0;
}

static int psxpad_destroy_dev(struct psxdriver *psxdriver, int pin_num, int pad_num)
{
	if(!psxdriver->pads[pin_num][pad_num]) return -EINVAL;

        struct input_dev *dev = psxdriver->pads[pin_num][pad_num]->dev;

        if(!dev) return -EINVAL;

	if(psxdriver->pads[pin_num][pad_num]->dev_reg) {
		input_unregister_device(dev);
		psxdriver->pads[pin_num][pad_num]->dev_reg = 0;
	}
	printk(KERN_INFO "psxpad_spi_ex: deleted controller on pin %d, port %d\n", pin_num, pad_num);

	return 0;
}

static void psxpad_report(struct psxpad *pad, u8 pad_type, u8 *data)
{
        struct input_dev *dev = pad->dev;
        int i;

        switch (pad_type) {
        case 0x73: // analog
	case 0xF3: // dualshock 2 native
                input_report_key(dev, BTN_THUMBL, ~data[0] & 0x02);
                input_report_key(dev, BTN_THUMBR, ~data[0] & 0x04);

                for (i = 0; i < 4; i++)
                       input_report_abs(dev, psx_abs[i + 2], data[i + 2]);

                input_report_abs(dev, ABS_HAT0X, !(data[0] & 0x20) - !(data[0] & 0x80));
                input_report_abs(dev, ABS_HAT0Y, !(data[0] & 0x40) - !(data[0] & 0x10));


                for (i = 0; i < 8; i++)
                        input_report_key(dev, psx_btn[i], ~data[1] & (1 << i));

                input_report_key(dev, BTN_START,  ~data[0] & 0x08);
                input_report_key(dev, BTN_SELECT, ~data[0] & 0x01);

                input_sync(dev);

                break;

        case 0x41: // digital

                input_report_abs(dev, ABS_HAT0X,
                        !(data[0] & 0x20) - !(data[0] & 0x80));
                input_report_abs(dev, ABS_HAT0Y,
                        !(data[0] & 0x40) - !(data[0] & 0x10));

                /*
                 * For some reason if the extra axes are left unset
                 * they drift.
	         */
                for (i = 0; i < 4; i++)
                	input_report_abs(dev, psx_abs[i + 2], 128);
                /* This needs to be debugged properly,
                 * maybe fuzz processing needs to be done
                 * in input_sync()
                 *                               --vojtech
                 */

                for (i = 0; i < 8; i++)
                        input_report_key(dev, psx_btn[i], ~data[1] & (1 << i));

                input_report_key(dev, BTN_START,  ~data[0] & 0x08);
                input_report_key(dev, BTN_SELECT, ~data[0] & 0x01);

                input_sync(dev);

                break;

        default: /* not a pad, ignore */
                break;
        }
}

#ifdef HAVE_TIMER_SETUP
static void psx_timer(struct timer_list *t)
{
	if(!t)
	{
		printk(KERN_ERR "psxpad_spi_ex: timer_list was null in timer!\n");
		return;
	}
	struct psxdriver *psx = from_timer(psx, t, timer);
#else
static void psx_timer(unsigned long private)
{
	struct psxdriver *psx = (void *) private;
#endif
	if(!psx)
	{
		printk(KERN_ERR "psxpad_spi_ex: psx struct was null in timer!\n");
		return;
	}


	u8 pin_num, pad_num, pads_added, index;
	int err;
	for(pin_num = 0; pin_num < 2; pin_num++)
	{
	        memcpy(psx->sendbuf, PSX_CMD_POLL, sizeof(PSX_CMD_POLL));
	        err = psxpad_command(psx, sizeof(PSX_CMD_POLL), pin_num);
	        if (err < 0) {
	                dev_err(&psx->spi->dev,
	                        "%s: poll command failed mode: %d\n", __func__, err);
	                return;
	        }

		if(err == 0) { // no bytes received or ACK fail
			psxpad_destroy_dev(psx, pin_num, 0);
			psxpad_destroy_dev(psx, pin_num, 1);
			psxpad_destroy_dev(psx, pin_num, 2);
			psxpad_destroy_dev(psx, pin_num, 3);
		}
		else {
			pads_added = 0;
			if(psx->sendbuf[1] & 0x80) {
				for(index = 3, pad_num = 0; index < err; index += 8, pad_num++) {
					if(psx->response[index] != 0 && psx->response[index] != 0xFF) {
						if(psxpad_create_dev(psx, pin_num, pad_num, psx->response[index]) == 0) {
							pads_added++;
						}
						psxpad_report(psx->pads[pin_num][pad_num], psx->response[index], psx->response + index + 2);
					}
					else psxpad_destroy_dev(psx, pin_num, pad_num);
				}
			}
			else { // if there's no multitap but we've gotten data, there must be valid data there
				if(psxpad_create_dev(psx, pin_num, 0, psx->response[1]) == 0) pads_added++;

				psxpad_report(psx->pads[pin_num][0], psx->response[1], psx->response + 3);
			}
			if(pads_added) {
				psxpad_control_motor(psx, pin_num, true, true);
			}
		}


	}

	mod_timer(&psx->timer, jiffies + PSX_REFRESH_TIME);

}

static int psxpad_spi_probe(struct spi_device *spi)
{
	struct psxdriver *psxdriver;

	/* Set up gpio pointer for direct register access */
 	if(gpio == NULL) {
		if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
   		   	pr_err("io remap failed\n");
	   	   	return -EBUSY;
	   	}

		*(gpio+(PSX_GPIO_ACK/10)) &= ~(7<<((PSX_GPIO_ACK%10)*3)); // ACK pin to input

		*(gpio+(PSX_GPIO_ATT1/10)) &= ~(7<<((PSX_GPIO_ATT1%10)*3));
		*(gpio+(PSX_GPIO_ATT1/10)) |= (1<<((PSX_GPIO_ATT1%10)*3)); // ATT1 (chip enable 0) to output

		*(gpio+(PSX_GPIO_ATT2/10)) &= ~(7<<((PSX_GPIO_ATT2%10)*3));
		*(gpio+(PSX_GPIO_ATT2/10)) |= (1<<((PSX_GPIO_ATT2%10)*3)); // ATT2 (chip enable 1) to output

		GPIO_SET = (1<<PSX_GPIO_ATT1) | (1<<PSX_GPIO_ATT2);
	}

	psxdriver = devm_kzalloc(&spi->dev, sizeof(struct psxdriver), GFP_KERNEL);

	if (!psxdriver)
		return -ENOMEM;

	psxdriver->spi = spi;
	/* SPI settings */
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	/* note: though 500 khz is specified, on the bus it's more like 250 because of RPi firmware oddities. */
	spi->master->min_speed_hz = 500000;
	spi->master->max_speed_hz = 500000;
	spi_setup(spi);

#ifdef HAVE_TIMER_SETUP
	timer_setup(&psxdriver->timer, psx_timer, 0);
#else
	setup_timer(&psxdriver->timer, psx_timer, (long) gc);
#endif

	pm_runtime_enable(&spi->dev);

	mod_timer(&psxdriver->timer, jiffies + PSX_REFRESH_TIME);

	printk(KERN_INFO "psxpad_spi_ex: listening for controllers on %s.\n", dev_name(&spi->dev));

	return 0;
}

static void psxpad_shutdown(struct spi_device *spi)
{
	//printk(KERN_INFO "psxpad_spi_ex: shutting down.\n");
	struct psxdriver *psx = spi_get_drvdata(spi);
	int pin_num, pad_num;
	if(psx)
	{
		del_timer(&psx->timer);
		for(pin_num = 0; pin_num < 2; pin_num++)
	       	{
	               	for(pad_num = 0; pad_num < 4; pad_num++)
	                {
				psxpad_destroy_dev(psx, pin_num, pad_num);
	       	        }
	        }
		kfree(psx);
	}
	if(gpio) {
		iounmap(gpio);
		gpio = NULL;
	}
}

static int __maybe_unused psxpad_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct psxdriver *psx = spi_get_drvdata(spi);
	int pin_num, pad_num;
	for(pin_num = 0; pin_num < 2; pin_num++)
	{
		for(pad_num = 0; pad_num < 4; pad_num++)
		{
			if(psx->pads[pin_num][pad_num]) psxpad_set_motor_level(psx->pads[pin_num][pad_num], 0, 0);
		}
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(psxpad_spi_pm, psxpad_spi_suspend, NULL);

static const struct spi_device_id psxpad_spi_id[] = {
	{ "psxpad-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, psxpad_spi_id);

static struct spi_driver psxpad_spi_driver = {
	.driver = {
		.name = "psxpad-spi",
		.pm = &psxpad_spi_pm,
	},
	.id_table = psxpad_spi_id,
	.probe   = psxpad_spi_probe,
	.shutdown = psxpad_shutdown
};

module_spi_driver(psxpad_spi_driver);

MODULE_AUTHOR("Katie McKean <mckeanke@msu.edu>");
MODULE_DESCRIPTION("PlayStation 1/2 joypads via SPI interface Driver -ex");
MODULE_LICENSE("GPL");
