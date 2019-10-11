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
 * 6: Attention -> CS(CE0 or CE1 for controller port 1/2)
 * 7: SCK -> SCK
 * 8: N.C.
 * 9: ACK -> N.C. or GPIO25 (pullup with 1kohm to 3.3V)
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
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sched/signal.h>

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

#define PSX_GPIO_ACK 25
#define PSX_GPIO_ATT1 23
#define PSX_GPIO_ATT2 24

static const char *psx_name = "PSX controller";

const u8 wakeup = 0x80;//0x01;
const u8 read_mode = 0x42;

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
	struct spi_device *spis[2];
	struct spi_device *cur_spi;
	struct psxpad pads[2][4];
	u8 mtaps[2];
	struct task_struct *thread;
	u8 sendbuf[50] ____cacheline_aligned;
	u8 response[50] ____cacheline_aligned;
};

static volatile struct psxdriver *psx_base = NULL;
static volatile unsigned *gpio = NULL;

/*
 * psx_xmit() sends the desired amount of bytes to the PSX controller via SPI. In most cases only 1 byte
 *  should be sent using this call unless transmitting at 1 Mhz, because the inter-byte delay is not enough
 *  time for the controller to recover and be ready for the next byte.
 */
static int psx_xmit(struct spi_device *spi, struct spi_transfer *xfers, const u8 *inBytes, u8 *outBytes, u8 len)
{
    int err;
	u8 index;
	xfers->tx_buf         = inBytes;
    xfers->rx_buf         = outBytes;
    xfers->len            = len;

    err = spi_sync_transfer(spi, xfers, 1);
    if (err) {
        dev_err(&spi->dev, "%s: failed to SPI xfers mode: %d\n", __func__, err);
        return err;
    }
    for(index = 0; index < len; index++) {
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
    struct timeval cur_time, start;
	long usec_elapsed;
    do_gettimeofday(&start);
	do
    {
        if(!(GPIO_STATUS & (1 << PSX_GPIO_ACK)))
        {
        	/* sleep at least one clock cycle to make sure controller settles before we return, but we still need to hit >50 us total */
        	if(usec_elapsed > 44) udelay(6);
        	else udelay(50 - usec_elapsed);
        	return 1;
        }
        do_gettimeofday(&cur_time);
        usec_elapsed = cur_time.tv_usec - start.tv_usec;
    }
    while(usec_elapsed < 50);
    return 0;
}

/*
 * psxpad_command() is the main function for sending a message to the PSX controller/multitap. In normal SPI
 *  the number of bytes sent dictate the number of bytes received, but PSX controllers require some dynamic
 *  handling. The digital controller returns less bytes than the analog for instance. If the multitap is
 *  connected, then it returns 35 bytes, and switches back and forth from 250 Khz to 1 Mhz.
 *
 * returns 0 for no controller connected, >0 for the number of bytes received, or <0 for SPI errors.
 */
static int psxpad_command(struct psxdriver *psx, const u8 sendcmdlen, u8 pin_num)
{
	struct spi_transfer xfer;
	unsigned long flags;
    int numBytesRecvd = 0;
	u8 num_pad_bytes = 8;
	u8 mode = 0, index = 0;
    int pin = 1 << (pin_num == 0 ? PSX_GPIO_ATT1 : PSX_GPIO_ATT2);
    if(!sendcmdlen)
        return 0;
	local_irq_save(flags);

	mode = psx->sendbuf[0];
	GPIO_CLR = pin;

	memset(&xfer, 0, sizeof(struct spi_transfer));
	xfer.delay_usecs = 0;
	xfer.speed_hz = 0;
	udelay(70); // waiting 100 us after pulling ATT low is key to multitap support
	
	if(psx_xmit(psx->spis[pin_num], &xfer, &wakeup, psx->response, 1) || !psxpad_wait_ack())
	{
	    GPIO_SET = pin;
		local_irq_restore(flags);
	    return -1;
	}
		
	numBytesRecvd++;
	xfer.delay_usecs = 5;
	psx_xmit(psx->spis[pin_num], &xfer, psx->sendbuf, psx->response + numBytesRecvd, 1);
	numBytesRecvd++;
	if(psx->response[1] == 0xFF)
	{
		local_irq_restore(flags);
		return 0;
	}

	if(psx->response[1] == 0x80) // multitap
	{
		xfer.delay_usecs = 2;
		xfer.speed_hz = 2000000;
        if(psx_xmit(psx->spis[pin_num], &xfer, &wakeup, psx->response + numBytesRecvd, 1))
        {
            GPIO_SET = pin;
			local_irq_restore(flags);
			return -1;
        }
	
        numBytesRecvd++;

		xfer.delay_usecs = 3;
		xfer.speed_hz = 0; // back to default rate
        psx_xmit(psx->spis[pin_num], &xfer, psx->sendbuf, psx->response + numBytesRecvd, 1);
        numBytesRecvd++;
	}
	psx->mtaps[pin_num] = psx->response[1] == 0x80;
	if(mode == read_mode && psx->pads[pin_num][0].dev_reg)
	{
		psx->sendbuf[3] = psx->pads[pin_num][0].motor1enable ? psx->pads[pin_num][0].motor1level : 0x00;
        psx->sendbuf[4] = psx->pads[pin_num][0].motor2enable ? psx->pads[pin_num][0].motor2level : 0x00;
	}
	if(psx->response[1] != 0x80)
	{
		num_pad_bytes = 2 * (psx->response[1] & 0x0F) + 1;
	}
	xfer.delay_usecs = 0;
    for(index = 1; index < num_pad_bytes; index++)
    {
        psx_xmit(psx->spis[pin_num], &xfer, psx->sendbuf + index, psx->response + numBytesRecvd, 1);
        numBytesRecvd ++;
    }

    if(psx->response[1] == 0x80)
    {
    	xfer.delay_usecs = 0;
        xfer.speed_hz = 2000000;
		if(sendcmdlen <= 8) // if the user only supplied 8 command bytes, repeat them for the other controllers on the multitap
		{
			for(index = 1; index < 4; index++)
			{
                psx_xmit(psx->spis[pin_num], &xfer, psx->sendbuf, psx->response + numBytesRecvd, 8);
       	        numBytesRecvd += 8;
			}
		}
		else
		{
			psx_xmit(psx->spis[pin_num], &xfer, psx->sendbuf + 8, psx->response + numBytesRecvd, sendcmdlen - 8);
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
	u8 len = 8;
	u8 offset = 0;
	struct psxpad *pad;
	const u8* messages[] = { PSX_CMD_ENTER_CFG, PSX_CMD_ENABLE_MOTOR, PSX_CMD_EXIT_CFG };
	u8 message_index = 0;
	for(pad_num = 0; pad_num < 3; pad_num++)
	{
		pad = &psx->pads[pin_num][pad_num];

		pad->motor1enable = motor1enable;
		pad->motor2enable = motor2enable;
	}
	for(message_index = 0; message_index < 3; message_index++) {
		if(message_index != 0) udelay(500);
		
		if(psx->mtaps[pin_num])	{
			len = 32;
			for(pad_num = 0, offset = 0; pad_num < 3; pad_num++, offset += 8) {
				pad = &psx->pads[pin_num][pad_num];
				if(pad->type != 0xFF) {
					memcpy(psx->sendbuf + offset, messages[message_index], 8);
					if(message_index == 1) {
							psx->sendbuf[offset + 3] = motor1enable ? 0x00 : 0xFF;
							psx->sendbuf[offset + 4] = motor2enable ? 0x80 : 0xFF;
					}
				}
				else memcpy(psx->sendbuf + offset, PSX_CMD_POLL, 8);
			}
		}
		else {
			memcpy(psx->sendbuf + offset, messages[message_index], 8);
		}
		err = psxpad_command(psx, len, pin_num);
		if (err < 0) {
			dev_err(&psx->spis[pin_num]->dev, "psxpad_spi_ex(%s): failed to enable motor control: %d\n", __func__, err);
			return;
		}
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

	if(!psxdriver->pads[pin_num][pad_num].dev) {
		dev = input_allocate_device();
		if (!dev) {
			pr_err("Not enough memory for input device\n");
			return -ENOMEM;
		}

		dev->name = psx_name;
		snprintf(psxdriver->pads[pin_num][pad_num].phys, 0x20,
			"psxpad_%d_%d", pin_num, pad_num);
		dev->phys = psxdriver->pads[pin_num][pad_num].phys;
		dev->id.bustype = BUS_SPI;
		dev->id.vendor = 0x0001;
		dev->id.product = pad_type;
		dev->id.version = 0x0100;

		input_set_drvdata(dev, &psxdriver->pads[pin_num][pad_num]);

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
			psxdriver->pads[pin_num][pad_num].dev = NULL;
			printk(KERN_ERR "Failed to create memless force-feedback for input device for controller on pin %d, port %d\n", pin_num, pad_num); 
	        return err;
        }
		psxdriver->pads[pin_num][pad_num].dev = dev;
	}

	if(!psxdriver->pads[pin_num][pad_num].dev_reg) {
		dev = psxdriver->pads[pin_num][pad_num].dev;
		err = input_register_device(dev);
		if(err) {
			input_free_device(dev);
			psxdriver->pads[pin_num][pad_num].dev = NULL;
			printk(KERN_ERR "Failed to register input device for controller on pin %d, port %d\n", pin_num, pad_num);
			return err;
		}
		psxdriver->pads[pin_num][pad_num].dev_reg = 1;
		printk(KERN_INFO "psxpad_spi_ex: registered device for controller on pin %d, port %d\n", pin_num, pad_num);
		return 0;
	}

	return -1; //already registered
}

static int psxpad_destroy_dev(struct psxdriver *psxdriver, int pin_num, int pad_num)
{
	struct input_dev *dev;

    dev = psxdriver->pads[pin_num][pad_num].dev;

    if(!dev) return -EINVAL;

	if(psxdriver->pads[pin_num][pad_num].dev_reg) {
		input_unregister_device(dev);
		psxdriver->pads[pin_num][pad_num].dev_reg = 0;
		psxdriver->pads[pin_num][pad_num].dev = NULL;
		printk(KERN_INFO "psxpad_spi_ex: unregistered device for controller on pin %d, port %d\n", pin_num, pad_num);
	}

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

static int psxpad_read_pads(void* args)
{
    allow_signal(SIGKILL);
	struct psxdriver *psx = args;
	while(!kthread_should_stop()) {
		u8 pin_num, pad_num, pads_added, index, len, offset = 0;
		int err;
		
		for(pin_num = 0; pin_num < 1; pin_num++) {
		        if(psx->mtaps[pin_num]) {
		        	
	    			len = 32;
	    			for(pad_num = 0, offset = 0; pad_num < 4; pad_num++, offset += 8){
	    				memcpy(psx->sendbuf + offset, PSX_CMD_POLL, sizeof(PSX_CMD_POLL));
	    				
	    				psx->sendbuf[offset + 3] = psx->pads[pin_num][pad_num].motor1level;
	    				psx->sendbuf[offset + 4] = psx->pads[pin_num][pad_num].motor2level;
	    			}
		        }
		        else {
		        	len = (2 * psx->pads[pin_num][pad_num].type) + 1;
		        	
					memcpy(psx->sendbuf, PSX_CMD_POLL, sizeof(PSX_CMD_POLL));
		        }
		        err = psxpad_command(psx, len, pin_num);
		        if (err < 0) {
                	dev_err(&psx->spis[pin_num]->dev, "%s: poll command failed mode: %d\n", __func__, err);
		        }
	
			if(err == 0) { // no bytes received or ACK fail
				psxpad_destroy_dev(psx, pin_num, 0);
				psxpad_destroy_dev(psx, pin_num, 1);
				psxpad_destroy_dev(psx, pin_num, 2);
				psxpad_destroy_dev(psx, pin_num, 3);
			}
			else {
				pads_added = 0;
				if(psx->response[1] & 0x80) {
					for(index = 3, pad_num = 0; index < err; index += 8, pad_num++) {
						if(psx->response[index] != 0 && psx->response[index] != 0xFF) {
							if(psxpad_create_dev(psx, pin_num, pad_num, psx->response[index]) == 0) {
								pads_added++;
							}
							psxpad_report(&psx->pads[pin_num][pad_num], psx->response[index], psx->response + index + 2);
						}
						else psxpad_destroy_dev(psx, pin_num, pad_num);
					}
				}
				else { // if there's no multitap but we've gotten data, there must be valid data there
					if(psxpad_create_dev(psx, pin_num, 0, psx->response[1]) == 0) pads_added++;
	
					psxpad_report(&psx->pads[pin_num][0], psx->response[1], psx->response + 3);
				}
				if(pads_added) {
					psxpad_control_motor(psx, pin_num, true, true);
				}
			}
		}
        if(signal_pending(current))
            break;
		mdelay(10);
	}
	
	
	while(!kthread_should_stop())
	{
		/* 
		* Flush any pending signal.
		*
		* Otherwise interruptible wait will not wait actually.
		*/
		flush_signals(current);
		/* Stopping thread is some sort of interrupt. That's why we need interruptible wait. */        
		set_current_state(TASK_INTERRUPTIBLE);
		if(!kthread_should_stop()) schedule();
		set_current_state(TASK_RUNNING);
	}
	return 0;
}

void memset_volatile(volatile void *s, char c, size_t n)
{
    volatile char *p = s;
    while (n-- > 0) {
        *p++ = c;
    }
} 

static int psxpad_spi_probe(struct spi_device *spi)
{
	char *our_thread = "psx_thread";
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
	/* SPI settings */
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	/* note: though 500 khz is specified, on the bus it's more like 250 because of RPi firmware oddities. */
	spi->master->min_speed_hz = 500000;
	spi->master->max_speed_hz = 500000;
	spi_setup(spi);
	

	if(psx_base == NULL) {
		psx_base = devm_kzalloc(&spi->dev, sizeof(struct psxdriver), GFP_KERNEL);
		memset_volatile(psx_base, 0, sizeof(struct psxdriver));

		if (!psx_base)
			return -ENOMEM;

#ifdef HAVE_TIMER_SETUP
		//timer_setup(&psx_base->timer, psx_timer, 0);
#else
		//setup_timer(&psx_base->timer, psx_timer, (long) psx_base);
#endif

		psx_base->spis[0] = spi;
	}
	else { // entered probe for second time
		if(!psx_base->spis[0])
		{
			printk(KERN_ERR "psxpad_spi_ex: spis[0] was null! Cannot start timer.");
			return -EINVAL;
		}
		psx_base->spis[1] = spi;

		psx_base->thread = kthread_run(psxpad_read_pads, psx_base, our_thread);
		
		//mod_timer(&psx_base->timer, jiffies + PSX_REFRESH_TIME);
	}
	pm_runtime_enable(&spi->dev);

	printk(KERN_INFO "psxpad_spi_ex: listening for controllers on %s.\n", dev_name(&spi->dev));

	return 0;
}

static int psxpad_remove(struct spi_device *spi)
{
	//printk(KERN_INFO "psxpad_spi_ex: shutting down.\n");
	int pin_num, pad_num;
	if(psx_base) {
       	if(psx_base->thread) 
       	{
       		kthread_stop(psx_base->thread);
       		psx_base->thread = NULL;
       	}
		for(pin_num = 0; pin_num < 2; pin_num++) {
          	for(pad_num = 0; pad_num < 4; pad_num++) {
				psxpad_destroy_dev(psx_base, pin_num, pad_num);
        	}
      	}
		psx_base = NULL;
	}
	if(gpio) {
		iounmap(gpio);
		gpio = NULL;
	}
	return 0;
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
			psxpad_set_motor_level(&psx->pads[pin_num][pad_num], 0, 0);
		}
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(psxpad_spi_pm, psxpad_spi_suspend, NULL);

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "rohm,dh2228fv" },
	{ .compatible = "lineartechnology,ltc2488" },
	{ .compatible = "ge,achc" },
	{ .compatible = "semtech,sx1301" },
	{ .compatible = "lwn,bk4" },
	{ .compatible = "dh,dhcom-board" },
	{ .compatible = "menlo,m53cpld" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver psxpad_spi_driver = {
	.driver = {
		//.name = "psxpad-spi",
		.name = "spidev",
		.of_match_table = of_match_ptr(spidev_dt_ids),
		.pm = &psxpad_spi_pm,
	},
	//.id_table = psxpad_spi_id,
	.probe   = psxpad_spi_probe,
	.remove = psxpad_remove
};

module_spi_driver(psxpad_spi_driver);

MODULE_AUTHOR("Katie McKean <mckeanke@msu.edu>");
MODULE_DESCRIPTION("PlayStation 1/2 joypads via SPI interface Driver -ex");
MODULE_LICENSE("GPL");
