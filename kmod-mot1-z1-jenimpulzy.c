/*
 * Basic Linux Kernel module using a tasklet to blink LEDs.
 *
 * Author:
 * 	Stefan Wendler (devnull@kaltpost.de)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>	
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>

#define TIMER_PAGE_BASE 0x7e100000
#define TIMER_BASE	0x00001070
#define GPIO_PAGE_BASE 0x7e200000

#define PIN_STEP 4
//#define PIN_STEP 18
#define PIN_DIR  17

/* Define pins, directin and inital state of GPIOs for LEDs */
static struct gpio leds[] = {
		{ PIN_DIR,  GPIOF_OUT_INIT_HIGH, "MOT1-DIR" },
		{ PIN_STEP, GPIOF_IN, "MOT1-STEP" },
};

/* Define GPIOs for BUTTONS */
#if 0
static struct gpio buttons[] = {
                { 4 , GPIOF_IN | GPIOF_OPEN_SOURCE, "BUTTON 1" }
};

/* Later on, the assigned IRQ numbers for the buttons are stored here */
static int button_irqs[] = { -1 };
#endif



static volatile unsigned char * iobase = (void *) 0x7e000000;

static volatile unsigned int * gpio_pf  = (void *)0x7e200000;
static volatile unsigned int * timer_pf = (void *)0x7e101070;

 // GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g) *(gpio_pf + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio_pf + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio_pf + (((g)/10))) |= (((a)<=3?((a)+4):(a)==4?3:2)<<(((g)%10)*3)) 
#define GPIO_SET *(gpio_pf + 7) // sets bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio_pf + 10)// clears bits which are 1 ignores bits which are 0 
#define GPIO_READ(g) *(gpio_pf + 13) &= (1<<(g))

#if 0
/*
 * The interrupt service routine called on button presses
 */
static irqreturn_t button_isr(int irq, void *data)
{
        if(irq == button_irqs[0] ) {
	    gpio_set_value( PIN_STEP , !gpio_get_value(PIN_STEP) );

            return IRQ_HANDLED;
        }
        return IRQ_HANDLED;
}
#endif

/*
 * Module init function
 */
static int __init gpiomod_init(void)
{
	int ret = 0;

	printk(KERN_INFO "%s\n", __func__);

	// register, turn on
	ret = gpio_request_array(leds, ARRAY_SIZE(leds));

	if (ret) {
		printk(KERN_ERR "Unable to request GPIOs: %d\n", ret);
		return ret;
	}

#if 0
        // register BUTTON gpios
        ret = gpio_request_array(buttons, ARRAY_SIZE(buttons));

        if (ret) {
                printk(KERN_ERR "Unable to request GPIOs for BUTTONs: %d\n", ret);
                goto fail1;
        }

        printk(KERN_INFO "Current button1 value: %d\n", gpio_get_value(buttons[0].gpio));

        ret = gpio_to_irq(buttons[0].gpio);

        if(ret < 0) {
                printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
                goto fail2;
        }

        button_irqs[0] = ret;

        printk(KERN_INFO "Successfully requested BUTTON1 IRQ # %d\n", button_irqs[0]);

        ret = request_irq(button_irqs[0], button_isr, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#button1", NULL);

        if(ret) {
                printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
                goto fail2;
        }
#endif

	iobase = ioremap( 0x20000000, 16 * 1024 * 1024 );
	gpio_pf = (unsigned int *)(iobase + 0x00200000);
	printk(KERN_INFO "gpio_pf @ %pK\n", gpio_pf);

        /* get the mapping for the page frame containing the timer */
	timer_pf = (unsigned int *)(iobase + 0x00101070);
	printk(KERN_INFO "timer_pf @ %pK\n", timer_pf);

	OUT_GPIO(4);
//	GPIO_CLR = (1<<4);
//	GPIO_SET = (1<<4);
//	GPIO_SET = (1<<4);
//	GPIO_CLR = (1<<4);

	INP_GPIO( PIN_STEP );
	SET_GPIO_ALT( PIN_STEP , 0 );

	*timer_pf = 0x5a000221;
	mdelay(10);
//	mdelay(10);
	*(timer_pf+1) = 0x5a000000 + ( 0x400 << 12 ) + ( 0x0 );
	*timer_pf = 0x5a000211 ;

//	printk(KERN_INFO "%x %x\n", timer_pf[0] , timer_pf[1]);


	return ret;

#if 0
fail2: 
        gpio_free_array(buttons, ARRAY_SIZE(buttons));

fail1:
        gpio_free_array(leds, ARRAY_SIZE(leds));

        return ret;
#endif
}

/*
 * Module exit function
 */
static void __exit gpiomod_exit(void)
{
	int i;

	printk(KERN_INFO "%s\n", __func__);

	writel( 0x5a000201 , timer_pf );
	mdelay(10);

	INP_GPIO( PIN_STEP );
#if 0
        // free irqs
        free_irq(button_irqs[0], NULL);
#endif
	// turn all off
	for(i = 0; i < ARRAY_SIZE(leds); i++) {
		gpio_set_value(leds[i].gpio, 0); 
	}

        // unregister
        gpio_free_array(leds, ARRAY_SIZE(leds));
#if 0
        gpio_free_array(buttons, ARRAY_SIZE(buttons));
#endif
	iounmap( iobase );
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Moje Identita");
MODULE_DESCRIPTION("Linux Kernel module MOT1");

module_init(gpiomod_init);
module_exit(gpiomod_exit);
