/*
 * Basic Linux Kernel module using a tasklet to blink LEDs.
 *
 * Author:
 * 	Inspired by Stefan Wendler (devnull@kaltpost.de)
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

#define TIMER_VIRT_BASE 0x20101070
#define GPIO_VIRT_BASE  0x20200000

#define PIN_STEP 4
#define PIN_DIR  17

/* Define pins, directin and inital state of GPIOs for LEDs */
static struct gpio moto_outputs[] = {
//		{ PIN_STEP, GPIOF_IN, "MOT1-STEP" },
		{ PIN_DIR,  GPIOF_OUT_INIT_LOW, "MOT1-DIR" }
};

/* Define GPIOs for BUTTONS */
static struct gpio moto_inputs[] = {
                { 4 , GPIOF_IN | GPIOF_OPEN_SOURCE, "MOT1-STEP" }
};

/* Later on, the assigned IRQ numbers for the buttons are stored here */
static int button_irqs[] = { -1 };

static volatile unsigned int * gpio_pf  = 0;
static volatile unsigned int * timer_pf = 0;

 // GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g) *(gpio_pf + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio_pf + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio_pf + (((g)/10))) |= (((a)<=3?((a)+4):(a)==4?3:2)<<(((g)%10)*3)) 
#define GPIO_SET *(gpio_pf + 7) // sets bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio_pf + 10)// clears bits which are 1 ignores bits which are 0 
#define GPIO_READ(g) *(gpio_pf + 13) &= (1<<(g))

static int nGone = 0;
static int nToGo = 3200 * 64;

static int nToBrake = 0;
static int nStartDiv = 1024;
static int nMinDiv = 256;
static int nCurDiv = 1024;
static int nAccelDist = 600;
static int nAccelStep = 32;

/*
 * The interrupt service routine called on button presses
 */
static irqreturn_t button_isr(int irq, void *data)
{
//        if(irq == button_irqs[0] ) 
	{
	    ++nGone;
	    --nToGo;
	    if( nToGo <= 1 )
		writel( 0x5a000201 , timer_pf );
	    else
	    {
		if( nToGo > nToBrake + nAccelDist )
		{ // muzeme zrychlovat
		    if( nCurDiv - nAccelStep > nMinDiv && nToBrake + nAccelDist <= nGone )
		    {
			nToBrake += nAccelDist;
			nCurDiv -= nAccelStep;
			*(timer_pf+1) = 0x5a000000 + ( nCurDiv << 12 ) + ( 0x0 );
		    }
		} else { // brzdime
		    if( nCurDiv < nStartDiv && nToBrake >= nToGo )
		    {
			nToBrake -= nAccelDist;
			nCurDiv += nAccelStep;
			*(timer_pf+1) = 0x5a000000 + ( nCurDiv << 12 ) + ( 0x0 );
		    }
		}
	    }
            return IRQ_HANDLED;
        }
        return IRQ_HANDLED;
}

/*
 * Module init function
 */
static int __init gpiomod_init(void)
{
	int ret = 0;

	printk(KERN_INFO "%s\n", __func__);

	// register, turn on
	ret = gpio_request_array(moto_outputs, ARRAY_SIZE(moto_outputs));

	if (ret) {
		printk(KERN_ERR "Unable to request GPIOs: %d\n", ret);
		return ret;
	}

        // register BUTTON gpios
        ret = gpio_request_array(moto_inputs, ARRAY_SIZE(moto_inputs));

        if (ret) {
                printk(KERN_ERR "Unable to request GPIOs for moto_inputs: %d\n", ret);
                goto fail1;
        }

	gpio_pf = ioremap( GPIO_VIRT_BASE, 4096 );
//	printk(KERN_INFO "gpio_pf @ %pK\n", gpio_pf);
	if( (int)gpio_pf == -1 ) {
		gpio_pf = NULL;
                printk(KERN_ERR "Unable to ioremap gpio: %d\n", ret);
                goto fail2;
        }

        /* get the mapping for the page frame containing the timer */
	timer_pf = ioremap( TIMER_VIRT_BASE, 4096 );
//	printk(KERN_INFO "timer_pf @ %pK\n", timer_pf);
	if( (int)timer_pf == -1 ) {
		timer_pf = NULL;
                printk(KERN_ERR "Unable to ioremap timer: %d\n", ret);
                goto fail3;
        }
	*timer_pf = 0x5a000221;

//        printk(KERN_INFO "Current input1 value: %d\n", gpio_get_value(moto_inputs[0].gpio));

        ret = gpio_to_irq(moto_inputs[0].gpio);

        if(ret < 0) {
                printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
                goto fail4;
        }

        button_irqs[0] = ret;

        ret = request_irq(button_irqs[0], button_isr, IRQF_TRIGGER_RISING | IRQF_DISABLED, "mot1#step", NULL);

        if(ret) {
                printk(KERN_ERR "Unable to request IRQ: %d\n", ret);
                goto fail4;
        }

//        printk(KERN_INFO "Successfully requested BUTTON1 IRQ # %d\n", button_irqs[0]);


//	OUT_GPIO(4);
//	GPIO_CLR = (1<<4);
//	GPIO_SET = (1<<4);
//	GPIO_SET = (1<<4);
//	GPIO_CLR = (1<<4);
//	INP_GPIO( PIN_STEP );

	SET_GPIO_ALT( PIN_STEP , 0 );
	mdelay(10);
	*(timer_pf+1) = 0x5a000000 + ( nStartDiv << 12 ) + ( 0x0 );
	*timer_pf = 0x5a000211 ;

	return ret;

/*
fail5: 
        free_irq(button_irqs[0], NULL);
*/
fail4: 
	iounmap( timer_pf );

fail3: 
	iounmap( gpio_pf );

fail2: 
        gpio_free_array(moto_inputs, ARRAY_SIZE(moto_inputs));

fail1:
        gpio_free_array(moto_outputs, ARRAY_SIZE(moto_outputs));

        return ret;
}

/*
 * Module exit function
 */
static void __exit gpiomod_exit(void)
{
	int i;

	printk(KERN_INFO "%s\n", __func__);
	if( timer_pf )
	    writel( 0x5a000201 , timer_pf );

	INP_GPIO( PIN_STEP );
        // free irqs
        free_irq(button_irqs[0], NULL);

	// turn all off
	for(i = 0; i < ARRAY_SIZE(moto_outputs); i++) {
		gpio_set_value(moto_outputs[i].gpio, 0); 
	}

        // unregister
        gpio_free_array(moto_outputs, ARRAY_SIZE(moto_outputs));
        gpio_free_array(moto_inputs, ARRAY_SIZE(moto_inputs));

	iounmap( gpio_pf );
	iounmap( timer_pf );

	printk(KERN_INFO "nCount: %d\n", nGone );
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Moje Identita");
MODULE_DESCRIPTION("Linux Kernel module MOT1");

module_init(gpiomod_init);
module_exit(gpiomod_exit);
