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

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>

#define MY_MAJOR 240
#define MY_MINOR 0

static int my_open(struct inode *np, struct file *fp);
static ssize_t my_read(struct file *fp, char *usrbuf, size_t num, loff_t *offs);
static ssize_t my_write(struct file *fp, const char *usrbuf, size_t num, loff_t *offs);
static int my_close(struct inode *np, struct file *fp);

static struct file_operations my_fops = {
    read    :       my_read,
    write   :       my_write,
    open    :       my_open,
    release :       my_close,
    owner   :       THIS_MODULE
};

static struct cdev my_cdev;
static void my_tasklet_handler(unsigned long data);
DECLARE_TASKLET(mytasklet, my_tasklet_handler, 0);
static wait_queue_head_t  my_device_wait;

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
                { PIN_STEP , GPIOF_IN | GPIOF_OPEN_SOURCE, "MOT1-STEP" }
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

static int fBusy = 0;
static int nToGo = 3200 * 64;
static int fDir = 0;
static int nStartDiv = 1024;
static int nMinDiv = 384;
static int nAccelDist = 400;
static int nAccelStep = 8;
static int nStopTime = 100;

static int nGone = 0;
static int nToBrake = 0;
static int nCurDiv = 1024;

static void mot_start( void )
{
    fBusy = 1;
    nToBrake = nGone = 0;
    nCurDiv = nStartDiv;

    if( fDir )
    {
	GPIO_CLR = (1 << PIN_DIR );
    } else {
	GPIO_SET = (1 << PIN_DIR );
    }

    *(timer_pf+1) = 0x5a000000 + ( nStartDiv << 12 ) + ( 0x0 );
    *timer_pf = 0x5a000211;
}

static void mot_kill( void )
{
    writel( 0x5a000201 , timer_pf );
    fBusy = 0;
}

#define MIN(a,b) ((a)<(b)?(a):(b))

static ssize_t my_write(struct file *fp, const char *usrbuf, size_t num, loff_t *offs) 
{
    long n = 0;
    int s;
    char buf[ 64 ];
    if( num < 1 )
	return -EINVAL;
    if( fBusy )
    {
//	return -EWOULDBLOCK;
	wait_event_interruptible( my_device_wait, fBusy == 0 );
    }

    s = copy_from_user( buf , usrbuf , MIN( num , sizeof( buf ) -1 ) );
    buf[ MIN( num , sizeof( buf ) -1 ) ] = '\0';

    s = kstrtol( buf + 1 , 0 , &n );
    if( s < 0 )
    {
        printk(KERN_ERR "Invalid value in write: %s\n", buf);
	return s;
    }

    switch( buf[0] )
    {
	case 'G':;
	    nToGo = n;
	    mot_start();
	    break;

	case 'g':;
	    nStopTime = n;
	    break;

	case 'S':;
	    nStartDiv = n;
	    break;

	case 's':;
	    nMinDiv = n;
	    break;

	case 'a':;
	    nAccelDist = n;
	    break;

	case 'A':;
	    nAccelStep = n;
	    break;

	case 'D':;
	    fDir = n ? 1 : 0;
	    break;

	default:
	    printk(KERN_ERR "Invalid comand in write: %s\n", buf);
	    return -EINVAL;
    }

    return num;
};

static int my_open(struct inode *np, struct file *fp) {return 0;};

static ssize_t my_read(struct file *fp, char *usrbuf, size_t num, loff_t *offs) {return 0;};

static int my_close(struct inode *np, struct file *fp) {return 0;};


static void my_tasklet_handler(unsigned long data)
{
    printk(KERN_INFO "%s\n", __func__);
    mdelay( nStopTime );
    fBusy = 0;
    wake_up_interruptible(&my_device_wait);
}

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
	    {
		unsigned long flags;
		writel( 0x5a000201 , timer_pf );
		local_irq_save(flags);
		tasklet_schedule(&mytasklet);
		local_irq_restore(flags);
	    }
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
	dev_t devno = MKDEV(MY_MAJOR, MY_MINOR);

	printk(KERN_INFO "%s\n", __func__);

	init_waitqueue_head(&my_device_wait);

        ret = register_chrdev_region(devno, 1, "mot");
	if (ret) {
		printk(KERN_ERR "Unable to register device: %d\n", ret);
		return ret;
	}

        cdev_init(&my_cdev, &my_fops);
        my_cdev.owner = THIS_MODULE;
        my_cdev.ops = &my_fops;
	ret = cdev_add(&my_cdev, devno, 1);
	if (ret) {
	    printk(KERN_ERR "Unable to add device: %d\n", ret);
	    unregister_chrdev_region(devno, 1);
	    return ret;
	}

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

        printk(KERN_INFO "Current input1 value: %d\n", gpio_get_value(moto_inputs[0].gpio));

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

        printk(KERN_INFO "Successfully requested BUTTON1 IRQ # %d\n", button_irqs[0]);

	SET_GPIO_ALT( PIN_STEP , 0 );
	mdelay(2);
//	start_mot();

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
//	int i;
	dev_t devno = MKDEV(MY_MAJOR, MY_MINOR);

        // free irqs
        free_irq(button_irqs[0], NULL);

	printk(KERN_INFO "%s\n", __func__);
	if( timer_pf && fBusy )
	    mot_kill();

	cdev_del(&my_cdev);
	unregister_chrdev_region(devno, 1);

	INP_GPIO( PIN_STEP );

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
