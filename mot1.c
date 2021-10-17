#include <bcm2835.h>
//#include <stdio.h>
//#include <string.h>

//#include <sched.h>
//#include <sys/mman.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>


// Motor je na RPi pin GPIO 12
#define PIN_STEP 18
#define PIN_DIR  17

#define PERIOD_MS	90

struct timespec ts;

// Adds "delay" nanoseconds to timespecs and sleeps until that time
static void sleep_until(struct timespec *ts, int delay)
{
    
    ts->tv_nsec += delay;
    if(ts->tv_nsec >= 1000*1000*1000){
	ts->tv_nsec -= 1000*1000*1000;
	ts->tv_sec++;
    }
    while( EINTR == clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL) )
    ;
}

void motej( int nSteps, int nDir , int nPer )
{
    int i;
    bcm2835_gpio_write(PIN_DIR, nDir?LOW:HIGH);
    for ( i = 0 ; i < nSteps ; i++ )
    {
	// Turn it on
	bcm2835_gpio_write(PIN_STEP, HIGH);
	
	// wait a bit
	bcm2835_delayMicroseconds(nPer - 1);
//	sleep_until( &ts , nPer * 1000 );
	
	// turn it off
	bcm2835_gpio_write(PIN_STEP, LOW);
	
	// wait a bit
	bcm2835_delayMicroseconds(1);
    }
}



int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
    // Use for testing
//    bcm2835_set_debug(1);

    if (!bcm2835_init())
	{
		perror( "Nepovedlo se inicializovat GPIO knihovnu!" );
		return 1;
	}

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Set the pin to be an output
    bcm2835_gpio_fsel(PIN_STEP, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PIN_DIR, BCM2835_GPIO_FSEL_OUTP);

    clock_gettime(CLOCK_MONOTONIC, &ts);

    motej( 200 , 0 , 120 );
    motej( 250 , 0 , 100 );
    motej( 300 , 0 , 70 );
    motej( 3200*20 , 0 , 50 );
    return 0;

    for( int i = 0 ; i < 3 ; i++ )
    {
    motej( 100 , 1 , 120 );
    motej( 3200 , 0 , 80 );
    motej( 100 , 0 , 120 );
    motej( 3200 , 1 , 140 );
    motej( 3200 , 0 , 80 );
    motej( 100 , 0 , 120 );
    motej( 3200 , 1 , 80 );
    motej( 100 , 1 , 120 );
    motej( 3200 , 0 , 140 );
    motej( 3200 , 1 , 80 );
    motej( 100 , 1 , 120 );
    motej( 3200 , 0 , 80 );
    motej( 100 , 0 , 120 );
    }
    return 0;
}

