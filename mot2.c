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


#include <pigpio.h>

#define PERIOD	50
#define RUNTIME	150000
#define STOPTIME	20000

#define PIN_ASTEP	18
#define PIN_ADIR	17
#define PIN_BSTEP	4
#define PIN_BDIR	14

int main(int argc, char ** argv)
{
	if (gpioInitialise() < 0)
	{
	   // pigpio initialisation failed.
	}

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &sp);
    mlockall(MCL_CURRENT | MCL_FUTURE);


	gpioSetMode( PIN_ASTEP , PI_OUTPUT );
	gpioSetMode( PIN_BSTEP , PI_OUTPUT );
	gpioSetMode( PIN_ADIR , PI_OUTPUT );
	gpioSetMode( PIN_BDIR , PI_OUTPUT );

	gpioWaveClear();
//	gpioWaveAddNew();

#define PULSES 1
	gpioPulse_t pulse[ PULSES * 2 ]={ 0 };

	for( int i = 0 ; i < PULSES * 2 ; ++i )
	{
	    pulse[i].gpioOn = (1<<(PIN_ASTEP)) | (1<<(PIN_BSTEP));
	    pulse[i].gpioOff = 0;
	    pulse[i].usDelay = 10;
	    ++i;
	    pulse[i].gpioOn = 0;
	    pulse[i].gpioOff = (1<<(PIN_ASTEP)) | (1<<(PIN_BSTEP));
	    pulse[i].usDelay = PERIOD - 10;
	}
	gpioWaveAddGeneric( PULSES * 2 , pulse );

	int wave_id = gpioWaveCreate();

	if (wave_id >= 0)
	for( int i = 0 ; i < 1 ; i++ )
	{
	    gpioWrite( PIN_ADIR , 0 );
	    gpioWrite( PIN_BDIR , 1 );

	    gpioWaveTxSend(wave_id, 1);
	    gpioDelay(RUNTIME);
	    gpioWaveTxStop();

	    gpioDelay(STOPTIME);
	    gpioWrite( PIN_ADIR , 1 );
	    gpioWrite( PIN_BDIR , 0 );

	    gpioWaveTxSend(wave_id, 1);
	    gpioDelay(RUNTIME);
	    gpioWaveTxStop();

	    gpioDelay(STOPTIME);
	}

	gpioTerminate();
	return 0;
}
