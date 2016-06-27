/**
 * @file digital_16.h
 *
 * fake digital 16 bit guide sensor
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include "../../../common/system.h"
#include "../../../platform/platform.h"
#include "../common_cfg.h"
#include "digital_16.h"

/* MGT-50-25 (MGT Series Magnetic Guidance Tape) */
#define MAG_TAPE_WIDTH		50 // mm

/* MA16 */
#define D16_SENSE_RATE		100 // Hz
#define D16_SENSE_BIT_NUM	16 // bit
#define D16_SENSE_PITCH 	10 // mm
#define D16_SENSE_RANGE 	(D16_SENSE_BIT_NUM * D16_SENSE_PITCH) // mm
#define D16_SENSE_RES		(D16_SENSE_RANGE / D16_SENSE_BIT_NUM) // mm
#define D16_SENSE_FOV		atan2(D16_SENSE_RANGE/2, DEFAULT_GUIDE_SENSOR_OFFSET) * 2 // radian
#define D16_SENSE_RAD		DEFAULT_GUIDE_SENSOR_OFFSET * D16_SENSE_FOV // mm, arc length
#define D16_SENSE_MARK_BIT  MAG_TAPE_WIDTH / D16_SENSE_RES // bit

system_state ss;

void MyFunc(void *data)
{
	uint16_t yaw = 0;

    for(;;)
    { 
        sys_get_status(&ss);
        
        ss.yaw =  ss.yaw + 0.1f;
        
		sys_set_status(&ss);

        sleep(2);
    }     
}

uint16_t guide_sensor_d16_init(uint16_t offset)
{
    //srand( (unsigned volatile)time(NULL) ); // 以時間序列當亂數種子
    //srand((unsigned) time(NULL) + getpid());
    pthread_t tid;
    pthread_create(&tid, NULL, (void *)&MyFunc, NULL);

	uint16_t data = rand();
	
	return data;
}

void guide_sensor_d16_info(void)
{
	uint16_t i = 0;
	float randx = 0.0f;

	printf("------------------------------------- \n");
	printf("Fake Digital 16 Bit Guide Sensor INFO \n");
	printf("------------------------------------- \n");
	printf("DEFAULT_GUIDE_SENSOR_OFFSET = %f (mm) \n", (float)DEFAULT_GUIDE_SENSOR_OFFSET);
	printf("D16_SENSE_MARK_BIT = %d (bit) \n", D16_SENSE_MARK_BIT);
	printf("D16_SENSE_RANGE = %f (mm) \n", (float)D16_SENSE_RANGE);
	printf("D16_SENSE_RES = %f (mm) \n", (float)D16_SENSE_RES);
	printf("D16_SENSE_FOV = %f (rad), %f (deg) \n", (float)D16_SENSE_FOV, (float)D16_SENSE_FOV * 180.0/M_PI );
	printf("D16_SENSE_RAD = %f (mm) \n", (float)D16_SENSE_RAD);
	printf("------------------------------------- \n");

	#if 0
	for(i = 0; i < 20; i++)
	{
		randx = (double) rand() / (RAND_MAX + 1.0f );
		randx = randx * D16_SENSE_RAD;
		printf("guide sensor data = %f \n", randx);
	}
	#endif
}
