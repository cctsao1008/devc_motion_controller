#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "cutest.h"

#include "system.h"
#include "..\platform\platform.h"

void test_kinematics(void)
{
	system_data* sd;
	sd = system_init();
	
	TEST_CHECK(sd != NULL);
	TEST_CHECK(kinematics_init(sd) != false);
	
	sd->cv.vx = 1;
	sd->cv.vy = 1;
	sd->cv.w0 = 1;
    
    TEST_CHECK(inverse_kinematics(sd) != false);
    
    sd->mot.in.w1 = sd->mot.out.w1;
    sd->mot.in.w2 = sd->mot.out.w2;
    sd->mot.in.w3 = sd->mot.out.w3;
    sd->mot.in.w4 = sd->mot.out.w4;
    
    TEST_CHECK(forward_kinematics(sd) != false);
    
    TEST_CHECK(sd->pv.vx != sd->sv.vx);
    TEST_CHECK(sd->pv.vy != sd->sv.vy);
    TEST_CHECK(sd->pv.w0 != sd->sv.w0);
}

TEST_LIST = {
    { "kinematics", test_kinematics },
    { 0 }
};
