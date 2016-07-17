/**
 * @file fuzzy.c
 *
 * fuzzy
 *
 * @author Ricardo <tsao.ricardo@iac.com.tw>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <conio.h>

#include "system.h"
#include "../platform/platform.h"

#define DEBUG false

#define MAX(A,B)  ((A) > (B) ? (A) : (B))
#define MIN(A,B)  ((A) < (B) ? (A) : (B))

/*
  2 input fuzzy controller for differential speed control system.  Controller has
  5 membship functions for each input and 5 membership functions for the output.
  Center-of-gravity is used for defuzzification.
*/

typedef struct in_mem {
  double width;         /* Input membership function width (1/2 of triangle base).  */
  double *center;       /* Center of each input membership function.                */
  double *dom;          /* Degree of membership for each membership function.       */
} IN_MEM;

typedef struct out_mem {
  double width;         /* Output membership function width (1/2 of triangle base). */
  double *center;       /* Center of each output membership function.               */
} OUT_MEM;

typedef struct fuz_sys {
  IN_MEM  *emem;        /* Groups all fuzzy system parameters in a single variable. */
  IN_MEM  *edotmem;
  OUT_MEM *outmem;
} FUZ_SYS;

/* Function Prototypes: */

void fuzzy_init(FUZ_SYS *fuzzy_system);
void fuzzy_free(FUZ_SYS *fuzzy_system);
double fuzzy_control(double e, double edot, FUZ_SYS *fuzzy_system);
void fuzzyify(double u, IN_MEM *mem);
double leftall(double u, double w, double c);
double rightall(double u, double w, double c);
double triangle(double u, double w, double c);
void match(const IN_MEM *emem, const IN_MEM *edotmem, int *pos);
double inf_defuzz(IN_MEM *emem, IN_MEM *edotmem, OUT_MEM *outmem, int *pos);

void fuzzy_init(FUZ_SYS *fuzzy_system) {

/* Define the input and output membership functions. */

  int i;

  /* Allocate memory for membership functions. */
  if (!(fuzzy_system->emem = (IN_MEM *) malloc(sizeof(IN_MEM)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->edotmem = (IN_MEM *) malloc(sizeof(IN_MEM)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->outmem = (OUT_MEM *) malloc(sizeof(OUT_MEM)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->emem->center = (double *) malloc(5*sizeof(double)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->emem->dom = (double *) malloc(5*sizeof(double)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->edotmem->center = (double *) malloc(5*sizeof(double)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->edotmem->dom = (double *) malloc(5*sizeof(double)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }
  if (!(fuzzy_system->outmem->center = (double *) malloc(5*sizeof(double)))) {
    printf("[ERROR] allocating memory.\n");
    exit(1);
  }

  /* Initialize for inverted pendulum. */
  fuzzy_system->emem->width = M_PI/4.0;  /* Width defined to be 1/2 of triangle base. */
  fuzzy_system->edotmem->width = M_PI/8.0;
  fuzzy_system->outmem->width = 10;

  for (i=0; i<5; i++) {
    fuzzy_system->emem->center[i] = (-M_PI/2.0 + i*M_PI/4.0);
    fuzzy_system->edotmem->center[i] = (-M_PI/4.0 + i*M_PI/8.0);
    fuzzy_system->outmem->center[i] = (-20.0 + i*10.0);
  }
}

void fuzzy_free(FUZ_SYS *fuzzy_system) {

/* Free memory allocated in fuzzy_init(). */

  free(fuzzy_system->emem->center);
  free(fuzzy_system->emem->dom);
  free(fuzzy_system->edotmem->center);
  free(fuzzy_system->edotmem->dom);
  free(fuzzy_system->outmem->center);
  free(fuzzy_system->emem);
  free(fuzzy_system->edotmem);
  free(fuzzy_system->outmem);
}

double fuzzy_control(double e, double edot, FUZ_SYS *fuzzy_system) {

/* Given crisp inputs e and edot, determine the crisp output u. */

  int pos[2];

  fuzzyify(e, fuzzy_system->emem);
  fuzzyify(edot, fuzzy_system->edotmem);
  match(fuzzy_system->emem, fuzzy_system->edotmem, pos);
  return inf_defuzz(fuzzy_system->emem, fuzzy_system->edotmem, fuzzy_system->outmem, pos) * (-1.0);
}

void fuzzyify(double u, IN_MEM *mem) {

/* Fuzzify the input u by determining the degree of membership for each membership
   function in mem. Assumes 5 membership functions, with first and last membership
   functions leftall and rightall respectively.  Other membership functions are
   triangular. */

  int i;

  mem->dom[0] = leftall(u, mem->width, mem->center[0]);
  for (i=1; i<4; i++)
    mem->dom[i] = triangle(u, mem->width, mem->center[i]);
  mem->dom[4] = rightall(u, mem->width, mem->center[4]);
}

double leftall(double u, double w, double c)

/* Determine degree of membership for a leftall membership function.
   NOTE:  u is input, c is mem. fun. center, and w is mem. fun. width. */

{
  if (u < c)
    return 1.0;
  else
    return MAX(0,(1-(u-c)/w));
}

double rightall(double u, double w, double c)
/* Determine degree of membership for a RIGHTALL membership function
   NOTE:  u is input, c is mem. fun. center, and w is mem. fun. width. */

{
  if (u >= c)
         return 1.0;
  else
         return MAX(0,(1-(c-u)/w));
}

double triangle(double u, double w, double c)

/* Determine degree of membership for a TRIANGLE membership function
   NOTE:  u is input, c is mem. fun. center, and w is mem. fun. width. */

{
  if (u >= c)
    return MAX(0,(1-(u-c)/w));
  else
    return MAX(0,(1-(c-u)/w));
}

void match(const IN_MEM *emem, const IN_MEM *edotmem, int *pos) {

/* For each universe of discourse, determine the index of the first membership function
   with a non-zero degree (i.e. match the rules to the current inputs to find which rules
   are on).  These indices are used to determine which four rules to evaluate.  (NOTE:
   A 2 input sytem with no more than 50% overlap for input membership functions only
   requires the evaluation of at most 4 rules.) */

  int i;

  for (i=0; i<5; i++) {
    if(emem->dom[i] != 0) {
      pos[0] = i;
      break;
    }
  }
  for (i=0; i<5; i++) {
    if(edotmem->dom[i] != 0) {
      pos[1] = i;
      break;
    }
  }
}

double inf_defuzz(IN_MEM *emem, IN_MEM *edotmem, OUT_MEM *outmem, int *pos) {

/* We use the degrees of membership found in the function match() to form the implied
   fuzzy sets. The correct output membership function for each rule is determined by
   adding (and saturating) a shifted version of the input membership function indices
   (this implements the typical pattern of linguistic-numeric indices in the body of
   the table of rules).  In this way we compute the rule-base at every step, rather
   than storing the rule-base in a table.  Defuzzification is also performed using
   the center-of-gravity method.  A crisp output is returned. */


  double outdom, area, Atot = 0, WAtot = 0;
  int i, j, out_index;

  for(i=0; i<2; i++) {
    for(j=0; j<2; j++) {
      if ( ((pos[0]+i)<5) && ((pos[1]+j)<5)) { /* Check that bounds are not exceeded. */
        outdom = 0;

        /* Shift indices left. */
        out_index = ((pos[0]+i)-2) + ((pos[1]+j)-2);

        /* Saturate */
        if (out_index < -2)
          out_index = -2;
        else if (out_index > 2)
          out_index = 2;

        /* Shift indices right.*/
        out_index += 2;

        /* Determine the certainty of the premise */
        outdom = MIN((emem->dom[pos[0]+i]), (edotmem->dom[pos[1]+j]));

        /* Defuzzify */
        area = 2*outmem->width*(outdom - (outdom*outdom)/2);
        Atot += area;
        WAtot += area*outmem->center[out_index];
      }
    }
  }
  /* Return the crisp value.  Minus sign required to give correct output for
     pendulum system!  Note that this minus sign actually ensures that the table of
     indices works out as shown in class. */

  return -(WAtot/Atot);
}

FUZ_SYS fuzzy_system;

bool fuzzy_control_init(system_data* sd)
{
    fuzzy_init(&fuzzy_system);

    return true;
}

bool fuzzy_control_update(system_data* sd)
{
    static float vx_err_prev, vy_err_prev, w0_err_prev;
    static float vx_err, vy_err, w0_err;
    static float vx_err_dif, vy_err_dif, w0_err_dif;

    sd->t_prev = sd->t_curr;
    sd->t_curr = clock();

    sd->t_delta = sd->t_curr - sd->t_prev;

    if(sd->t_delta < 0.0f)
    {
        MSG(sd->log, "[ERROR] pid_control_update, failed! \n");
        return false;
    }

    #if DEBUG
    MSG(sd->log, "[DEBUG] pid_control_update : \n");
    MSG(sd->log, "t_prev, t_curr, t_delta (ms) = \n");
    MSG(sd->log, "%9.4ld %9.4ld %9.4ld \n\n", sd->t_prev, sd->t_curr, sd->t_delta);
    #endif

    /* proportional */
    vx_err = sd->sv.vx - sd->pv.vx;
    vy_err = sd->sv.vy - sd->pv.vy;
    w0_err = sd->sv.w0 - sd->pv.w0;

    /* derivative */
    vx_err_dif = (vx_err - vx_err_prev) / (sd->t_delta / 1000.0f) ;
    vy_err_dif = (vy_err - vy_err_prev) / (sd->t_delta / 1000.0f) ;
    w0_err_dif = (w0_err - w0_err_prev) / (sd->t_delta / 1000.0f) ;

    vx_err_prev = vx_err;
    vy_err_prev = vy_err;
    w0_err_prev = w0_err;

    sd->cv.vx += (float)fuzzy_control((double)vx_err, (double)vx_err_dif, &fuzzy_system) * 0.004f;
    sd->cv.vy += (float)fuzzy_control((double)vy_err, (double)vy_err_dif, &fuzzy_system) * 0.004f;
    sd->cv.w0 += (float)fuzzy_control((double)w0_err, (double)w0_err_dif, &fuzzy_system) * 0.004f;

    return true;
}
