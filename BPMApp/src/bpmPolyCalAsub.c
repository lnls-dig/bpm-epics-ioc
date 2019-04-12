/* bpmPolyCalAsub.cpp: Process BPM positions and corrects then via polynomial */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

#include "registryFunction.h"
#include "aSubRecord.h"
#include "epicsExport.h"
#include "link.h"
#include "dbAddr.h"
#include "dbCommon.h" /* precord: now = paddr->precord->time;*/
#include "epicsTime.h"
#include "waveformRecord.h"

/* FIXME. This is bound to the environment variable WAVEFORM_MAX_POINTS set in BPM.config */
#define WAVEFORM_MAX_POINTS 100000

static int bpmPolyCalProcessAsubDebug = 0;

typedef long (*processMethod)(aSubRecord *precord);

static long bpmPolyCalInitAsub(aSubRecord *precord,processMethod process)
{
    if (bpmPolyCalProcessAsubDebug) {
        printf("Record %s called bpmPolyCalInitAsub(%p, %p), initial value is: %d\n",
               precord->name, (void*) precord, (void*) process, precord->val);
    }
    return 0;
}

static long bpmPolyCalProcessAsub(aSubRecord *prec)
{
    double *in_x = (double *) prec->a;
    double *in_y = (double *) prec->b;
    double *in_q = (double *) prec->c;
    double *in_sum = (double *) prec->d;
    double *in_coeff_x = (double *) prec->e;
    double coeffx1_x = in_coeff_x[0];
    double coeffx1y2_x = in_coeff_x[1];
    double coeffx1y4_x = in_coeff_x[2];
    double coeffx1y6_x = in_coeff_x[3];
    double coeffx1y8_x = in_coeff_x[4];
    double coeffx3y0_x = in_coeff_x[5];
    double coeffx3y2_x = in_coeff_x[6];
    double coeffx3y4_x = in_coeff_x[7];
    double coeffx3y6_x = in_coeff_x[8];
    double coeffx5y0_x = in_coeff_x[9];
    double coeffx5y2_x = in_coeff_x[10];
    double coeffx5y4_x = in_coeff_x[11];
    double coeffx7y0_x = in_coeff_x[12];
    double coeffx7y2_x = in_coeff_x[13];
    double coeffx9y0_x = in_coeff_x[14];
    double *in_coeff_y = (double *) prec->f;
    double coeffy1_y = in_coeff_y[0];
    double coeffy1x2_y = in_coeff_y[1];
    double coeffy1x4_y = in_coeff_y[2];
    double coeffy1x6_y = in_coeff_y[3];
    double coeffy1x8_y = in_coeff_y[4];
    double coeffy3x0_y = in_coeff_y[5];
    double coeffy3x2_y = in_coeff_y[6];
    double coeffy3x4_y = in_coeff_y[7];
    double coeffy3x6_y = in_coeff_y[8];
    double coeffy5x0_y = in_coeff_y[9];
    double coeffy5x2_y = in_coeff_y[10];
    double coeffy5x4_y = in_coeff_y[11];
    double coeffy7x0_y = in_coeff_y[12];
    double coeffy7x2_y = in_coeff_y[13];
    double coeffy9x0_y = in_coeff_y[14];
    double *out_x = (double *) prec->vala;
    double *out_y = (double *) prec->valb;
    int i = 0;
    
    /* Calculate polynomial correction */
    for (i = 0; i < prec->noa; ++i) {
        double x = in_x[i];
        double y = in_y[i];
        double y2 = y*y;
        double y3 = y2*y;
        double y4 = y2*y2;
        double y5 = y3*y2;
        double y6 = y4*y2;
        double y7 = y4*y3;
        double y8 = y6*y2;
        double y9 = y6*y3;
        double x2 = x*x; 
        double x3 = x2*x; 
        double x4 = x2*x2; 
        double x5 = x3*x2; 
        double x6 = x4*x2; 
        double x7 = x5*x2; 
        double x8 = x6*x2; 
        double x9 = x6*x3;

        out_x[i] = coeffx1_x*x*(1+coeffx1y2_x*y2+coeffx1y4_x*y4+coeffx1y6_x*y6+coeffx1y8_x*y8) + 
                   coeffx3y0_x*x3*(1+coeffx3y2_x*y2+coeffx3y4_x*y4+coeffx3y6_x*y6) +
                   coeffx5y0_x*x5*(1+coeffx5y2_x*y2+coeffx5y4_x*y4) + 
                   coeffx7y0_x*x7*(1+coeffx7y2_x*y2) + 
                   coeffx9y0_x*x9;
        out_y[i] = coeffy1_y*y*(1+coeffy1x2_y*x2+coeffy1x4_y*x4+coeffy1x6_y*x6+coeffy1x8_y*x8) + 
                   coeffy3x0_y*y3*(1+coeffy3x2_y*x2+coeffy3x4_y*x4+coeffy3x6_y*x6) +
                   coeffy5x0_y*y5*(1+coeffy5x2_y*x2+coeffy5x4_y*x4) + 
                   coeffy7x0_y*y7*(1+coeffy7x2_y*x2) + 
                   coeffy9x0_y*y9;
    }
       
    return 0;
}

/* Register these symbols for use by IOC code: */
epicsRegisterFunction(bpmPolyCalInitAsub);
epicsRegisterFunction(bpmPolyCalProcessAsub);
