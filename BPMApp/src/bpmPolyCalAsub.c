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

static int bpmPolyCalProcessAsubDebug = 1;

typedef long (*processMethod)(aSubRecord *precord);

static long bpmPolyCalInitAsub(aSubRecord *precord,processMethod process)
{
    if (bpmPolyCalProcessAsubDebug) {
        printf("Record %s called bpmPolyCalInitAsub(%p, %p), initial value is: %d\n",
               precord->name, (void*) precord, (void*) process, precord->val);
    }
    return 0;
}

static long bpmPolyCalXYProcessAsub(aSubRecord *prec)
{
    double *in_x = (double *) prec->a;
    double *in_y = (double *) prec->b;
    double *in_q = (double *) prec->c;
    double *in_sum = (double *) prec->d;
    double *in_coeff_x = (double *) prec->e;
    double coeffx1y0_x = in_coeff_x[0];
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
    double coeffx0y1_y = in_coeff_y[0];
    double coeffx2y1_y = in_coeff_y[1];
    double coeffx4y1_y = in_coeff_y[2];
    double coeffx6y1_y = in_coeff_y[3];
    double coeffx8y1_y = in_coeff_y[4];
    double coeffx0y3_y = in_coeff_y[5];
    double coeffx2y3_y = in_coeff_y[6];
    double coeffx4y3_y = in_coeff_y[7];
    double coeffx6y3_y = in_coeff_y[8];
    double coeffx0y5_y = in_coeff_y[9];
    double coeffx2y5_y = in_coeff_y[10];
    double coeffx4y5_y = in_coeff_y[11];
    double coeffx0y7_y = in_coeff_y[12];
    double coeffx2y7_y = in_coeff_y[13];
    double coeffx0y9_y = in_coeff_y[14];
    long in_offs_x = *(long *) prec->g;
    long in_offs_y = *(long *) prec->h;
    double *out_x = (double *) prec->vala;
    double *out_y = (double *) prec->valb;
    int i = 0;

    /* Calculate polynomial correction */
    for (i = 0; i < prec->noa; ++i) {
        double x1 = in_x[i];
        double y1 = in_y[i];
        double y2 = y1*y1;
        double y3 = y2*y1;
        double y4 = y2*y2;
        double y5 = y3*y2;
        double y6 = y4*y2;
        double y7 = y4*y3;
        double y8 = y6*y2;
        double y9 = y6*y3;
        double x2 = x1*x1;
        double x3 = x2*x1;
        double x4 = x2*x2;
        double x5 = x3*x2;
        double x6 = x4*x2;
        double x7 = x5*x2;
        double x8 = x6*x2;
        double x9 = x6*x3;

        out_x[i] = (x1*(coeffx1y0_x + coeffx1y2_x*y2  + coeffx1y4_x*y4  + coeffx1y6_x*y6  + coeffx1y8_x*y8) +
                    x3*(coeffx3y0_x + coeffx3y2_x*y2  + coeffx3y4_x*y4  + coeffx3y6_x*y6) +
                    x5*(coeffx5y0_x + coeffx5y2_x*y2  + coeffx5y4_x*y4) +
                    x7*(coeffx7y0_x + coeffx7y2_x*y2) +
                    x9*(coeffx9y0_x)) - 
                   in_offs_x;
        out_y[i] = (y1*(coeffx0y1_y + coeffx2y1_y*x2  + coeffx4y1_y*x4  + coeffx6y1_y*x6  + coeffx8y1_y*x8) +
                    y3*(coeffx0y3_y + coeffx2y3_y*x2  + coeffx4y3_y*x4  + coeffx6y3_y*x6) +
                    y5*(coeffx0y5_y + coeffx2y5_y*x2  + coeffx4y5_y*x4) +
                    y7*(coeffx0y7_y + coeffx2y7_y*x2) +
                    y9*(coeffx0y9_y)) - 
                   in_offs_y;
    }

    return 0;
}

static long bpmPolyCalQProcessAsub(aSubRecord *prec)
{
    double *in_x = (double *) prec->a;
    double *in_y = (double *) prec->b;
    double *in_q = (double *) prec->c;
    double *in_sum = (double *) prec->d;
    double *in_coeff_q = (double *) prec->e;
    double coeffx1y1_q = in_coeff_q[0];
    double coeffx1y3_q = in_coeff_q[1];
    double coeffx1y5_q = in_coeff_q[2];
    double coeffx3y1_q = in_coeff_q[3];
    double coeffx3y3_q = in_coeff_q[4];
    double coeffx3y5_q = in_coeff_q[5];
    double coeffx5y1_q = in_coeff_q[6];
    double coeffx5y3_q = in_coeff_q[7];
    double coeffx5y5_q = in_coeff_q[8];
    long in_offs_q = *(long *) prec->f;
    double *out_q = (double *) prec->vala;
    int i = 0;

    /* Calculate polynomial correction */
    for (i = 0; i < prec->noa; ++i) {
        double x1 = in_x[i];
        double y1 = in_y[i];
        double q1 = in_q[i];
        double y2 = y1*y1;
        double y4 = y2*y2;
        double x2 = x1*x1;
        double x3 = x2*x1;
        double x5 = x3*x2;

        out_q[i] = q1 -
                   (x1*y1*(coeffx1y1_q + coeffx1y3_q*y2 + coeffx1y5_q*y4) +
                    x3*y1*(coeffx3y1_q + coeffx3y3_q*y2 + coeffx3y5_q*y4) +
                    x5*y1*(coeffx5y1_q + coeffx5y3_q*y2 + coeffx5y5_q*y4)) -
                   in_offs_q;
    }

    return 0;
}

static long bpmPolyCalSUMProcessAsub(aSubRecord *prec)
{
    double *in_x = (double *) prec->a;
    double *in_y = (double *) prec->b;
    double *in_q = (double *) prec->c;
    double *in_sum = (double *) prec->d;
    double *in_coeff_sum = (double *) prec->e;
    double coeffx0y0_sum = in_coeff_sum[0];
    double coeffx0y2_sum = in_coeff_sum[1];
    double coeffx0y4_sum = in_coeff_sum[2];
    double coeffx0y6_sum = in_coeff_sum[3];
    double coeffx2y0_sum = in_coeff_sum[4];
    double coeffx2y2_sum = in_coeff_sum[5];
    double coeffx2y4_sum = in_coeff_sum[6];
    double coeffx2y6_sum = in_coeff_sum[7];
    double coeffx4y0_sum = in_coeff_sum[8];
    double coeffx4y2_sum = in_coeff_sum[9];
    double coeffx4y4_sum = in_coeff_sum[10];
    double coeffx4y6_sum = in_coeff_sum[11];
    double coeffx6y0_sum = in_coeff_sum[12];
    double coeffx6y2_sum = in_coeff_sum[13];
    double coeffx6y4_sum = in_coeff_sum[14];
    double coeffx6y6_sum = in_coeff_sum[15];
    long in_offs_sum = *(long *) prec->f;
    double *out_sum = (double *) prec->vala;
    int i = 0;

    /* Calculate polynomial correction */
    for (i = 0; i < prec->noa; ++i) {
        double x1 = in_x[i];
        double y1 = in_y[i];
        double sum1 = in_sum[i];
        double y2 = y1*y1;
        double y3 = y2*y1;
        double y4 = y2*y2;
        double y6 = y4*y2;
        double x2 = x1*x1;
        double x3 = x2*x1;
        double x4 = x2*x2;
        double x6 = x4*x2;

        double sum_poly = 1 *(coeffx0y0_sum + coeffx0y2_sum*y2 + coeffx0y4_sum*y4 + coeffx0y6_sum*y6) +
                          x2*(coeffx2y0_sum + coeffx2y2_sum*y2 + coeffx2y4_sum*y4 + coeffx2y6_sum*y6) +
                          x4*(coeffx4y0_sum + coeffx4y2_sum*y2 + coeffx4y4_sum*y4 + coeffx4y6_sum*y6) +
                          x6*(coeffx6y0_sum + coeffx6y2_sum*y2 + coeffx6y4_sum*y4 + coeffx6y6_sum*y6);
        if(sum_poly == 0) {
            return -1;
        }

        out_sum[i] = (sum1 / sum_poly) - in_offs_sum;
    }

    return 0;
}

/* Register these symbols for use by IOC code: */
epicsRegisterFunction(bpmPolyCalInitAsub);
epicsRegisterFunction(bpmPolyCalXYProcessAsub);
epicsRegisterFunction(bpmPolyCalQProcessAsub);
epicsRegisterFunction(bpmPolyCalSUMProcessAsub);
