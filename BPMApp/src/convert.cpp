/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */


/* Libera position calculations and conversions. */

/* 29/07/2015 (Lucas Russo) Deleted/modified Libera specific routines */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "cordic.h"
#include "numeric.h"

#include "convert.h"

/*****************************************************************************/
/*                                                                           */
/*                              Static State                                 */
/*                                                                           */
/*****************************************************************************/

/* The following global parameters are used to control the calculation of
 * electron beam position from button signal level readout. */

#define K_SCALE 1000000

/* Scaling factors.  These convert relative intensities into electron beam
 * positions and are in units of distance.  The scaling factor is determined
 * by the geometry of the button or stripline assembly.
 *     These values are in units of nm and cannot be set larger than 32mm
 * without causing numerical overflow later on in the processing chain! */

//static int K_X = 10 * K_SCALE;  // 10mm: largely reasonable defaults
//static int K_Y = 10 * K_SCALE;

/* Electron beam zero point offsets.  These are used to adjust the nominal
 * zero point returned.  These are stored in nm.
 *
 * We compute
 *      X_0 = BBA_X + BCD_X + GOLDEN_X
 *      Y_0 = BBA_Y + BCD_Y + GOLDEN_Y
 * and apply the offset X_0, Y_0 globally.
 *
 * However, the sum BBA+BCD is intended as a nominal zero for interlock
 * calculations.  Thus when GOLDEN is subtracted from the beam position in
 * the FPGA calculation we also need to shift the interlock window
 * accordingly. */
//static int X_0 = 0;
//static int Y_0 = 0;

//static int Q_0 = 0;

//static int BBA_X = 0;
//static int BBA_Y = 0;
//static int BCD_X = 0;
//static int BCD_Y = 0;
//static int GOLDEN_X = 0;
//static int GOLDEN_Y = 0;

/* Button gain adjustments.  By default we start with gain of 1.  See
 * SCALE_GAIN macro below. */
#define DEFAULT_GAIN    (1 << 30)
static int ChannelGain[4] =
    { DEFAULT_GAIN, DEFAULT_GAIN, DEFAULT_GAIN, DEFAULT_GAIN };

/* Rescales value by gain factor making use of the GAIN_OFFSET defined above.
 * The gain factors are scaled by a factor of 2^31, and are intended to
 * always be <= 1.  The value to be scaled derives from a CORDIC computation,
 * and will be comfortably less that 2^31. */
#define SCALE_GAIN(gain, value) \
    ((int) (((long long int) (gain) * (value)) >> 30))


/* This flag determines the beam orientation: either diagonal or vertical.
 * Note that the Z axis (or S, as accelerator physicists call it) is *into*
 * the page, and X points out of the ring.
 *
 *       ^ Y        A                   A       B
 *       |      D   *   B                   *
 * X <---+-         C                   D       C
 *       |
 *              Vertical                Diagonal
 *
 * The default configuration is diagonal: this is the normal arrangement with
 * buttons in an oblong cross-section vacuum vessel for a synchrotron ring.
 * The vertical configuration can arise when buttons or striplines are
 * arranged around a circular vacuum vessel in a linear accelerator or
 * transfer path. */

/* This flasg was covnerted to ABCDtoXYWS parameter so we can have both
 * values calculated dinamically */
//static bool Diagonal = true;

/*****************************************************************************/
/*                                                                           */
/*                           Conversion Routines                             */
/*                                                                           */
/*****************************************************************************/


/* The total intensity for each button is the magnitude of its IQ data: we
 * perform the reduction using cordic for which we have a very fast algorithm
 * available. */

void IQtoABCD(const IQ_ROW *IQ, ABCD_ROW *ABCD, int Count)
{
    for (int i = 0; i < Count; i ++)
    {
        const IQ_ROW & iq = IQ[i];
        ABCD_ROW & abcd = ABCD[i];
        abcd.A = CordicMagnitude(iq.AI, iq.AQ);
        abcd.B = CordicMagnitude(iq.BI, iq.BQ);
        abcd.C = CordicMagnitude(iq.CI, iq.CQ);
        abcd.D = CordicMagnitude(iq.DI, iq.DQ);
    }
}



/* Computes K * M / S without loss of precision.  We use our knowledge of the
 * arguments to do this work as efficiently as possible.  The algorithm
 * computes:
 *
 *      position = K M / S
 *
 *                  shift
 *               = 2      InvS K M      (case 1 below)
 *
 *                  a+b-64
 *               = 2       InvS K M     (assuming no overflow: 2, 3 below)
 *
 *                  -32    -32   a              b
 *               = 2    (2     (2  K * InvS) * 2  M)
 *
 * (inner multiplication unsigned, outer mixed unsigned/signed) so that:
 *
 *   1.  a + b - 64 = shift
 *   2.  2^a K < 2^32
 *   3.  |2^b M| < 2^31
 *
 * From construction of (InvS, shift) we know:
 *
 *   4.  InvS = 2^shift / S
 *   5.  2^31 <= InvS < 2^32
 *
 * From (4, 5) we can infer that S <= 2^(shift-31), from construction we know
 * that |M| <= S, and in practice we can safely assume |M| < S, so by setting
 * b = 62-shift we get
 *
 *      | 62-shift  |    62-shift       62-shift  shift-31    31
 *      |2         M| < 2         S <= 2         2         = 2   ,
 *
 * and so a = shift+64-b = 2 gives us plenty of headroom for K. */

static int DeltaToPosition(int K, int M, int InvS, int shift)
{
    //printf ("K << 2 = %d\n", K << 2);
    //printf ("InvS = %d\n", InvS);
    //printf ("Shift = %d\n", shift);
    //printf ("MulUU = %d\n", MulUU(K << 2, InvS));
    //printf ("M << (62 - shift) = %d\n", M << (62 - shift));
    //fflush(stdout);
    return MulUS(MulUU(K << 2, InvS), M << (62 - shift));
}


/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function.  The underlying model for the
 * transfer of electron beam intensity to buttons simplifies to a model where
 * we can write
 *
 *              Vertical                        Diagonal
 *
 *              A = I * (1 + Y/K)               A = I * (1 + X/K + Y/K)
 *              B = I * (1 - X/K)               B = I * (1 - X/K + Y/K)
 *              C = I * (1 - Y/K)               C = I * (1 - X/K - Y/K)
 *              D = I * (1 + X/K)               D = I * (1 + X/K - Y/K)
 *
 * where I is proportional to beam intensity and we are neglecting terms of
 * order X^2, Y^2 and XY.  Given this model we can calculate
 *
 *      S = A + B + C + D = 4 * I
 *      Q = A - B + C - D = 0
 *              D_X = D - B = 2*I*X/K           D_X = A - B - C + D = 4*I*X/K
 *              D_Y = A - C = 2*I*Y/K           D_Y = A + B - C - D = 4*I*Y/K
 *
 * and thus
 *              X = 2*K * (D - B) / S           X = K * (A - B - C + D) / S
 *              Y = 2*K * (A - C) / S           X = K * (A + B - C - D) / S .
 */

void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, K_FACTORS *K, POS_OFFSETS *OFFSETS,
        int Count, bool Diagonal)
{
    K_FACTORS k_factors = *K;
    POS_OFFSETS pos_offsets = *OFFSETS;

    for (int i = 0; i < Count; i++)
    {
        const ABCD_ROW & abcd = ABCD[i];
        XYQS_ROW & xyqs = XYQS[i];

        /* First compute the total intensity S.  To avoid overflow we
         * prescale by 4.  This can involve loss of bits when the intensity
         * is extremely low, but in fact the bottom bits are pretty well pure
         * noise and can be cheaply discarded.
         *    The button values A,B,C,D are known to lie in the range 0 to
         * 2^31 - 1 so we similarly know that 0 <= S < 2^31. */
        /* *FIXME: we are not scaling down the amplitudes as it was before */
        uint32_t A = abs(abcd.A /* >> 2*/);
        uint32_t B = abs(abcd.B /* >> 2*/);
        uint32_t C = abs(abcd.C /* >> 2*/);
        uint32_t D = abs(abcd.D /* >> 2*/);

        /* --- Experimental implementation of partial diffence-over-sum mehtod ---
         * Only meant for tests. Author: Daniel Tavares */
        uint32_t S_AC = A + C;
        uint32_t S_BD = B + D;
        uint32_t S = S_AC + S_BD;

        /* Now compute the positions according to the model.  As this is an
         * inner loop function we take some time to optimise its execution by
         * precomputing as much as possible.
         *    Start by precomputing 1/S, or more precisely, a scaled version
         * of 1/S.  (InvS,shift) = Reciprocal(S) returns InvS=2^shift/S,
         * where shift derives from a bit normalisation count on S so that
         * 2^31 <= InvS < 2^32. */
        int shift_AC = 0;
        int InvS_AC = Reciprocal(S_AC << 1, shift_AC);

        int shift_BD = 0;
        int InvS_BD = Reciprocal(S_BD << 1, shift_BD);

        /* KX and XY should be divided by 2 in partial difference-over-sum. Shift partial sums by 1 bit to implement it. */
        uint32_t partial_AC_pos = DeltaToPosition(k_factors.KX, A - C, InvS_AC, shift_AC);
        uint32_t partial_BD_pos = DeltaToPosition(k_factors.KY, B - D, InvS_BD, shift_BD);

        /* Compute X and Y according to the currently selected detector
         * orientation. */
        if (Diagonal)
        {
            xyqs.X = partial_AC_pos - partial_BD_pos - pos_offsets.XOFFSET;
            xyqs.Y = partial_AC_pos + partial_BD_pos - pos_offsets.YOFFSET;
        }
        else
        {
            xyqs.X = -partial_BD_pos - pos_offsets.XOFFSET;
            xyqs.Y = partial_AC_pos - pos_offsets.YOFFSET;
        }
        /* We scale Q up quite a bit more so that we have access to as much
         * information as possible: the values can be quite small,
         * particulary after Q_0 correction. */
        /* xyqs.Q = DeltaToPosition(
            100 * K_SCALE, A - B + C - D, InvS, shift) - Q_0; */
        xyqs.Q = partial_AC_pos + partial_BD_pos - pos_offsets.QOFFSET;
        /* --- Experimental implementation of partial diffence-over-sum mehtod end here --- */
        /* xyqs.S = S; */
        xyqs.S = k_factors.KSUM * S;
    }
}

void GainCorrect(int Channel, int *Column, int Count)
{
    int Gain = ChannelGain[Channel];
    for (int i = 0; i < Count; i ++)
        Column[i] = SCALE_GAIN(Gain, Column[i]);
}
