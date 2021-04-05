/*
 * position.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Mar. 17, 2020
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "position.h"

/* Computes the reciprocal of X */

double Reciprocal(double X)
{
    return (1 / X);
}

/* Computes the Dela-over-Sigma algotithm */

static double DeltaToPosition(double M, double InvS)
{
    return (InvS * M);
}

/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function.  The underlying model for the
 * transfer of electron beam intensity to buttons simplifies to a model where
 * we can write
 *
 *              Vertical                        Diagonal
 *
 *              A = I * (1 + Y)               A = I * (1 + X + Y)
 *              B = I * (1 - X)               B = I * (1 - X + Y)
 *              C = I * (1 - Y)               C = I * (1 - X - Y)
 *              D = I * (1 + X)               D = I * (1 + X - Y)
 *
 * where I is proportional to beam intensity and we are neglecting terms of
 * order X^2, Y^2 and XY.  Given this model we can calculate
 *
 *      S = A + B + C + D = 4 * I
 *      Q = A - B + C - D = 0
 *              D_X = D - B = 2*I*X           D_X = A - B - C + D = 4*I*X
 *              D_Y = A - C = 2*I*Y           D_Y = A + B - C - D = 4*I*Y
 *
 * and thus
 *              X = 2 * (D - B) / S           X = (A - B - C + D) / S
 *              Y = 2 * (A - C) / S           X = (A + B - C - D) / S .
 */

void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS,
        int Count, bool Diagonal, bool PartialDelta)
{
    if (PartialDelta) {
        ABCDtoXYQSPartial(ABCD, XYQS, Count, Diagonal);
    }
    else {
        ABCDtoXYQSStd(ABCD, XYQS, Count, Diagonal);
    }
}

void ABCDtoXYQSStd(const ABCD_ROW *ABCD, XYQS_ROW *XYQS,
        int Count, bool Diagonal)
{
    for (int i = 0; i < Count; i++)
    {
        const ABCD_ROW & abcd = ABCD[i];
        XYQS_ROW & xyqs = XYQS[i];

        /* First compute the total intensity S */
        double A = abcd.A;
        double B = abcd.B;
        double C = abcd.C;
        double D = abcd.D;
        double S = A + B + C + D;

        /* Now compute the positions according to the model.
         *    Start by precomputing 1/S. */
        double InvS = Reciprocal(S);
        /* Compute X and Y according to the currently selected detector
         * orientation. */
        if (Diagonal)
        {
            xyqs.X = DeltaToPosition(A - B - C + D, InvS);
            xyqs.Y = DeltaToPosition(A + B - C - D, InvS);
        }
        else
        {
            xyqs.X = (DeltaToPosition(D - B, InvS) * 2.0);
            xyqs.Y = (DeltaToPosition(A - C, InvS) * 2.0);
        }

        xyqs.Q = DeltaToPosition(A - B + C - D, InvS);
        xyqs.S = S;
    }
}

void ABCDtoXYQSPartial(const ABCD_ROW *ABCD, XYQS_ROW *XYQS,
        int Count, bool Diagonal)
{
    for (int i = 0; i < Count; i++)
    {
        const ABCD_ROW & abcd = ABCD[i];
        XYQS_ROW & xyqs = XYQS[i];

        /* First compute the total intensity S */
        double A = abcd.A;
        double B = abcd.B;
        double C = abcd.C;
        double D = abcd.D;

        double S_AC = A + C;
        double S_BD = B + D;
        double S_AB = A + B;
        double S_CD = C + D;
        double S = S_AC + S_BD;

        /* Now compute the positions according to the model.
         *    Start by precomputing 1/S. */
        double InvS_AC = Reciprocal(S_AC * 2.0);
        double InvS_BD = Reciprocal(S_BD * 2.0);
        double InvS_AB = Reciprocal(S_AB * 2.0);
        double InvS_CD = Reciprocal(S_CD * 2.0);

        /* KX and XY should be divided by 2 in partial difference-over-sum. Shift partial sums by 1 bit to implement it. */
        double partial_AC_pos_x = DeltaToPosition(A - C, InvS_AC);
        double partial_BD_pos_x = DeltaToPosition(B - D, InvS_BD);

        double partial_AC_pos_y = DeltaToPosition(A - C, InvS_AC);
        double partial_BD_pos_y = DeltaToPosition(B - D, InvS_BD);

        double partial_AB_pos_q = DeltaToPosition(A - B, InvS_AB);
        double partial_CD_pos_q = DeltaToPosition(C - D, InvS_CD);

        /* Compute X and Y according to the currently selected detector
         * orientation. */
        if (Diagonal)
        {
            xyqs.X = partial_AC_pos_x - partial_BD_pos_x;
            xyqs.Y = partial_AC_pos_y + partial_BD_pos_y;
        }
        else
        {
            xyqs.X = -partial_BD_pos_x;
            xyqs.Y = partial_AC_pos_y;
        }

        xyqs.Q = partial_AB_pos_q + partial_CD_pos_q;
        xyqs.S = S;
    }
}
