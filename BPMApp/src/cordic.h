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


/* Interface to fast cordic cartesian to polar conversion. */


/* Computes sqrt(x*x+y*y) using purely integer arithmetic.  The result
 * returned is scaled by a factor of approximately 0.5822 times the true
 * magnitude.  For our application this factor can be ignored. */
int CordicMagnitude(int x, int y);

/* To convert the magnitude returned by CordicMagnitude() into the correct
 * units compute
 *
 *  true_magnitude = MulUU(CordicMagnitude(int x, int y) << 1, CORDIC_SCALE)
 *
 * In practice this will overflow, so the left shift should be omitted for
 * an unsigned result.  If a signed result without overflow is needed then
 * CORDIC_SCALE must be halved before use and MulSS can be used.
 *
 * This constant is computed (in Python) as
 *
 *  CORDIC_SCALE = int(round(2**32 /
 *      reduce(operator.mul, [sqrt(1 + 2**-(2*n)) for n in range(1, 13)])))
 *
 * See cordic.cpp for more details of this computation. */
#define CORDIC_SCALE  3688454971U
