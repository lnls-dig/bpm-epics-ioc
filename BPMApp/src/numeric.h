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

/* High efficiency numeric support routines:
 *
 *      CLZ             Counts leading zeros.
 *      MulUU           Scaled multiplication routines
 *      MulSS                   "
 *      MulUS                   "
 *      Reciprocal      Computes 1/x
 *      nmTOmm          Multiplies by 1e-6 and converts to float.
 */

/* Macro derived from the kernel to tell the compiler that x is quite
 * unlikely to be true. */
#define unlikely(x)   __builtin_expect((x), 0)


/* Arch types */
#if (defined (__ARM_ARCH_5__) || defined(__ARM_ARCH_5E__) || defined(__ARM_ARCH_5T__) || \
    defined(__ARM_ARCH_5TE__) || defined(__ARM_ARCH_5TEJ__))
#define __ARM_ARCH__
#else
#define __ARM_OTHER__
#endif

/* Returns the number of leading zeros in an integer.  On the v5 ARM this
 * generates a single clz instruction. */
#define CLZ(x)  ((unsigned int) __builtin_clz(x))

/* Implementation of CLZ for 64 bit integers. */
inline unsigned int clz_64(uint64_t x)
{
    if (x >> 32)
        return CLZ(x >> 32);
    else
        return 32 + CLZ((uint32_t) (x & 0xFFFFFFFF));
}




/* Returns 2^-32 * x * y, signed or unsigned.  This is particularly convenient
 * for fixed point arithmetic, and is remarkably inexpensive (approximately
 * 30ns).
 *    We do these using inline assembler because the compiler is too
 * dim-witted to do this right otherwise (gcc 3.4.5 does the whole 64 bit
 * right shift).  Note the "=&r" modifiers: the true constraint on -mull is
 * that the first three registers be distinct, but the only practical way to
 * achieve this seems to be to mark the two (already separate) outputs as
 * "early clobber" to force them to be distinct from both inputs. */
inline unsigned int MulUU(unsigned int x, unsigned int y)
{
#ifdef __ARM_ARCH__
    unsigned int result, temp;
    __asm__("umull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y));
    return result;
#else
    /* FIXME. We are only getting the 32 MSB bits, which is ok for small
     * amplitudes, but not for real and big amplitudes */
    return (unsigned int)((((uint64_t)x*(uint64_t)y) & 0xFFFFFFFF00000000) >> 32);
#endif
}

inline int MulSS(int x, int y)
{
#ifdef __ARM_ARCH__
    unsigned int result, temp;
    __asm__("smull   %1, %0, %2, %3" :
        "=&r"(result), "=&r"(temp) : "r"(x), "r"(y));
    return result;
#else
    /* FIXME. We are only getting the 32 MSB bits, which is ok for small
     * amplitudes, but not for real and big amplitudes */
    return (int)((((int64_t)x*(int64_t)y) & 0xFFFFFFFF00000000) >> 32);
#endif
}


/* To retain the maximum possible number of bits we have to take a bit of
 * care when multiplying a signed by an unsigned integer.  This routine works
 * by writing the signed part as
 *      y = y0 - s*2^31
 * where s is the sign bit and y0 the bottom 31 bits of y.
 *
 * As an unsigned number, this same y is treated as
 *      y_ = y0 + s*2^31 ,
 * so when we calculcate x*y_ (for unsigned x) we get
 *      x*y_ = x*y0 + s*x*2^31 = x*y + s*x*2^32
 * and so we can use unsigned multiplication to compute
 *      x*y = x*y_ - s*x*2^32 .
 *
 * If it is known that x < 2^31 (and so cannot be mistaken for a signed
 * value) then it will be faster to use MulSS instead. */
inline int MulUS(unsigned int x, int y)
{
    int result = (int) MulUU(x, y);
    if (y < 0)
        result -= x;
    return result;
}


/* The following routines are a kind of "poor man's floating point": we
 * perform long multiplication without loss of precision, while maintaining
 * the residual offset as a separately returned result. */

/* Returns 2^s * x * y and s, where s+32 is the maximum shift that can be
 * applied to both arguments to ensure that as few significant bits as
 * possible are lost.  The value for s lies in the range +32 to -32.
 *     The shift is accumulated to assist with extended expressions. */
inline unsigned int MulUUshift(unsigned int x, unsigned int y, int &shift)
{
    int sx = CLZ(x);
    int sy = CLZ(y);
    shift += sx + sy - 32;
    return MulUU(x << sx, y << sy);
}

/* The same game with signed arguments is somewhat more difficult, so we pass
 * for now. */
// inline int MulSSshift(int x, int y, int &shift)


/* Takes into account an existing shift and applies it to the argument,
 * taking care to allow for overflow.   Returns 2^-s * X, but takes overflow
 * into account. */
unsigned int Denormalise(unsigned int x, int shift);


/* Returns (Y, s) such that:
 *
 *               s
 *  1.  X * Y = 2    (to approximately 31 bits of precision)
 *
 *       31         32
 *  2.  2   <= Y < 2   .
 *
 * From this the following further facts follow:
 *
 *           s
 *      Y = 2  / X
 *
 *       s-32         s-31
 *      2     < X <= 2
 *
 *      32 <= s <= 63.
 *
 * If X is zero then Y=0, s=31 is returned: this is an accident of
 * implementation of little significance, it's up to the caller not to pass
 * X=0.
 *
 * The value assigned to shift is accumulated by s to assist with extended
 * expressions. */
unsigned int Reciprocal(unsigned int X, int &shift);



/* Fast fixed point computation of logarithm base 2 with about 20 bits of
 * precision using pure integer arithmetic.  The input has 16 bits after the
 * binary point and the output 27 bits, so we can say:
 *
 *                          16               27
 *      Takes argument X = 2  x and returns 2  log x.
 *                                                2
 *
 * Alternatively, we can write the relationship between this function (log2)
 * and the true base 2 logarithm (log_2) as
 *
 *                 27      -16      27
 *      log2(X) = 2  log (2   X) = 2  (log X - 16)
 *                      2                 2
 * or
 *               -27
 *      log X = 2   log2(X) + 16.
 *         2
 *
 * Output is clipped in response to extreme inputs. */
int log2(unsigned int X);

/* Fast fixed point computation of exponential 2^X with about 20 bits of
 * precision using pure integer arithmetic.  The input has 27 bits after the
 * binary point and the output 16 bits:
 *
 *                          27               s 16 x   s+16+x
 *      Takes argument X = 2  x and returns 2 2 2  = 2      , where s is
 *      returned as the argument shift.  Thus:
 *
 *                       -27         -27
 *                 s 16 2   X   s + 2   X + 16
 *      exp2(X) = 2 2  2     = 2
 *
 * or
 *       X    -s      27           -s-16      27
 *      2  = 2  exp2(2  (X-16)) = 2     exp2(2  X)
 *
 * This is the inverse of the log2 function, and the format of arguments
 * matches precisely, except for the shift which is separated out here. */
unsigned int exp2(int X, int &shift);

/* Returns 1e6 * 20 * log_10(X): computes the dB value corresponding to X. */
int to_dB(unsigned int X);

/* Converts a dB value into the corresponding exponential value.
 * Returns 2^s * 10^(X/(20 * 1e6)) together with s as shift.
 *    The range of possible input values is approximately +-67e6.  Note that
 * there is an offset of 16 on the shift, so the range of possible shift
 * values is 16 to 48. */
unsigned int from_dB(int X, int &shift);


/* Computes simultaneous cos and sin of the given angle.  The angle is a
 * 32-bit fraction of a complete rotation, so to compute angle_in from an
 * angle in radians compute
 *
 *                  32   angle
 *      angle_in = 2   * -----
 *                       2 pi
 *
 * The computed values are scaled by 2^30, so that the signed result fits in
 * 32 bits without overflow. */
void cos_sin(int angle_in, int &c_out, int &s_out);



/*****************************************************************************/
/*                                                                           */
/*                       Poor Man's Floating Point Class                     */
/*                                                                           */
/*****************************************************************************/

/* The following class is an exercise in C++ cuteness.  I am by no means
 * convinced that it is a good idea...
 *
 * The idea here is to package up some of the "poor man's floating point"
 * routines above into something a bit more "friendly".  However, this
 * approach hides pitfalls, and is a very long winded way of expressing
 * something really quite simple.
 *
 * Never mind, here it is. */

class PMFP
{
public:
    /* Here we implement a fairly classical style operator oriented interface
     * to make a PMFP look like an ordinary value. */

    /* Standard constructors and assignment.  These are all inlined in the
     * hope that the compiler will be sensible. */
    inline PMFP(unsigned int InitialValue = 0, int InitialShift = 0)
    {
        Value = InitialValue;
        Shift = InitialShift;
    }

    inline PMFP(const PMFP &Copy)
    {
        Value = Copy.Value;
        Shift = Copy.Shift;
    }

    inline void operator=(const PMFP &Copy)
    {
        Value = Copy.Value;
        Shift = Copy.Shift;
    }


    /* Extracting the underlying value.  We don't provide a casting operator,
     * as implicit denormalising is actually rather a bad idea! */
    inline unsigned int Denormalise() const
    {
        return ::Denormalise(Value, Shift);
    }

    /* Actually doing work. */
    inline PMFP operator*(const PMFP &Multiplier) const
    {
        int NewShift = Shift + Multiplier.Shift;
        unsigned int NewValue = MulUUshift(Value, Multiplier.Value, NewShift);
        return PMFP(NewValue, NewShift);
    }

    inline PMFP operator/(const PMFP &Divisor) const
    {
        int NewShift = Shift - Divisor.Shift;
        unsigned int NewValue = MulUUshift(
            Value, ::Reciprocal(Divisor.Value, NewShift), NewShift);
        return PMFP(NewValue, NewShift);
    }

    inline PMFP Reciprocal() const
    {
        int NewShift = - Shift;
        unsigned int NewValue = ::Reciprocal(Value, NewShift);
        return PMFP(NewValue, NewShift);
    }


    /* This one is rather tricky: here we construct a PMFP instance from any
     * function which returns a value and a shift (for example, exp2). */
    template<class T> inline PMFP(
        unsigned int (*f)(T,int&), T Argument, int InitialShift = 0)
    {
        Shift = InitialShift;
        Value = f(Argument, Shift);
    }


private:
    /* The underlying value of a PMFP is 2^Shift * Value, so the shift will
     * be removed by the Denormalise operation when all calculations are
     * complete. */
    unsigned int Value;
    int Shift;
};

/* Conversions of methods into operations. */
inline PMFP Reciprocal(const PMFP &Argument)
{
    return Argument.Reciprocal();
}

inline unsigned int Denormalise(const PMFP &Argument)
{
    return Argument.Denormalise();
}
