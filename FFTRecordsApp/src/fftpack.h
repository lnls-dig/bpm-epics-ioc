#include <math.h>
#include <stdio.h>
#define DOUBLE
/*#undef DOUBLE*/

#ifdef DOUBLE
#define Treal double
#else
#define Treal float
#endif

/* function prototypes */
void cffti(int n, Treal wsave[]);
void cfftf(int n, Treal c[], Treal wsave[]);
void cfftb(int n, Treal c[], Treal wsave[]);

void rffti(int n, Treal wsave[]);
void rfftf(int n, Treal r[], Treal wsave[]);
void rfftb(int n, Treal r[], Treal wsave[]);


