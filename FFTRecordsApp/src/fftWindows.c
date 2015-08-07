#include <math.h>

static double SQUARE(long i, long N){
  return 1.0;
}

static double  Hanning(long i, long N){
  /*      Hanning Window function. */
  /*      See sec. 12.7 in "Numerical Recipes in C" 1st ed. */
  double dn=N, x=i, w;
  if (i < 0 || i > N){
    return 0.0;
  }
  else{
    w=0.5*(1.0-cos(2.0*M_PI*x/dn));
    return w;
  }
}

static double  Parzen(long i, long N){
  /*      Parzen Window function. */
  /*      See sec. 12.7 in "Numerical Recipes in C" 1st ed. */
  double dn=N, x=i, w;
  w=1.0-abs(2.0*(x -(dn-1)/2.0)/(dn+1));
  return w;
}

static double  Welch(long i, long N){
  /*      Welch Window function. */
  /*      See sec. 12.7 in "Numerical Recipes in C" 1st ed. */
  double dn=N, x=i, w;

  w=1.0-(2.0*(x-(dn-1)/2.0)/(dn+1));
  w=w*w;
  return w;
}

static double  QuadW(long i, long N){
  double dn=N, x=i, w;

  w=x*(x - dn)/(dn*dn/4.0);
  w=w*w;
  return w;
}

static void filter( double *a, double av, long n, long wind){
  register long i;
  double (*func)(long , long);

  switch( wind){
  case WFM_Square:
    for(i=0; i<n; i++){
      a[i] -=av;
    }
    return;
  case WFM_Hanning:
    func=Hanning;
    break;
  case WFM_Parzen:
    func=Parzen;
    break;
  case WFM_Welch:
    func=Welch;
    break;
  case WFM_QuadW:
    func=QuadW;
    break;
  default:
    func=SQUARE;
  }
  for(i=0; i<n; i++){
    a[i] -=av;
    a[i] *=func(i,n);
  }
  return;
}

static void cfilter( double *a, long n, long wind){
  long i;
  double (*func)(long , long);
  register double f;

  switch( wind){
  case WFM_Square:
    return;
  case WFM_Hanning:
    func=Hanning;
    break;
  case WFM_Parzen:
    func=Parzen;
    break;
  case WFM_Welch:
    func=Welch;
    break;
  case WFM_QuadW:
    func=QuadW;
    break;
  default:
    func=SQUARE;
  }
  for(i=0; i<n; i++){
    f=func(i,n);
    a[2*i] *=f;
    a[2*i+1] *=f;
  }
  return;
}
