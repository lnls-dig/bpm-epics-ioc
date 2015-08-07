/* CFFTRecord.c */
/* CFFTRecord.c - Record Support Routines for Real FFT record 
 *
 *      
 *      Author:         Noboru Yamamoto @KEK.JAPAN
 *      Date:           Jan. 5, 1998
 *                      based on subArrayRecord.c by
 *                      Author:         Carl Lionberger
 *                      Date:           090293
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 *		The Control Systems Group
 *		Systems Engineering Department
 *		Lawrence Berkeley Laboratory
 *
 *      NOTES:
 * Derived from subArray Record
 * Modification Log:
 * -----------------
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <dbEvent.h>
#include <dbFldTypes.h>
#include <dbScan.h>
#include <devSup.h>
#include <errMdef.h>
#include <errlog.h>
#include <recSup.h>
#include <recGbl.h>
#define GEN_SIZE_OFFSET
#include <CFFTRecord.h>
#undef  GEN_SIZE_OFFSET
#include "epicsExport.h"

#define RECORDTYPE CFFTRecord


/* Create RSET - Record Support Entry Table*/
#define report NULL
#define initialize NULL
static long init_record();
static long process();
#define special NULL
#define get_value NULL
static long cvt_dbaddr();
static long get_array_info();
static long put_array_info();
static long get_units();
static long get_precision();
#define get_enum_str NULL
#define get_enum_strs NULL
#define put_enum_str NULL
static long get_graphic_double();
static long get_control_double();
#define get_alarm_double NULL

struct rset CFFTRSET={
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double };

epicsExportAddress(rset,CFFTRSET);

/*sizes of field types*/
#include "menuFtype.h"

static void monitor();
static long readValue();

/* fftpack routines */
#include "fftpack.h"
#include "fftWindows.c"

static long alloc_buffer(struct RECORDTYPE *precord)
{
  RECORDTYPE	*self = (RECORDTYPE *)precord;
  size_t nelm=self->nelm;

  self->bptr = (double *)calloc(2*nelm, sizeof(double));
  
  self->iptr = (double *)calloc(nelm, sizeof(double));
  self->qptr = (double *)calloc(nelm, sizeof(double));
  self->bamp = (double *)calloc(nelm, sizeof(double));
  self->bpha = (double *)calloc(nelm, sizeof(double));
  self->bwvn = (double *)calloc(nelm, sizeof(double));
  
  self->wsav = (double *)calloc(6*nelm+15, sizeof(double));
  
  if (self->bptr == NULL ||
      self->iptr == NULL ||
      self->qptr == NULL ||
      self->bamp == NULL ||
      self->bpha == NULL ||
      self->bwvn == NULL ||
      self->wsav == NULL ){
    self->pact=TRUE;
    /* errlogPrintf("aRaw:(array select) cannot allocate buffer area.");*/
    self->nord = 0;
    if (self->bptr != NULL) free(self->bptr);
    if (self->iptr != NULL) free(self->iptr);
    if (self->qptr != NULL) free(self->qptr);
    if (self->bamp != NULL) free(self->bamp);
    if (self->bpha != NULL) free(self->bpha);
    if (self->bwvn != NULL) free(self->bwvn);
    if (self->wsav != NULL) free(self->wsav);
    return (-1);
  }
  return 0;
}
static long init_record(struct RECORDTYPE *precord, int pass)
{
  RECORDTYPE	*self = (RECORDTYPE *)precord;
  long	status;
  DBLINK *plink;

  if (pass == 0){
    if(self->nelm <= 0) self->nelm=1;
    if((self->span <= 0) || ( self->span > self->nelm)){
      self->span=self->nelm;
    }

    if(self->nelm == 1) {
      self->nord = 1;
    } else {
      self->nord = 0;
    }
    self->mxix=self->nelm/self->span-1;
    /* buffers should be allocated in pass 0 */
    return alloc_buffer(self);
  }

  /* CFFT Record  is a pure soft record */
  plink = &(self->inp);
  if(plink->type == CONSTANT){
    self->nord = 1;
    recGblInitConstantLink(plink, DBF_DOUBLE, self->bptr);
  }
  else{
    status=dbGetNelements(plink, &self->nord);
    if (self->nord > self->nelm){
      self->nord=self->nelm;
    }
    if((self->span <= 0) || ( self->span > self->nelm)){
      self->span=self->nelm;
    }
    /*errlogPrintf("CFFTRec: nelm=%lu ,nord=%lu",self->nelm, self->nord);*/
    /* initialize  fft routine */
    cffti(self->span, (double *) &(self->wsav[self->span]));
  }
    return(0);
}

static long process(self)
	struct RECORDTYPE	*self;
{
	long		 status;
        unsigned char  	 pact=self->pact;
	long r,i;
	double *rptr, norm, iav,qav;
	double complex *cptr;

        if ( pact ) return(0); 
	status=readValue(self); /* read the new value */
	errlogPrintf("CFFTRec: nelm=%lu ,nord=%lu\n",self->nelm, self->nord); 
        self->pact = TRUE;

        self->udf = TRUE;
	norm=sqrt(self->span);
	cffti((int) self->span, (double *) &(self->wsav[2*self->span]));

	rptr=self->wsav;
	cptr=(double complex *) self->wsav;
	for (r = 0;
	     r <= (self->nelm - self->span); 
	     r += self->span ){
	  errlogPrintf("CFFTRec: data from %ld ,to  %ld\n",r,r+self->span-1);
	  iav=0.0; qav=0.0;
	  if(self->asub == FAS_AverageSubtraction){
	    for (i = 0; i< self->span; i++){
	      iav +=self->iptr[r+i];
	      qav +=self->qptr[r+i];
	    }
	    iav /=(double) self->span;
	    qav /=(double) self->span;
	  }
	  else if(self->asub == FAS_LinearSubtraction){
	    errlogPrintf("CFFT records: Linear Subtraction is not supported yes.");

	  }
	  for (i = 0; i< self->span; i++){
	    rptr[2*i]=self->iptr[r+i]-iav;
	    rptr[2*i+1]=self->qptr[r+i]-qav;
	  }
	  cfilter(rptr, self->span, self->wind);
	  if(self->cdir == FCD_Backward){
	    cfftb((int) self->span, rptr, &self->wsav[2*self->span]);
	  }
	  else{
	    cfftf((int) self->span, rptr, &self->wsav[2*self->span]);
	  }
	  for (i=0; i<self->span;i++){
	    double c,s;
	    c=rptr[2*i];
	    s=rptr[2*i+1];
	    self->bamp[r+i]=sqrt( c*c + s*s );
	    self->bpha[r+i]=atan2(s,c);
	    self->bwvn[r+i]=((double) i)/((double) self->span);
	  }
	}
	self->mxix=self->nord/self->span-1;
        self->udf = FALSE;

        recGblGetTimeStamp(self);
	monitor(self);

        /* process the forward scan link record */
        recGblFwdLink(self);

        self->pact = FALSE;
        return(0);
}

static long cvt_dbaddr(paddr)
    struct dbAddr *paddr;
{
    struct RECORDTYPE *self=(struct RECORDTYPE *)paddr->precord;
    long sz,indx;

    sz=self->span;
    indx=self->indx;
    if (indx > self->mxix){
      indx=self->indx=self->mxix;
    }
    if(paddr->pfield == &(self->val)){
      paddr->pfield = &self->bptr[0];
      paddr->no_elements = 2*self->nord;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    else if(paddr->pfield == &(self->sigi)){
      paddr->pfield = &self->iptr[0];
      paddr->no_elements = self->nord;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    else if(paddr->pfield == &(self->sigq)){
      paddr->pfield = &self->qptr[0];
      paddr->no_elements = self->nord;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    else if(paddr->pfield == &(self->amp)){
      paddr->pfield = &self->bamp[0];
      paddr->no_elements = sz+sz*self->mxix;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    else if(paddr->pfield == &(self->pha)){
      paddr->pfield = &self->bpha[0];
      paddr->no_elements = sz+sz*self->mxix;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    else if(paddr->pfield == &(self->wavn)){
      paddr->pfield = &self->bwvn[0];
      paddr->no_elements = sz+sz*self->mxix;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    return(0);
}

static long get_array_info(paddr,no_elements,offset)
    struct dbAddr *paddr;
    long	  *no_elements;
    long	  *offset;
{
    struct RECORDTYPE	*self=(struct RECORDTYPE *)paddr->precord;

    *no_elements = 2*self->nelm;
    *offset = 0;
    return(0);
}

static long put_array_info(paddr,nNew)
    struct dbAddr *paddr;
    long	  nNew;
{
    struct RECORDTYPE	*self=(struct RECORDTYPE *)paddr->precord;

    if(nNew > self->nelm)
      self->nord = self->nelm;
    else
      self->nord = nNew;
    return(0);
}

static long get_units( struct dbAddr *paddr, char *units)
{
    struct RECORDTYPE *self=(struct RECORDTYPE *)paddr->precord;

    strncpy(units,self->egu,DB_UNITS_SIZE);
    return(0);
}

static long get_precision(struct dbAddr *paddr, long *precision)
{
    struct RECORDTYPE	*self =(struct RECORDTYPE *)paddr->precord;


    *precision = self->prec;
    if(*precision <1)     *precision = 1;
    if(paddr->pfield == (void *) self->bptr) return (0);
    if(paddr->pfield == (void *) self->iptr) return (0);
    if(paddr->pfield == (void *) self->qptr) return (0);
    if(paddr->pfield == (void *) self->bamp) return (0);
    if(paddr->pfield == (void *) self->bpha) return (0);
    if(paddr->pfield == (void *) self->bwvn) return (0);
    recGblGetPrec(paddr,precision);
    return(0);
}

static long get_graphic_double(struct dbAddr *paddr,struct dbr_grDouble *pgd)
{
    struct RECORDTYPE     *self=(struct RECORDTYPE *)paddr->precord;

    if((paddr->pfield==(void *)self->bptr) || (paddr->pfield== &(self->val))){
        pgd->upper_disp_limit = self->hopr;
        pgd->lower_disp_limit = self->lopr;
    } else 
      recGblGetGraphicDouble(paddr,pgd);
    return(0);
}
static long get_control_double(paddr,pcd)
    struct dbAddr *paddr;
    struct dbr_ctrlDouble *pcd;
{
    struct RECORDTYPE     *self=(struct RECORDTYPE *)paddr->precord;

    if((paddr->pfield==(void *)self->bptr) || (paddr->pfield== &(self->val))){
        pcd->upper_ctrl_limit = self->hopr;
        pcd->lower_ctrl_limit = self->lopr;
    } else recGblGetControlDouble(paddr,pcd);
    return(0);
}

static void monitor(self)
    struct RECORDTYPE	*self;
{
	unsigned short	monitor_mask;

        /* get previous stat and sevr  and new stat and sevr*/
        monitor_mask = recGblResetAlarms(self);

	monitor_mask |= (DBE_LOG|DBE_VALUE);
        if(monitor_mask){
	  db_post_events(self, self->bptr, monitor_mask);
	  db_post_events(self, &self->bamp[0], monitor_mask);
	  db_post_events(self, &self->bpha[0], monitor_mask);
	  db_post_events(self, &self->iptr[0], monitor_mask);
	  db_post_events(self, &self->qptr[0], monitor_mask);
	  db_post_events(self, &self->bwvn[0], monitor_mask);
	}
	return;
}

static long readValue(struct RECORDTYPE *self)
{
  long status=0;
  long nRequest,i ;

  if ( (self->span) <= 0) self->span=self->nelm;
  if(self->cdft == CDF_Default){
    nRequest=self->nelm;
    status = dbGetLink(&(self->inp), DBR_DOUBLE, self->iptr, 0, &nRequest);
    errlogPrintf("CFFTRec: I data read %lu\n",nRequest);
    if (status != 0){
      return -1;
    }
    if(nRequest>0) self->nord=nRequest;
    nRequest=self->nelm;
    status = dbGetLink(&(self->inpq), DBR_DOUBLE, self->qptr, 0, &nRequest);
    errlogPrintf("CFFTRec: Q data read %lu\n",nRequest);
    if (status != 0){
      return -1;
    }
    if(nRequest>0) self->nord=nRequest;
    for (i = 0; i<self->nord; ++i){
      self->bptr[2*i]=self->iptr[i];
      self->bptr[2*i+1]=self->qptr[i];
    }
    errlogPrintf("CFFTRec: copied data %lf %lf %lf %lf\n", self->bptr[2*i],self->iptr[i],
		 self->bptr[2*i],self->iptr[i]);
  }
  else{
    nRequest=2*self->nelm;
    status = dbGetLink(&(self->inp), DBR_DOUBLE, self->bptr, 0, &nRequest);
    if(nRequest>0) self->nord=nRequest/2;
    if(self->cdft == CDF_Interleved){
      for (i = 0;i<self->nord; ++i){
	self->iptr[i]=self->bptr[2*i];
	self->qptr[i]=self->bptr[2*i+1];
      }
    }
    else if(self->cdft == CDF_Sequential){
      errlogPrintf("CFFTRec: Sequentil data format  %d\n",self->cdft);
      for (i = 0;i<self->nord;++i){
	self->iptr[i]=self->bptr[ i             ];
	self->qptr[i]=self->bptr[ i + self->nord];
      }
    }
    else{
      errlogPrintf("CFFTRec: Invalid data format  %d\n",self->cdft);
    }
    errlogPrintf("CFFTRec: data read %lu\n",nRequest);
  }
  return(status);
}

