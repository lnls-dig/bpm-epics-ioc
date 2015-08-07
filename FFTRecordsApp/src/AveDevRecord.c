/* AveDevRecord.c */
/* AveDevRecord.c - Record Support Routines for Real FFT record 
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

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbEvent.h>
#include	<dbFldTypes.h>
#include	<dbScan.h>
#include	<devSup.h>
#include	<errMdef.h>
#include <errlog.h>
#include	<recSup.h>
#include	<recGbl.h>
#define GEN_SIZE_OFFSET
#include	<AveDevRecord.h>
#undef  GEN_SIZE_OFFSET
#include "epicsExport.h"

#define RECORDTYPE AveDevRecord

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

struct rset AveDevRSET={
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

epicsExportAddress(rset,AveDevRSET);

/*sizes of field types*/
#include "menuFtype.h"

static void monitor();
static long readValue();
static long avedev_with_cut(double *, size_t *,
			    double *, double *, 
			    double , double);

static long init_record(struct RECORDTYPE *precord, int pass)
{
  RECORDTYPE	*self = (RECORDTYPE *)precord;
  long	status;
  DBLINK *plink;

  if (pass == 0){
    if(self->nelm <= 0) self->nelm=1;
    if(self->nelm == 1) {
      self->nord = 1;
    } else {
      self->nord = 0;
    }
    self->bptr = (double *)calloc(self->nelm, sizeof(double));
    if (self->bptr == NULL ){
      self->pact=TRUE;
      /* errlogPrintf("aRaw:(array select) cannot allocate buffer area.");*/
      self->nord = 0;
      if (self->bptr != NULL) free(self->bptr);
      return (-1);
    }
    return(0);
  }

  /* AveDev Record  is a pure soft record */
  plink = &(self->inp);
  if(plink->type == CONSTANT){
    self->bptr = (double *) calloc(1,sizeof(double));
    self->nord = 1;
    recGblInitConstantLink(plink, DBF_DOUBLE, self->bptr);
  }
  else{
    status=dbGetNelements(plink, (signed long *)&self->nord);
    if (self->nord> self->nelm){
      self->nord = self->nelm;
    }
  }
    return(0);
}

static long process(self)
	struct RECORDTYPE	*self;
{
	long		 status;
        unsigned char  	 pact=self->pact;
	size_t nelement;

        if ( pact ) return(0); 
	status=readValue(self); /* read the new value */
	/*errlogPrintf("AveDevRec: nelm=%lu ,nord=%lu\n",self->nelm, self->nord); */
        self->pact = TRUE;

        self->udf = TRUE;

	nelement=self->nord;
	status=avedev_with_cut(self->bptr, &nelement,
			       &self->ave0, &self->dev0,
			       0.0, 0.0);
	if (status < 0){
	  recGblSetSevr(self,UDF_ALARM,INVALID_ALARM);
	}
	else if (status ==1){
	    self->val=self->ave0;
	    self->dev=0.0;
	    recGblSetSevr(self,CALC_ALARM,MAJOR_ALARM);
	}
	else{
	  if(self->cut > 0){
	    nelement=self->nord;
	    status=avedev_with_cut(self->bptr, &nelement, 
				   &self->val, &self->dev,
				   self->ave0, self->cut*self->dev0);
	  }
	  else{
	    self->val=self->ave0;
	    self->dev=self->dev0;
	  }
	  if (status < 0){
	    recGblSetSevr(self,UDF_ALARM,INVALID_ALARM);
	  }
	  else if (status ==1){
	    recGblSetSevr(self,CALC_ALARM,MAJOR_ALARM);
	  }
	}
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

    if(paddr->pfield == &(self->ival)){
      paddr->pfield = self->bptr;
      paddr->no_elements = self->nord;
      paddr->field_type = paddr->dbr_field_type = DBF_DOUBLE;
      paddr->field_size = sizeof(double);
      return(0);
    }
    /* do nothing */
    return(0);
}

static long get_array_info(paddr,no_elements,offset)
    struct dbAddr *paddr;
    long	  *no_elements;
    long	  *offset;
{
    struct RECORDTYPE	*self=(struct RECORDTYPE *)paddr->precord;

    *no_elements = self->nelm;
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
    if(*precision < 1)     *precision = 1;
    if(paddr->pfield == (void *) self->bptr) return (0);
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

    if((paddr->pfield==(void *)self->bptr) || (paddr->pfield== &(self->ival))
       ||(paddr->pfield==(void *) &self->val)){
        pcd->upper_ctrl_limit = self->hopr;
        pcd->lower_ctrl_limit = self->lopr;
    } 
    else {
      recGblGetControlDouble(paddr,pcd);
    }
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
	  db_post_events(self, &self->val, monitor_mask);
	  db_post_events(self, &self->dev, monitor_mask);
	}
	return;
}

static long readValue(struct RECORDTYPE *self)
{
  long            status=0;
  long nRequest;

  nRequest=self->nelm;
  status = dbGetLink(&(self->inp), DBR_DOUBLE, self->bptr, 0, &nRequest);
  /* errlogPrintf("AveDevRec: data read %lu\n",nRequest);*/
  if(nRequest>0) self->nord=nRequest;

  return(status);
}

static long avedev_with_cut(double *buf, size_t *nelm,
			    double *ave, double *dev, 
			    double offset, double cut)
{
  long i,cnt;
  double sum, res,d;

  if (*nelm < 0){
    return -1;
  }
  else if (*nelm == 1){
    *ave=buf[0];
    *dev=0.0;
    return 1;
  }
  
  sum=0.0; res=0.0; cnt=0;
  for(i=0;i<*nelm;i++){
    d=buf[i]-offset;
    if ((cut == 0.0) || (fabs(d) <= cut)){
      sum += d;
      res += d*d;
      cnt ++;
    }
  }
  *nelm=cnt;
  if (cnt <= 0){
    return -1;
  }
  else if (cnt == 1){
    *ave=sum+offset;
    res = res - sum*sum/cnt;
    *dev = sqrt(res);
    return 1;
  }
  else{
    *ave=sum/cnt+offset;
    res = res - sum*sum/cnt;
    *dev = sqrt(res/((double)cnt -1));
  }
  return 0;
}

