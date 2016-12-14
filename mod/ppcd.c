/*-
 * Copyright (c) 2005, Geocrime
 * Portions Copyright (c) 1997, 1998, 1999 Nicolas Souchu
 * Portions Copyright (c) 1996, 1997, 1998 Grant R. Guenther <grant@torque.net>
 * Portions Copyright (c) 2001, 2002 Thomas Quinot <thomas@cuivre.fr.eu.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/devicestat.h>	/* for struct devstat */

#include <machine/clock.h>

/* CAM stuff */
#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_sim.h>
#include <cam/cam_xpt_sim.h>
#include <cam/cam_debug.h>
#include <cam/cam_periph.h>

#include <cam/scsi/scsi_all.h>
#include <cam/scsi/scsi_message.h>
#include <cam/scsi/scsi_da.h>

#if __FreeBSD_version >= 500000
#include <sys/taskqueue.h>
#include <machine/bus.h>
#endif

/* ATA stuff */
#include <dev/ata/ata-all.h>

#if __FreeBSD_version >= 500000
#include <dev/ata/ata-commands.h>
#define ATA_C_ATAPI_RESET ATA_ATAPI_RESET
#define ATA_C_PACKET_CMD ATA_PACKET_CMD
#endif

#include <sys/kernel.h>

#include "ppcd.h"
#include "frpw.h"

/* ppbus stuff */
#include <dev/ppbus/ppbio.h>
#include <dev/ppbus/ppbconf.h>
#include <dev/ppc/ppcreg.h>

#include "ppbus_if.h"



/* CAM related functions */
static void	ppcd_action(struct cam_sim *sim, union ccb *ccb);
static void	ppcd_poll(struct cam_sim *sim);
static void	ppcd_cam_rescan_callback(struct cam_periph *periph,
					union ccb *ccb);
static void	ppcd_cam_rescan(struct ppcd_softc *sc);
#define stailq_next	spriv_ptr0

/* ATA layer emulation */
static void ppcd_intr(void *psc);

#define PCD_TMO		800	/* timeout in ticks */
#define PCD_DELAY	50	/* spin delay in us */
#define PCD_RESET_TMO	100	/* in tenth of a second */
#define PCD_SPIN	(2000000*PCD_TMO)/(hz*PCD_DELAY)

#define PPCD_TIMEOUT	0x100	/* 'superset' of ATA error codes */

#define WR(c,r,v)       frpw_write_regr(sc,c,r,v)
#define RR(c,r)         (frpw_read_regr(sc,c,r))


static int
p_ata_wait(struct ppcd_softc *sc, int go, int stop, char * msg )
{
	int j, r, p;

	j = 0;
	while ((((r=RR(1,ATA_DRIVE))&go)||(stop&&(!(r&stop))))&&(j++<PCD_SPIN))
		DELAY(PCD_DELAY);

	if ((r&(ATA_S_ERROR&stop))||(j>=PCD_SPIN)) {
	   sc->ppcd_stat = RR(0,ATA_STATUS);
	   sc->ppcd_error = RR(0,ATA_ERROR);
	   p = RR(0,ATA_IREASON);
       	   if (j >= PCD_SPIN) sc->ppcd_error |= PPCD_TIMEOUT;
#ifdef PPCD_VERBOSE
	   if (sc->ppcd_error != 0)
             device_printf(sc->device, "%s: alt=0x%x stat=0x%x"
			   " err=0x%x loop=%d phase=%d\n",
			    msg,r,sc->ppcd_stat,sc->ppcd_error,j,p);
#endif
	   return (sc->ppcd_stat<<8)+r;
	}
	return 0;
}

static int
p_ata_completion( struct ppcd_softc *sc, char * buf )
{	int r, d, p, n, k, j;

	r = -1; k = 0; j = 0;

	if (!p_ata_wait(sc,ATA_S_BUSY,ATA_S_DRQ|ATA_S_READY|ATA_S_ERROR,
						"completion")) {
	    r = 0;
	    while (RR(0,ATA_STATUS)&ATA_S_DRQ) {
	        d = (RR(0,ATA_CYL_LSB)+256*RR(0,ATA_CYL_MSB));
	        n = ((d+3)&0xfffc); /* Make it a divisor of 4 (dword) */
	        p = RR(0,ATA_IREASON)&3;

	        if ((p == 2) && (n > 0) && (j == 0)) {
		    frpw_read_block(sc,buf,n);
		    sc->ppcd_count = n;
#ifdef PPCD_DEBUG
		    device_printf(sc->device, "Read %d bytes\n",n);
#endif
		    r = 0; j++;
	        } else {
#ifdef PPCD_DEBUG
		  device_printf(sc->device, "Unexpected phase %d, d=%d, k=%d\n",
					p,d,k);
#endif
#if (defined PPCD_DEBUG || defined PPCD_VERBOSE)
		    device_printf(sc->device, "WARNING: ATAPI phase errors\n");
#endif
		    DELAY(1);
	        } 
		if (k++ > PCD_TMO) {
			device_printf(sc->device, "Stuck DRQ\n");
			break;
		}
	        if (p_ata_wait(sc,ATA_S_BUSY,ATA_S_DRQ|ATA_S_READY|ATA_S_ERROR,
				"completion")) { 
			r = -1;
			break;
		}
	    }
	}
	
	frpw_disconnect(sc); 
	ppb_release_bus(device_get_parent(sc->device), sc->device);

	return r;
}

static int
p_ata_command( struct ppcd_softc *sc, char * cmd, int dlen )
{	
	ppb_request_bus(device_get_parent(sc->device), sc->device, PPB_WAIT);
	frpw_connect(sc);

        WR(0,ATA_DRIVE,0xa0 + 0x10*sc->drive);

	if (p_ata_wait(sc,ATA_S_BUSY|ATA_S_DRQ,0,"before command")) {
		frpw_disconnect(sc);
		ppb_release_bus(device_get_parent(sc->device), sc->device);
		return -1;
	}

        WR(0,ATA_CYL_LSB,dlen % 256);
        WR(0,ATA_CYL_MSB,dlen / 256);
        WR(0,ATA_CMD,ATA_C_PACKET_CMD);  /* ATAPI packet command */

        if (p_ata_wait(sc,ATA_S_BUSY,ATA_S_DRQ,"command DRQ")) {
		frpw_disconnect(sc);
		ppb_release_bus(device_get_parent(sc->device), sc->device);
		return -1;
	}

        if (RR(0,ATA_IREASON) != 1) {
           device_printf(sc->device, "command phase error\n");
	   frpw_disconnect(sc);
	   ppb_release_bus(device_get_parent(sc->device), sc->device);
           return -1;
        }

	frpw_write_block(sc,cmd,12);

	return 0;
}

static int
p_ata_reset( struct ppcd_softc *sc )
{	int	i, k, flg;
	int	expect[5] = {1,1,1,0x14,0xeb};

	ppb_request_bus(device_get_parent(sc->device), sc->device, PPB_DONTWAIT);
	frpw_connect(sc);
	WR(0,ATA_DRIVE,0xa0 + 0x10*sc->drive);
	WR(0,ATA_CMD,ATA_C_ATAPI_RESET);

	DELAY(2);  		/* delay a bit */

	k = 0;
	while ((k++ < PCD_RESET_TMO) && (RR(1,ATA_DRIVE)&ATA_S_BUSY))
		DELAY(hz/10);

	flg = 1;
	for(i=0;i<5;i++) flg &= (RR(0,i+1) == expect[i]);

#ifdef PPCD_DEBUG
	device_printf(sc->device, "Reset (%d) signature = ",k);
	for (i=0;i<5;i++) printf("%3x",RR(0,i+1));
		if (!flg) printf(" (incorrect)");
		printf("\n");
#endif
	
	frpw_disconnect(sc);
	ppb_release_bus(device_get_parent(sc->device), sc->device);
	return flg-1;	
}

/* We don't use atapi_do_scsi(), since it's less overhead */ 
static int
p_atapi_identify( struct ppcd_softc *sc )
{
	int	i, rc;
	char	id_cmd[12] = {0x12,0,0,0,36,0,0,0,0,0,0,0};
	char	buf[36], *id = &buf[16];

        rc = p_ata_command(sc, id_cmd, 36);
	if (rc) return -1;
	rc = p_ata_completion(sc, buf);
	if (rc) return -1;
	
	if ((buf[0] & 0x1f) != 5) {
#ifdef PPCD_DEBUG
	  device_printf(sc->device, "%s is not a CD-ROM\n",
			sc->drive?"Slave":"Master");
#endif
	  return -1;
	}
	id[16] = '\0';
	for (i = 16; ((i >= 0) && (id[i] <= ' ')); i--) id[i] = '\0';

	device_printf(sc->device, "%s: %s\n",sc->drive?"Slave":"Master",id);
	return 0;
}

/*	returns  0, with id set if drive is detected
	        -1, if drive detection failed
*/
static int
p_atapi_probe( struct ppcd_softc *sc, int ms )
{
	if (ms == -1) {
            for (sc->drive = 0; sc->drive <= 1; sc->drive++)
	       if (!p_ata_reset(sc) && !p_atapi_identify(sc)) 
		  return 0;
	} else {
	    sc->drive = ms;
            if (!p_ata_reset(sc) && !p_atapi_identify(sc)) 
		return 0;
	}
	return -1;
}

/* End of ATA/ATAPI emulation code by Grant R. Guenther. */

static int 
ppcd_atapi_do_scsi(struct ppcd_softc *sc, int host, union ccb *ccb, int sense)
{
int rc = 0;
int len;
char *buf;
struct ccb_hdr *ccb_h = &ccb->ccb_h;
struct ccb_scsiio *csio = &ccb->csio;
struct scsi_request_sense sense_cmd;
static u_int8_t cmd[CAM_MAX_CDBLEN];

/* avoid races */
	if ((ccb_h->status & CAM_STATUS_MASK) != CAM_REQ_INPROG) {
		xpt_done(ccb);
		return 0;
	}
#ifdef PPCD_DEBUG
	device_printf(sc->device, "Target id %d.\n", ccb_h->target_id); 
#endif
	if (/* XXX ccb_h->target_id !=*/ 0) {
		device_printf(sc->device, "Target id %d.\n", ccb_h->target_id); 
		ccb_h->status = CAM_TID_INVALID;
		xpt_done(ccb);
		return 0;
	}

/* Idea comes from /sys/dev/ata/atapi-cam.c by Tomas Quinot */
	bcopy((ccb_h->flags & CAM_CDB_POINTER) ?
	      csio->cdb_io.cdb_ptr : csio->cdb_io.cdb_bytes,
	      cmd, csio->cdb_len);
#ifdef PPCD_DEBUG
	if (CAM_DEBUGGED(ccb_h->path, CAM_DEBUG_CDB)) {
		char cdb_str[(SCSI_MAX_CDBLEN * 3) + 1];

		printf("atapi_action: cmd@%p: %s\n", cmd,
		       scsi_cdb_string(cmd, cdb_str, sizeof(cdb_str)));
	}
#endif

	len = csio->dxfer_len;
	buf = csio->data_ptr;

	/* some SCSI commands require special processing */
	switch (cmd[0]) {
	case INQUIRY: {
	    /*
	     * many ATAPI devices seem to report more than
	     * SHORT_INQUIRY_LENGTH bytes of available INQUIRY
	     * information, but respond with some incorrect condition
	     * when actually asked for it, so we are going to pretend
	     * that only SHORT_INQUIRY_LENGTH are expected, anyway.
	     */
	    struct scsi_inquiry *inq = (struct scsi_inquiry *) &cmd[0];

	    if (inq->byte2 == 0 && inq->page_code == 0 &&
		inq->length > SHORT_INQUIRY_LENGTH) {
		bzero(buf, len);
		len = inq->length = SHORT_INQUIRY_LENGTH;
	    }
	    break;
	}
	case MODE_SELECT_6:
	    /* FALLTHROUGH */

	case MODE_SENSE_6:
	    /*
	     * not supported by ATAPI/MMC devices (per SCSI MMC spec)
	     * translate to _10 equivalent.
	     * (actually we should do this only if we have tried 
	     * MODE_foo_6 and received ILLEGAL_REQUEST or
	     * INVALID COMMAND OPERATION CODE)
	     * alternative fix: behave like a honest CAM transport, 
	     * do not muck with CDB contents, and change scsi_cd to 
	     * always use MODE_SENSE_10 in cdgetmode(), or let scsi_cd
	     * know that this specific unit is an ATAPI/MMC one, 
	     * and in /that case/ use MODE_SENSE_10
	     */

	    CAM_DEBUG(ccb_h->path, CAM_DEBUG_SUBTRACE, 
		      ("Translating %s into _10 equivalent\n",
		      (cmd[0] == MODE_SELECT_6) ?
		      "MODE_SELECT_6" : "MODE_SENSE_6"));
	    cmd[0] |= 0x40;
	    cmd[6] = 0;
	    cmd[7] = 0;
	    cmd[8] = cmd[4];
	    cmd[9] = cmd[5];
	    cmd[4] = 0;
	    cmd[5] = 0;
	    break;

	case READ_6:
	    /* FALLTHROUGH */

	case WRITE_6:
	    CAM_DEBUG(ccb_h->path, CAM_DEBUG_SUBTRACE, 
		      ("Translating %s into _10 equivalent\n",
		      (cmd[0] == READ_6) ? "READ_6" : "WRITE_6"));
	    cmd[0] |= 0x20;
	    cmd[9] = cmd[5];
	    cmd[8] = cmd[4];
	    cmd[7] = 0;
	    cmd[6] = 0;
	    cmd[5] = cmd[3];
	    cmd[4] = cmd[2];
	    cmd[3] = cmd[1] & 0x1f;
	    cmd[2] = 0;
	    cmd[1] = 0;
	    break;
		}

	    /* Send to device */
	if (sense) {
		sense_cmd.opcode = REQUEST_SENSE;
                sense_cmd.length = csio->sense_len;
                sense_cmd.control = 0;            
		rc = p_ata_command(sc, (u_int8_t *)&sense_cmd, csio->sense_len);
	} else
		rc = p_ata_command(sc,cmd,len);
#ifdef PPCD_DEBUG
	device_printf(sc->device, "p_ata_command: rc=0x%x\n", rc);
#endif
	DELAY(1);
	if (!rc) {
		if (sense)
			rc = p_ata_completion(sc,(char *)&csio->sense_data);
		else
			rc = p_ata_completion(sc,csio->data_ptr);
	}

#ifdef PPCD_DEBUG
	device_printf(sc->device, "p_ata_completion: rc=0x%x\n", rc);
#endif
	if (rc) /* Signal the need of sense */
		sc->ppcd_stat = SCSI_STATUS_CHECK_COND;	
	else { /* XXX: translate ATA Status & Error register to SCSI */
		sc->ppcd_stat = SCSI_STATUS_OK;
		sc->ppcd_error = 0;
	     }
return rc;
}

static void
ppcd_identify(driver_t *driver, device_t parent)
{

	BUS_ADD_CHILD(parent, 0, "ppcd", 0);
}

/*
 * ppcd_probe()
 */
static int
ppcd_probe(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	struct ppcd_softc *sc = device_get_softc(dev);
/*	struct ppc_data *pd = device_get_softc(device_get_parent(ppbus)); */
	char scratch[4096];


	bzero(sc, sizeof(struct ppcd_softc));

	/* ppcd dependent initialisation */
	sc->unit = device_get_unit(dev);
	sc->device = dev;

/* XXX: We should add a sysctl to handle those */

#ifdef IODELAY
	sc->delay = IODELAY;
#endif 

/* HACK: The following is a hack to avoid porting frpw.c to microseq(9), and
 * thus making it too FreeBSD specific. Instead, we do the raw I/O, and assume
 * that if we grabbed the bus, we can do  whatever, as long as we restore the
 * port state before releasing it.
 * SIDE NOTE: The microseq(9) engine is an FSM, byte-code interpreter and the
 * core of it is the isa/ppc.c. The next abstraction layer is the macros and
 * the dev/ppbus/microseq.c. Thus, in terms of device_t's, ppc is the parent
 * of the ppbus and we can do device_get_parent to get the info from its softc
 * and call the appropriate low-level functions to manipulate it.
 */
/*	device_printf(dev, "@ %p\n", pd); */
/*	device_printf(dev, "@ 0x%x\n", pd->ppc_base);*/

/* XXX: ppbus: ppc */
	sc->port = 0x378;

	/* XXX: Returns previous mode, save it;
         *	Somehow adjust to select best mode available.
	 */
	ppb_set_mode(ppbus, PPB_EPP);
	sc->mode = 2;

	frpw_load_microcode(sc);

	/* low level probe */
	if (frpw_test_proto(sc, scratch, bootverbose))
		return (ENXIO);

	switch(sc->model) {
	case MODEL_XILINX:
		device_set_desc(dev,
			"FreeCom PowerCD (Xilinx) Parallel to ATAPI interface");
	break;
	case MODEL_ASIC:
		device_set_desc(dev,
			"FreeCom PowerCD (ASIC) Parallel to ATAPI interface");
	break;
	case MODEL_DETECT:
	default:
		return (ENXIO);
	}

	if (p_atapi_probe(sc, -1))
		sc->drive = 0; /* Master */
#ifndef PPCD_DEBUG
	if (bootverbose)
#endif 
	  frpw_log_adapter(sc);

	return (0);
}

/*
 * ppcd_attach()
 */
static int
ppcd_attach(device_t dev)
{
	struct ppcd_softc *sc = device_get_softc(dev);
	struct cam_devq *devq;
	
	/* STAILQ_INIT */
	sc->stqh_first = NULL;
	sc->stqh_last = &sc->stqh_first;

	callout_handle_init(&sc->ata_task);

	/*
	**	Now tell the generic SCSI layer
	**	about our bus.
	*/
	devq = cam_simq_alloc(/*maxopenings*/1);
	/* XXX What about low-level detach on error? */
	if (devq == NULL)
		return (ENXIO);

	sc->sim = cam_sim_alloc(ppcd_action, ppcd_poll, "ppcd", sc,
				 device_get_unit(dev),
				 /*untagged*/1, /*tagged*/0, devq);
	if (sc->sim == NULL) {
		cam_simq_free(devq);
		return (ENXIO);
	}

	if (xpt_bus_register(sc->sim, /*bus*/0) != CAM_SUCCESS) {
		cam_sim_free(sc->sim, /*free_devq*/TRUE);
		return (ENXIO);
	}

	/* all went ok */

	ppcd_cam_rescan(sc);	/* have CAM rescan the bus */

	return (0);
}

/*
 * ppcd_detach()
 */
static int
ppcd_detach(device_t dev)
{
	struct ppcd_softc *sc = device_get_softc(dev);

        if (sc->sim) {
#if __FreeBSD_version >= 500000 /* 4.x CAM can't handle disappearing SIMs yet */
                if (xpt_bus_deregister(cam_sim_path(sc->sim)))
                        cam_sim_free(sc->sim, /*free_devq*/TRUE);
                else
#endif
                        return(EBUSY);
		untimeout(ppcd_intr, (void *)sc, sc->ata_task);
                sc->sim = NULL;
        }

        return(0);
}                    

static void
ppcd_cam_rescan_callback(struct cam_periph *periph, union ccb *ccb)
{
        free(ccb, M_TEMP);
}

static void
ppcd_cam_rescan(struct ppcd_softc *sc)
{
        struct cam_path *path;
        union ccb *ccb = malloc(sizeof(union ccb), M_TEMP, M_WAITOK);

        bzero(ccb, sizeof(union ccb));

        if (xpt_create_path(&path, xpt_periph, cam_sim_path(sc->sim), 0, 0)
            != CAM_REQ_CMP) {
		/* A failure is benign as the user can do a manual rescan */
                return;
	}

        xpt_setup_ccb(&ccb->ccb_h, path, 5/*priority (low)*/);
        ccb->ccb_h.func_code = XPT_SCAN_BUS;
        ccb->ccb_h.cbfcnp = ppcd_cam_rescan_callback;
        ccb->crcn.flags = CAM_FLAG_NONE;
        xpt_action(ccb);

        /* The scan is in progress now. */
}

/*
 * ppcd_intr()
 */
static void
ppcd_intr(void *psc)
{
	int errno;	/* error in errno.h */
	int s;
	int empty;
	int ts, tc;	/* temporary status & count */
	union ccb *ccb;
	struct ccb_scsiio *csio;
	struct ppcd_softc *sc = psc;
#ifdef PPCD_DEBUG
	int i;
#endif

next_ccb:

	s = splbio();
	ccb = (union ccb *) sc->stqh_first; /* STAILQ_FIRST */
	splx(s);

	csio = &ccb->csio;

	errno = ppcd_atapi_do_scsi(sc, PPCD_INITIATOR, ccb, 0);

#ifdef PPCD_DEBUG
	printf("ppcd_atapi_do_scsi = %d, status = 0x%x, count = %d, ppcd_error = 0x%x\n", 
		 errno, sc->ppcd_stat, sc->ppcd_count, sc->ppcd_error);

	/* dump of command */
	for (i=0; i<csio->cdb_len; i++)
		printf("%x ", ((char *)&csio->cdb_io.cdb_bytes)[i]);

	printf("\n");
#endif

#if XXX
	if (errno) {
		/* connection to ppbus interrupted */
		csio->ccb_h.status = CAM_CMD_TIMEOUT;
		goto error;
	}

	/* if a timeout occured, no sense */
	if (sc->ppcd_error) {
		if (sc->ppcd_error & PPCD_TIMEOUT)
			printf("ppcd%d: ppcd error/timeout (%d)\n",
				sc->unit, sc->ppcd_error);

		csio->ccb_h.status = CAM_CMD_TIMEOUT;
		goto error;
	}
#endif
	/* check scsi status */
	if (sc->ppcd_stat != SCSI_STATUS_OK) {
	   csio->scsi_status = sc->ppcd_stat;
	   /* check if we have to sense the drive */
	   if ((sc->ppcd_stat & SCSI_STATUS_CHECK_COND) != 0) {
		ts = sc->ppcd_stat; tc = sc->ppcd_count;
		errno = ppcd_atapi_do_scsi(sc, PPCD_INITIATOR, ccb, 1);
		sc->ppcd_sense_stat = sc->ppcd_stat; sc->ppcd_stat = ts;
		sc->ppcd_sense_count = sc->ppcd_count; sc->ppcd_count = tc;
#ifdef PPCD_DEBUG
		printf("(sense) ppcd_atapi_do_scsi = %d, status = 0x%x, count = %d, ppcd_error = %d\n", 
			errno, sc->ppcd_sense_stat, sc->ppcd_sense_count, sc->ppcd_error);
#endif

		/* check sense return status */
		if (errno == 0 && sc->ppcd_sense_stat == SCSI_STATUS_OK) {
		   /* sense ok */
		   csio->ccb_h.status = CAM_AUTOSNS_VALID | CAM_SCSI_STATUS_ERROR;
		   csio->sense_resid = csio->sense_len - sc->ppcd_sense_count;

#ifdef PPCD_DEBUG
		   /* dump of sense info */
		   printf("(sense) ");
		   for (i=0; i<sc->ppcd_sense_count; i++)
			printf("%x ", ((char *)&csio->sense_data)[i]);
		   printf("\n");
#endif

		} else {
		   /* sense failed */
		   csio->ccb_h.status = CAM_AUTOSENSE_FAIL;
		}
	   } else {
		/* no sense */
		csio->ccb_h.status = CAM_SCSI_STATUS_ERROR;			
	   }

	   goto error;
	}

	csio->resid = csio->dxfer_len - sc->ppcd_count;
	csio->ccb_h.status = CAM_REQ_CMP;

error:
	s = splbio();

	/* STAILQ_REMOVE_HEAD */
	if ( (sc->stqh_first=(union ccb *)sc->stqh_first->ccb_h.stailq_next) == NULL)
	  sc->stqh_last = &sc->stqh_first;
	empty = sc->stqh_first == NULL; /* STAILQ_EMPTY */

	splx(s);
 	ccb->ccb_h.status &= ~CAM_SIM_QUEUED;
	xpt_done(ccb);
	if (!empty)
	  goto next_ccb;
}

static void
ppcd_action(struct cam_sim *sim, union ccb *ccb)
{

	struct ppcd_softc *sc = (struct ppcd_softc *)sim->softc;

	switch (ccb->ccb_h.func_code) {
	case XPT_SCSI_IO:
	{
		int s;
		int was_empty;

#ifdef PPCD_DEBUG
		device_printf(sc->device, "XPT_SCSI_IO (0x%x) request\n",
			ccb->csio.cdb_io.cdb_bytes[0]);
#endif

		s = splcam();
		was_empty = sc->stqh_first == NULL; /* STAILQ_EMPTY */
		/* STAILQ_INSERT_TAIL */
		ccb->ccb_h.stailq_next = NULL;
		*(sc->stqh_last) = ccb;
		sc->stqh_last = (union ccb **) &ccb->ccb_h.stailq_next;
		ccb->ccb_h.status |= CAM_SIM_QUEUED;
		splx(s);

		if (was_empty) /* schedule an ATA task */
	  	  sc->ata_task = timeout(ppcd_intr, (void *)sc, 1);
		break;
	}
	case XPT_CALC_GEOMETRY:
	{
		struct	  ccb_calc_geometry *ccg;

		ccg = &ccb->ccg;

#ifdef PPCD_DEBUG
		printf("ppcd%d: XPT_CALC_GEOMETRY (bs=%d,vs=%d,c=%d,h=%d,spt=%d) request\n",
			sc->unit,
			ccg->block_size,
			ccg->volume_size,
			ccg->cylinders,
			ccg->heads,
			ccg->secs_per_track);
#endif

		ccg->heads = 64;
		ccg->secs_per_track = 32;
		ccg->cylinders = ccg->volume_size /
				 (ccg->heads * ccg->secs_per_track);

		ccb->ccb_h.status = CAM_REQ_CMP;
		xpt_done(ccb);
		break;
	}
	case XPT_RESET_BUS:		/* Reset the specified SCSI bus */
	{

#ifdef PPCD_DEBUG
		printf("ppcd%d: XPT_RESET_BUS request\n", sc->unit);
#endif
		if (p_ata_reset(sc)) {
			ccb->ccb_h.status = CAM_REQ_CMP_ERR;
			xpt_done(ccb);
			return;
		}

		frpw_load_microcode(sc);
		ccb->ccb_h.status = CAM_REQ_CMP;
		xpt_done(ccb);
		break;
	}
	case XPT_PATH_INQ:		/* Path routing inquiry */
	{
		struct ccb_pathinq *cpi = &ccb->cpi;
		
#ifdef PPCD_DEBUG
		printf("ppcd%d: XPT_PATH_INQ request\n", sc->unit);
#endif
		cpi->version_num = 1;
		cpi->hba_inquiry = 0;
		cpi->target_sprt = 0;
		cpi->hba_misc = 0;
		cpi->hba_eng_cnt = 0;
		cpi->max_target = 1;
		cpi->max_lun = 0;
		cpi->initiator_id = PPCD_INITIATOR;
		cpi->bus_id = sim->bus_id;
		cpi->base_transfer_speed = 93;
		strncpy(cpi->sim_vid, "FreeBSD", SIM_IDLEN);
		strncpy(cpi->hba_vid, "ppcd", HBA_IDLEN);
		strncpy(cpi->dev_name, sim->sim_name, DEV_IDLEN);
		cpi->unit_number = sim->unit_number;

		cpi->ccb_h.status = CAM_REQ_CMP;
		xpt_done(ccb);
		break;
	}
	default:
		ccb->ccb_h.status = CAM_REQ_INVALID;
		xpt_done(ccb);
		break;
	}

	return;
}

static void
ppcd_poll(struct cam_sim *sim)
{       
	/* The device is actually always polled through ppcd_action() */
	return;
}

static devclass_t ppcd_devclass;

static device_method_t ppcd_methods[] = {
	/* device interface */
	DEVMETHOD(device_identify,	ppcd_identify),
	DEVMETHOD(device_probe,		ppcd_probe),
	DEVMETHOD(device_attach,	ppcd_attach),
	DEVMETHOD(device_detach,	ppcd_detach),

	{ 0, 0 }
};

static driver_t ppcd_driver = {
	"ppcd",
	ppcd_methods,
	sizeof(struct ppcd_softc),
};

DRIVER_MODULE(ppcd, ppbus, ppcd_driver, ppcd_devclass, 0, 0);
