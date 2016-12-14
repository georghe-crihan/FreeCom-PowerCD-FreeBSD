#ifndef PPCD_H
#define PPCD_H

#include <sys/systm.h>
#include <cam/cam.h>
#include <cam/cam_ccb.h>

struct ppcd_softc {
	unsigned short unit; /* device unit */
	int	drive;	/* master/slave */
	int	model;	/* Chip type */
#define MODEL_DETECT	0x0
#define MODEL_XILINX	0x1
#define	MODEL_ASIC	0x2
	device_t device;/* device_t */
	struct	cam_sim *sim;
	int	port;	/* parallel port base */
	int	mode;	/* transfer mode in use */
	int	delay;	/* adapter delay setting */
			/* saved port state */
	int	saved_rd;/* data register */
	int	saved_rc;/* control register */

	int ppcd_stat;
	int ppcd_count;
	int ppcd_error;

	unsigned int ppcd_sense_stat;
	unsigned int ppcd_sense_count;

	void *ppcd_io;	/* interface to low level functions */
	/* STAILQ_HEAD */
       	union ccb *stqh_first; /* first element */
       	union ccb **stqh_last; /* addr of last next element */
	struct callout_handle ata_task; /* for request queue restart */
};

#define PPCD_INITIATOR	0x1

#endif /* PPCD_H */
