#BINDIR	= /tmp
SRCS	= bus_if.h device_if.h opt_cam.h opt_scsi.h opt_ccb.h ppbus_if.h
SRCS	+= ppcd.c ppcd.h frpw.c frpw.h
KMOD	= ppcd 
NOMAN	= t
KLDMOD	= t
.if defined(DEBUG_BUILD)
CFLAGS	+= -DPPCD_DEBUG
.endif

.if defined(VERBOSE_BUILD)
CFLAGS	+= -DPPCD_VERBOSE
.endif

.if defined(NO_FIRMWARE)
CFLAGS	+= -DDISABLE_MICROCODE_LOAD
.endif

.if defined(IODELAY)
CFLAGS	+= -DIODELAY=${IODELAY}
.endif

KLDLOAD		= /sbin/kldload
KLDUNLOAD	= /sbin/kldunload

CLEANFILES+= ${KMOD}

load: ${KMOD}.ko
	${KLDLOAD} -v ./${KMOD}.ko

unload:
	${KLDUNLOAD} -v -n ${KMOD}

.include <bsd.kmod.mk>
