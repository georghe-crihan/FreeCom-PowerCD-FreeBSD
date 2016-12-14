/* 
	frpw.c	(c) 1996-8  Grant R. Guenther <grant@torque.net>
		            Under the terms of the GNU public license

	frpw.c is a low-level protocol driver for the Freecom "Power"
	parallel port IDE adapter.
	
	Some applications of this adapter may require a "printer" reset
	prior to loading the driver.  This can be done by loading and
	unloading the "lp" driver, or it can be done by this driver
	if you define FRPW_HARD_RESET.  The latter is not recommended
	as it may upset devices on other ports.

*/

/* Changes:

        1.01    GRG 1998.05.06 init_proto, release_proto
			       fix chip detect
			       added EPP-16 and EPP-32
	1.02    GRG 1998.09.23 added hard reset to initialisation process
	1.03    GRG 1998.12.14 made hard reset conditional

*/

#define	FRPW_VERSION	"1.03" 

#include <sys/param.h>
#include <sys/bus.h> /* device_t et al */
#include <machine/clock.h> /* DELAY */
#include <machine/cpufunc.h> /* outb, inb */

#include "ppcd.h"

#define delay_p			(sc->delay?DELAY(sc->delay):0)
#define out_p(offs,byte)	outb(sc->port+offs, byte); delay_p;
#define in_p(offs)		(delay_p,inb(sc->port+offs))

#define wd(byte)                {out_p(0,byte);}
#define rd()                    (in_p(0) & 0xff)
#define ws(byte)                {out_p(1,byte);}
#define rs()                    (in_p(1) & 0xff)
#define wc(byte)                {out_p(2,byte);}
#define rc()                    (in_p(2) & 0xff)
#define wa(byte)                {out_p(3,byte);}
#define w4(byte)                {out_p(4,byte);}
#define r4()                    (in_p(4) & 0xff)
#define wdw(data)     		{outw(sc->port+4, data); delay_p;}
#define wdl(data)     		{outl(sc->port+4, data); delay_p;}
#define rdw()         		(delay_p,inw(sc->port+4)&0xffff)
#define rdl()         		(delay_p,inl(sc->port+4)&0xffffffff)

#include "frpw.h"

#include "microcode.h"

#define MODE_COMP 0x0
#define MODE_NIBB 0x1
#define MODE_PS2  0x2
#define MODE_EPP  0x3
#define MODE_EPPW 0x4
#define MODE_EPPD 0x5

#define cec4		wc(0xc);wc(0xe);wc(0xe);wc(0xc);wc(4);wc(4);wc(4);
#define nibbles2b(l,h)	(((l>>4)&0x0f)|(h&0xf0))

/* cont = 0 - access the IDE register file 
   cont = 1 - access the IDE command set 
*/

static int  cont_map[2] = { 0x08, 0x10 };

int
frpw_read_regr( struct ppcd_softc *sc, int cont, int regr )
{	int	h,l,r;

	r = regr + cont_map[cont];

	wc(4);
	wd(r); cec4;
	wc(6); l = rs();
	wc(4); h = rs();
	wc(4); 

	return nibbles2b(l,h);

}

void
frpw_write_regr( struct ppcd_softc *sc, int cont, int regr, int val )
{	int r;

        r = regr + cont_map[cont];

	wc(4); wd(r); cec4; 
	wd(val);
	wc(5);wc(7);wc(5);wc(4);
}

static void
frpw_read_block_int( struct ppcd_softc *sc, char * buf, int count, int regr )
{       int     h, l, k, ph;

        switch(sc->mode) {

        case MODE_COMP:
		wc(4); wd(regr); cec4;
                for (k=0;k<count;k++) {
                        wc(6); l = rs();
                        wc(4); h = rs();
                        buf[k] = nibbles2b(l,h);
                }
                wc(4);
                break;

        case MODE_NIBB: 
		ph = 2;
                wc(4); wd(regr + 0xc0); cec4;
                wd(0xff);
                for (k=0;k<count;k++) {
                        wc(0xa4 + ph); 
                        buf[k] = rd();
                        ph = 2 - ph;
                } 
                wc(0xac); wc(0xa4); wc(4);
                break;

        case MODE_PS2:
		wc(4); wd(regr + 0x80); cec4;
                for (k=0;k<count;k++) buf[k] = r4();
                wc(0xac); wc(0xa4);
                wc(4);
                break;

	case MODE_EPP:
		wc(4); wd(regr + 0x80); cec4;
		for (k=0;k<count-2;k++) buf[k] = r4();
		wc(0xac); wc(0xa4);
		buf[count-2] = r4();
		buf[count-1] = r4();
		wc(4);
		break;

	case MODE_EPPW:
		wc(4); wd(regr + 0x80); cec4;
                for (k=0;k<(count/2)-1;k++) ((u_int16_t *)buf)[k] = rdw();
                wc(0xac); wc(0xa4);
                buf[count-2] = r4();
                buf[count-1] = r4();
                wc(4);
                break;

	case MODE_EPPD:
		wc(4); wd(regr + 0x80); cec4;
                for (k=0;k<(count/4)-1;k++) ((u_int32_t *)buf)[k] = rdl();
                buf[count-4] = r4();
                buf[count-3] = r4();
                wc(0xac); wc(0xa4);
                buf[count-2] = r4();
                buf[count-1] = r4();
                wc(4);
                break;

        }
}

void
frpw_read_block( struct ppcd_softc *sc, char * buf, int count)
{	frpw_read_block_int(sc,buf,count,0x08);
}

void
frpw_write_block( struct ppcd_softc *sc, char * buf, int count )
{	int	k;
	
	switch(sc->mode) {
	case MODE_COMP:
	case MODE_NIBB:
	case MODE_PS2:
		wc(4); wd(8); cec4; wc(5);
        	for (k=0;k<count;k++) {
			wd(buf[k]);
			wc(7);wc(5);
		}
		wc(4);
		break;

	case MODE_EPP:
		wc(4); wd(0xc8); cec4; wc(5);
		for (k=0;k<count;k++) w4(buf[k]);
		wc(4);
		break;

        case MODE_EPPW:
		wc(4); wd(0xc8); cec4; wc(5);
                for (k=0;k<count/2;k++) wdw(((u_int16_t *)buf)[k]);
                wc(4);
                break;

        case MODE_EPPD:
		wc(4); wd(0xc8); cec4; wc(5);
                for (k=0;k<count/4;k++) wdl(((u_int32_t *)buf)[k]);
                wc(4);
                break;
	}
}

void
frpw_connect ( struct ppcd_softc *sc  )
{       sc->saved_rd = rd();
        sc->saved_rc = rc();
	wc(4);
}

void
frpw_disconnect ( struct ppcd_softc *sc )
{       wc(4); wd(0x20); cec4;
	wd(sc->saved_rd);
        wc(sc->saved_rc);
} 

/* Stub logic to see if PNP string is available - used to distinguish
   between the Xilinx and ASIC implementations of the Freecom adapter.
   returns chip_type:   0 = Xilinx, 1 = ASIC
*/

static int
frpw_test_pnp ( struct ppcd_softc *sc )
{	int olddelay, a, b;

#ifdef FRPW_HARD_RESET
        wd(0); wc(8); DELAY(50); wc(0xc);   /* parallel bus reset */
        UDELAY(1500);
#endif

	olddelay = sc->delay;
	sc->delay = 10;

	sc->saved_rd = rd();
        sc->saved_rc = rc();
	
	wc(4); wd(4); wc(6); wc(7);
	a = rs() & 0xff; wc(4); b = rs() & 0xff;
	wc(0xc); wc(0xe); wc(4);

	sc->delay = olddelay;
        wd(sc->saved_rd);
        wc(sc->saved_rc);

	return ((~a&0x40) && (b&0x40)) + 1;
} 

int
frpw_test_proto( struct ppcd_softc *sc, char * scratch, int verbose )
{       int     j, k, r;
	int	e[2] = {0,0};

	if (sc->model == MODEL_DETECT)
	   sc->model = frpw_test_pnp(sc);

	if ((sc->model == MODEL_XILINX) && (sc->mode > 2)) {
	   if (verbose) 
		device_printf(sc->device,
			"frpw: Xilinx does not support mode %d\n", sc->mode);
	   return 1;
	}

	if ((sc->model == MODEL_ASIC) && (sc->mode == 2)) {
	   if (verbose)
		device_printf(sc->device,
			"frpw: ASIC does not support mode 2\n");
	   return 1;
	}

	frpw_connect(sc);
	for (j=0;j<2;j++) {
                frpw_write_regr(sc,0,6,0xa0+j*0x10);
                for (k=0;k<256;k++) {
                        frpw_write_regr(sc,0,2,k^0xaa);
                        frpw_write_regr(sc,0,3,k^0x55);
                        if (frpw_read_regr(sc,0,2) != (k^0xaa)) e[j]++;
                        }
                }
	frpw_disconnect(sc);

	frpw_connect(sc);
        frpw_read_block_int(sc,scratch,512,0x10);
        r = 0;
        for (k=0;k<128;k++) if (scratch[k] != k) r++;
	frpw_disconnect(sc);

        if (verbose)  {
            device_printf(sc->device,
		   "frpw: port 0x%x, chip %d, mode %d, test=(%d,%d,%d)\n",
                   sc->port,sc->model,sc->mode,e[0],e[1],r);
        }

        return (r || (e[0] && e[1]));
}

void
frpw_log_adapter( struct ppcd_softc *sc )
{       char    *mode_string[6] = {"4-bit","8-bit","EPP",
				   "EPP-8","EPP-16","EPP-32"};

        device_printf(sc->device,
		"frpw " FRPW_VERSION
		" Freecom (%s) adapter at 0x%x,\n"
		"mode %d (%s), delay %d\n",
		sc->model == MODEL_XILINX ? "Xilinx" : "ASIC", sc->port,
		sc->mode, mode_string[sc->mode], sc->delay);

}

