#ifndef FRPW_H
#define FRPW_H


void frpw_connect( struct ppcd_softc *sc );
void frpw_disconnect( struct ppcd_softc *sc );
void frpw_write_regr( struct ppcd_softc *sc, int cont, int regr, int val);
int  frpw_read_regr( struct ppcd_softc *sc, int cont, int regr );
void frpw_read_block( struct ppcd_softc *sc, char * buf, int count);
void frpw_write_block( struct ppcd_softc *sc, char * buf, int count);

int frpw_test_proto( struct ppcd_softc *sc, char * scratch, int verbose);

void frpw_load_microcode( struct ppcd_softc *sc);

void frpw_log_adapter( struct ppcd_softc *sc );

#endif /* FRPW_H */
