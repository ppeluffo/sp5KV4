/*
 * file_sp5K.h
 *
 *  Created on: 31/10/2015
 *      Author: pablo
 */

#ifndef SRC_SP5KLIBS_FILE_SP5K_H_
#define SRC_SP5KLIBS_FILE_SP5K_H_

#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include "global.h"
#include <avr/wdt.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "FRTOS-IO.h"
#include "ee_sp5K.h"

#define FF_SIZE_IN_KB	32	// Tamanio en KB de la eeprom externa.
#define FF_RECD_SIZE	64	// Tamanio del registro
#define FF_ADDR_START	0	// Posicion inicial
#define FF_MAX_RCDS		512	// Cantidad de registros

#define FF_TAG	0xC5

typedef struct {
	u16 WRptr;		// Estructura de control de archivo
	u16 RDptr;
	u16 DELptr;
	u16 rcds4wr;
	u16 rcds4rd;
	u16 rcds4del;
	u08 errno;
} StatBuffer_t;

typedef struct {					// File Control Block
	StatBuffer_t ff_stat;			// Estructura de control de archivo
	char ff_buffer[FF_RECD_SIZE];	//
	char check_buffer[FF_RECD_SIZE];
} FCB_t;

FCB_t FCB;

#define pdFF_ERRNO_NONE		0
#define pdFF_ERRNO_MEMFULL	1
#define pdFF_ERRNO_MEMWR	2
#define pdFF_ERRNO_MEMEMPTY	3
#define pdFF_ERRNO_MEMRD	4
#define pdFF_ERRNO_RDCKS	5
#define pdFF_ERRNO_RDNOTAG	6
#define pdFF_ERRNO_INIT		7


//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
size_t FF_fopen(void);
size_t FF_fwrite( const void *pvBuffer, size_t xSize);
size_t FF_fread( void *pvBuffer, size_t xSize);
void FF_stat( StatBuffer_t *pxStatBuffer );
s08 FF_truncate(void);
s08 FF_rewind(void);
s08 FF_seek(void);
int FF_errno( void );
s08 FF_del(void);
//------------------------------------------------------------------------------------

#endif /* SRC_SP5KLIBS_FILE_SP5K_H_ */
