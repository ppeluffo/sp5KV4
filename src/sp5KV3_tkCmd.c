/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include <sp5KV3.h>
#include "FRTOS-IO.h"
#include <ctype.h>
#include "mcp_sp5K.h"
#include "ads7828_sp5K.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

static u08 pvMakeargv(void);
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdRedialFunction(void);
static void cmdStatusFunction(void);

// Debug----
static void cmdPrintFunction(void);
static void cmdSearchFunction(void);
s08 pvSetParamRtc(u08 *s);
// ---------

char *cb_strstr(Peripheral_Control_t *UART, const char substring[], int *pos);

/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

u08 c;
( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	MCP_init();

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand("cls", cmdClearScreen );
	cmdlineAddCommand("help", cmdHelpFunction);
	cmdlineAddCommand("reset", cmdResetFunction);
//	cmdlineAddCommand("print", cmdPrintFunction);
//	cmdlineAddCommand("search", cmdSearchFunction);
	cmdlineAddCommand("read", cmdReadFunction);
	cmdlineAddCommand("write", cmdWriteFunction);
	cmdlineAddCommand("redial", cmdRedialFunction);
	cmdlineAddCommand("status", cmdStatusFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// loop
	for( ;; )
	{
//		clearWdg(WDG_CMD);

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
			cmdlineInputFunc(c);
		}

		// Mientras escribo comandos no apago la terminal
		if (c=='\r') {
//			pb_restartTimerTerminal();
		}

		/* run the cmdline execution functions */
		cmdlineMainLoop();

	}
}
/*------------------------------------------------------------------------------------*/
static u08 pvMakeargv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, NULL, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{

RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 pos;
u08 argc;
u08 regValue;
u16 adcRetValue;

	argc = pvMakeargv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_read( (u16)(atoi(argv[2])), cmd_printfBuff, atoi(argv[3]) );
		if ( retS ) {
			snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
	}

	// ADC
	if (!strcmp_P( strupr(argv[1]), PSTR("ADC\0"))) {
		switch(atoi(argv[2])) {
		case 0:
			retS = ADS7827_readCh0( &adcRetValue );
			break;
		case 1:
			retS = ADS7827_readCh1( &adcRetValue );
			break;
		case 2:
			retS = ADS7827_readCh2( &adcRetValue );
			break;
		case 3:
			retS = ADS7827_readBatt( &adcRetValue );
			break;
		}
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ACD[%d]=%d\r\n\0"), atoi(argv[2]), adcRetValue );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	// Lee la hora del RTC.
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = RTC_read(&rtcDateTime);
		if (retS ) {
			pos = snprintf_P( cmd_printfBuff,CHAR128,PSTR("OK\r\n"));
			pos += snprintf_P( &cmd_printfBuff[pos],CHAR128,PSTR("%02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
			pos += snprintf_P( &cmd_printfBuff[pos],CHAR128,PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
		} else {
			snprintf_P( cmd_printfBuff,CHAR128,PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
			break;
		case 1:
			retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
			break;
		}
		if (retS ) {
			// Convierto el resultado a binario.
			pos = snprintf_P( cmd_printfBuff,CHAR128,PSTR("OK\r\n"));
			pos += snprintf_P( &cmd_printfBuff[pos],CHAR128,PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
		} else {
			snprintf_P( cmd_printfBuff,CHAR128,PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;
u32 param1 = 0;
u08 length = 0;
char *p;

	argc = pvMakeargv();

	// EE
	// write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		p = argv[3];
		while (*p != NULL) {
			p++;
			length++;
		}
		retS = EE_write( (u16)(atoi(argv[2])), argv[3], length );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = pvSetParamRtc(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// LED
	// write led 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("LED\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setLed_LogicBoard( (u08)param1 );
		if ( param1 == 1 ) {
			cbi(LED_KA_PORT, LED_KA_BIT);
		} else {
			sbi(LED_KA_PORT, LED_KA_BIT);
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setGprsPwr( (u08) param1 );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setGprsSw( (u08) param1 );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// termPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMPWR\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setTermPwr( (u08)param1 );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// sensorPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORPWR\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setSensorPwr( (u08)param1 );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOGPWR\0"))) {
		param1 = atoi(argv[2]);
		retS = MCP_setAnalogPwr( (u08)param1 );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_write( MCP0_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		case 1:
			retS = MCP_write( MCP1_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}


}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset {default,memory}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-redial\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "      param { wrkmode [service | monitor {sqe|frame}], pwrmode [continuo|discreto] } \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              timerpoll, timerdial, dlgid, gsmband \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              pwrsave [modo {on|off}, {hhmm1}, {hhmm2}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              debuglevel +/-{none,basic,mem,eventos,data,gprs,digital,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              loglevel (none, info, all)\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              A{0..3} aname imin imax mmin mmax\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              D{0..1} dname magp\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              apn, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              output [consigna|manual] o0,01,02,03,hhmm1,hhmm2,chVA,chVB  \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       mcp devId regAddr regValue\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       clearq0,clearq1\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       led {0|1},gprspwr {0|1},gprssw {0|1},termpwr {0|1},sensorpwr {0|1},analogpwr {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       atcmd {atcmd,timeout}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       output {reset|noreset|sleep|nosleep},{enable|disable} {0..3}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       output phase {0..3} {01|10}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       output pulse {0..3} {01|10} {ms}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("       output consigna{diurna|nocturna} ms\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read rtc\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("      mcp devId regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("      dcd,ri,termsw,din0,din1\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("      frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("      ee addr lenght\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("      memory \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pvMakeargv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
//		MEM_drop();
	}

	cmdClearScreen();
	wdt_enable(WDTO_30MS);
	while(1) {}

}
/*------------------------------------------------------------------------------------*/
static void cmdRedialFunction(void)
{
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkGprs, GPRSMSG_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
u16 pos;
u08 channel;
u08 modemStatus;
frameDataType Cframe;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	rtcGetTime(&rtcDateTime);
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.serverAddress,systemVars.serverPort );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(900)"));
		break;
	case 1:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(1800)"));
		break;
	case 2:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("pcs (1900)"));
		break;
	case 4:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("gsm (850)"));
		break;
	case 5:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem status */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  modem: "));
	modemStatus = getGprsModemStatus();
	switch ( modemStatus ) {
	case M_OFF:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("OFF\r\n\0"));
		break;
	case M_OFF_IDLE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("OFF IDLE\r\n\0"));
		break;
	case M_ON_CONFIG:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ON CONFIG\r\n\0"));
		break;
	case M_ON_READY:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ON READY\r\n\0"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dlg ip: %s\r\n\0"), systemVars.dlgIp );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// DCD/RI/TERMSW
	if ( systemVars.dcd == 0 ) { pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pines: dcd=ON,\0")); }
	if ( systemVars.dcd == 1 ) { pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pines: dcd=OFF,\0"));}
	if ( systemVars.ri == 0 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ri=ON,\0")); }
	if ( systemVars.ri == 1 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ri=OFF,\0"));}
	if ( systemVars.termsw == 1 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("term=ON\r\n\0")); }
	if ( systemVars.termsw == 0 ) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("term=OFF\r\n\0"));}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
//	snprintf_P( cmd_printfBuff,CHAR256,PSTR("  memory: pWr=%d,pRd=%d,pDel=%d,free=%d,4rd=%d,4del=%d  \r\n"), MEM_getWrPtr(), MEM_getRdPtr(), MEM_getDELptr(), MEM_getRcdsFree(),MEM_getRcds4rd(),MEM_getRcds4del() );
//	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* WRK mode (NORMAL / SERVICE) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  wrkmode: "));
	/* WRK mode (NORMAL / SERVICE) */
	switch (systemVars.wrkMode) {
	case WK_NORMAL:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("normal\r\n"));
		break;
	case WK_SERVICE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("service\r\n"));
		break;
	case WK_MONITOR_FRAME:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_frame\r\n"));
		break;
	case WK_MONITOR_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_sqe\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* PWR mode (CONTINUO / DISCRETO) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrmode: "));
	switch (systemVars.pwrMode) {
	case PWR_CONTINUO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("continuo\r\n"));
		break;
	case PWR_DISCRETO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("discreto\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Pwr.Save */
	if ( systemVars.pwrSave == modoPWRSAVE_ON ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrSave ON: "));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d->"),pv_convertMINS2hhmm( systemVars.pwrSaveStartTime));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d\r\n"),pv_convertMINS2hhmm( systemVars.pwrSaveEndTime));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrSave OFF\r\n"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll: %d | %d\r\n\0"),systemVars.timerPoll, getTimeToNextPoll() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerDial: %lu | %lu\r\n\0"), systemVars.timerDial, getTimeToNextDial() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DebugLevel */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debugLevel: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("none") );
	} else {
		if ( (systemVars.debugLevel & D_BASIC) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+basic")); }
		if ( (systemVars.debugLevel & D_DATA) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+data")); }
		if ( (systemVars.debugLevel & D_GPRS) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+gprs")); }
		if ( (systemVars.debugLevel & D_MEM) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+mem")); }
		if ( (systemVars.debugLevel & D_EVENTOS) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+ev")); }
		if ( (systemVars.debugLevel & D_DIGITAL) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+digital")); }
	}
	snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Bateria
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  batt{0-15V}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	for ( channel = 0; channel < NRO_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d{%d-%dmA/%d-%.1f}(%s)\r\n\0"),channel, systemVars.Imin[channel],systemVars.Imax[channel],systemVars.Mmin[channel],systemVars.Mmax[channel], systemVars. aChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	/* Configuracion de canales digitales */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d0{%.2f p/p} (%s)\r\n\0"), systemVars.magPP[0],systemVars.dChName[0]);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d1{%.2f p/p} (%s)\r\n\0"), systemVars.magPP[1],systemVars.dChName[1]);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Salidas */
	// Modo
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  outputs mode:consigna [chVA=%d] [chVB=%d]"),systemVars.outputs.chVA,systemVars.outputs.chVB);
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("  C.Dia:[%04d]"), pv_convertMINS2hhmm( systemVars.outputs.horaConsDia ) );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("  C.Noc:[%04d]\r\n\0"), pv_convertMINS2hhmm( systemVars.outputs.horaConsNoc ));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* VALUES --------------------------------------------------------------------------------------- */
	getAnalogFrame (&Cframe);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Values:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("  "));
	// TimeStamp.
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff),PSTR( "%04d%02d%02d,"),Cframe.rtc.year,Cframe.rtc.month,Cframe.rtc.day );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%02d%02d%02d,"),Cframe.rtc.hour,Cframe.rtc.min, Cframe.rtc.sec );
	// Valores analogicos
	for ( channel = 0; channel < NRO_CHANNELS; channel++) {
		pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%.2f,"),systemVars.aChName[channel],Cframe.analogIn[channel] );
	}
	// Valores digitales
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.2f,%sL=%d,%sW=%d,"), systemVars.dChName[0],Cframe.dIn.din0_pulses,systemVars.dChName[0],Cframe.dIn.din0_level,systemVars.dChName[0], Cframe.dIn.din0_width);
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.2f,%sL=%d,%sW=%d"), systemVars.dChName[1],Cframe.dIn.din1_pulses,systemVars.dChName[1],Cframe.dIn.din1_level,systemVars.dChName[1], Cframe.dIn.din1_width);
	// Bateria
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR(",bt=%.2f\r\n\0"),Cframe.batt );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdPrintFunction(void)
{
UART_device_control_t *uartDevice;
fifo_handle_s *uartFifo;
char p[32];
int stringSize;
int head, tail;
u08 i;

	uartDevice = (UART_device_control_t *) pdUART1.phDevice;
	uartFifo = (fifo_handle_s *) uartDevice->rxStruct;

	memcpy(p, uartFifo->buff, 32 );
	for ( i = 0; i<32; i++) {
		if ( !  isalnum(p[i]) )
			p[i] = '.';
	}
	p[31] = '\0';

	stringSize = uartFifo->length;
	head = uartFifo->head;
	tail = uartFifo->tail;

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Fifo[%s][size %d][head %d][tail %d]\r\n\0"),p,stringSize, head, tail);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

//	memcpy(cmd_printfBuff,p, 32 );
//	cmd_printfBuff[32] = '\0';
//	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdSearchFunction(void)
{
UART_device_control_t *uartDevice;
fifo_handle_s *uartFifo;
char *p;
int pos;

	uartDevice = (UART_device_control_t *) pdUART1.phDevice;
	uartFifo = (fifo_handle_s *) uartDevice->rxStruct;

	p = cb_strstr(&pdUART1, "pablo\0", &pos );
	if ( p == NULL) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Not Found\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Found[%d]\r\n\0"), pos);
	}

	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	cmdPrintFunction();
}
/*------------------------------------------------------------------------------------*/
char *cb_strstr(Peripheral_Control_t *UART, const char *s, int *pos )
{
int i,f;
char *p,*q, *res = NULL;
int stringSize;
s08 inString = FALSE;
s08 rollover = FALSE;

UART_device_control_t *uartDevice;
fifo_handle_s *uartFifo;

	// Busca un substring en un string circular

	uartDevice = (UART_device_control_t *) UART->phDevice;
	uartFifo = (fifo_handle_s *) uartDevice->rxStruct;

	p = uartFifo->buff;
	stringSize = uartFifo->length;
	q = s;

	// Chequeo que el substrig no sea vacio.
    if ( *s == 0) {
    	*pos = 0;
       	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR1\r\n\0"));
        FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return p;
    }

	// Recorro el string base. Lo recorro todo ya que como es circular, puede
	// tener un \0 en el medio.
    i = 0;
	inString = FALSE;
    while(1) {
		if ( *p == *q ) {
			if ( inString == FALSE ) {
				res = p;	// Guardo la primer posicion de coincidencia.
				*pos = i;
			}
			inString = TRUE;
			q++;
			if ( *q == '\0')
				// Esta es la unica condicion de salida valida.
				break;
		} else {
			// Reinicio las condiciones de busqueda
			inString = FALSE;
			q = s;
			*pos = 0;
			if ( rollover )	// Ya di vuelta y no coincide.
				break;
		}
		// Avanzo
		p++;
		i++;
		if ( i == stringSize ) {
			// Llegue al final. Rollover.
			i = 0;
			p = uartFifo->buff;
			rollover = TRUE;
		}
    }

    if (inString) {
    	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK[%d]\r\n\0"),f);
	 } else {
		res = NULL;
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("FAIL[%d]\r\n\0"),i);
   	}
    FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
    return(res);

}
//------------------------------------------------------------------------------------
s08 pvSetParamRtc(u08 *s)
{
u08 dateTimeStr[11];
char tmp[3];
s08 retS;
RtcTimeType_t rtcDateTime;


	/* YYMMDDhhmm */
	memcpy(dateTimeStr, s, 10);
	// year
	tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
	rtcDateTime.year = atoi(tmp);
	// month
	tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
	rtcDateTime.month = atoi(tmp);
	// day of month
	tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
	rtcDateTime.day = atoi(tmp);
	// hour
	tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
	rtcDateTime.hour = atoi(tmp);
	// minute
	tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
	rtcDateTime.min = atoi(tmp);

	retS = RTC_write(&rtcDateTime);
	return(retS);
}
/*------------------------------------------------------------------------------------*/

