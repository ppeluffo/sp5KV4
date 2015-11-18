/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "sp5KV4.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static u08 pv_makeArgv(void);

void pv_cmdRdRTC(void);
void pv_cmdRdDCD(void);
void pv_cmdRdEE(void);
void pv_cmdRdADC(void);
void pv_cmdRdMCP(void);
void pv_cmdRdDIN(void);
void pv_cmdRdTERMSW(void);
void pv_cmdRdDefaults(void);
s08 pv_cmdWrDebugLevel(u08 *s);
s08 pv_cmdWrWrkMode(u08 *s0, u08 *s1);
s08 pv_cmdWrEE(u08 *s0, u08 *s1);
s08 pv_cmdWrRtc(u08 *s);
static void pv_readMemory(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdRedialFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

u08 c;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand("cls", cmdClearScreen );
	cmdlineAddCommand("help", cmdHelpFunction);
	cmdlineAddCommand("reset", cmdResetFunction);
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
		u_clearWdg(WDG_CMD);

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
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  wrkmode [service | monitor {sqe|frame}], pwrmode [continuo|discreto] \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, timerdial, dlgid, gsmband\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrsave [modo {on|off}, {hhmm1}, {hhmm2}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debuglevel +/-{none,basic,mem,eventos,data,gprs,digital,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  loglevel (none, info, all)\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  A{0..3} aname imin imax mmin mmax\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  D{0..1} dname magp\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consigna {on|off} hhmm1,hhmm2,chVA,chVB \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) mcp devId regAddr regValue\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) clearQ0 clearQ1\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) led {0|1},gprspwr {0|1},gprssw {0|1},termpwr {0|1},sensorpwr {0|1},analogpwr {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
//	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  atcmd {atcmd,timeout}\r\n\0"));
//	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) output {reset|noreset|sleep|nosleep},{enable|disable} {0..3}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        phase {0..3} {01|10}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM)        pulse {0..3} {01|10} {ms}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("              consigna{diurna|nocturna} ms\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) atcmd {cmd}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {0|1} regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dcd,termsw,din {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, adc {ch}, frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr lenght\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) frame,memory,gprs\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  defaults \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
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
	while ( xTaskNotify(xHandle_tkGprs,TKG_PARAM_RELOAD , eSetBits ) != pdPASS ) {
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
frameData_t Cframe;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
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
	//modemStatus = getGprsModemStatus();
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
	FF_stat(&pxFFStatBuffer);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

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
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d->"),u_convertMINS2hhmm( systemVars.pwrSaveStartTime));
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%04d\r\n"),u_convertMINS2hhmm( systemVars.pwrSaveEndTime));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrSave OFF\r\n"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll [%ds]: %d\r\n\0"),systemVars.timerPoll, u_readTimeToNextPoll() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerDial: [%lus]: %li\r\n\0"), systemVars.timerDial, u_readTimeToNextDial() );
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

	/* Consignas */
	if ( systemVars.consigna.status == CONSIGNA_ON ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consignas: [chVA=%d] [chVB=%d]"),systemVars.consigna.chVA,systemVars.consigna.chVB);
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("  C.Dia:[%04d]"), u_convertMINS2hhmm( systemVars.consigna.horaConsDia ) );
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("  C.Noc:[%04d]\r\n\0"), u_convertMINS2hhmm( systemVars.consigna.horaConsNoc ));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consignas: OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* VALUES --------------------------------------------------------------------------------------- */
	memset(&Cframe,'\0', sizeof(frameData_t));
	u_readAnalogFrame (&Cframe);
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
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.2f,%sL=%d,"), systemVars.dChName[0],Cframe.dIn.pulses[0],systemVars.dChName[0],Cframe.dIn.level[0]);
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%sP=%.2f,%sL=%d,"), systemVars.dChName[1],Cframe.dIn.pulses[1],systemVars.dChName[1],Cframe.dIn.level[1]);
	// Bateria
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("bt=%.2f\r\n\0"),Cframe.batt );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
u08 argc;
char *p;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmdRdEE();
		return;
	}

	// ADC
	// read adc channel
	if (!strcmp_P( strupr(argv[1]), PSTR("ADC\0"))) {
		pv_cmdRdADC();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		pv_cmdRdRTC();
		return;
	}

	// DCD
	if (!strcmp_P( strupr(argv[1]), PSTR("DCD\0"))) {
		pv_cmdRdDCD();
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		pv_cmdRdMCP();
		return;
	}

	// DIN
	// read din 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0"))) {
		pv_cmdRdDIN();
		return;
	}

	// TERMSW
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMSW\0"))) {
		pv_cmdRdTERMSW();
		return;
	}

	// DEFAULT
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		u_loadDefaults();
		return;
	}

	// FRAME
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		while ( xTaskNotify(xHandle_tkAIn, TKA_READ_FRAME , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		pv_readMemory();
		return;
	}

	// GPRS RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART0);
		FreeRTOS_write( &pdUART1, "rx->", sizeof("rx->")  );
		FreeRTOS_write( &pdUART1, p, UART0_RXBUFFER_LEN );
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0")  );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();

	// SAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = u_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverPort, '\0', sizeof(systemVars.serverPort));
			memcpy(systemVars.serverPort, argv[2], sizeof(systemVars.serverPort));
			systemVars.serverPort[PORT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverAddress, '\0', sizeof(systemVars.serverAddress));
			memcpy(systemVars.serverAddress, argv[2], sizeof(systemVars.serverAddress));
			systemVars.serverAddress[IP_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGLEVEL\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* WRKMODE */
	if (!strcmp_P( strupr(argv[1]), PSTR("WRKMODE\0"))) {
		retS = pv_cmdWrWrkMode(argv[2],argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS
	if (!strcmp_P( strupr(argv[1]), PSTR("A0\0"))) {
		retS = u_configAnalogCh( 0, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A1\0"))) {
		retS = u_configAnalogCh( 1, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A2\0"))) {
		retS = u_configAnalogCh( 2, argv[2],argv[3],argv[4],argv[5],argv[6]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES DIGITALES
	if (!strcmp_P( strupr(argv[1]), PSTR("D0\0"))) {
		u_configDigitalCh( 0, argv[2],argv[3]);
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("D1\0"))) {
		u_configDigitalCh( 1, argv[2],argv[3]);
		pv_snprintfP_OK();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = u_configTimerPoll(argv[2]);

		// tk_aIn: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkAIn, TKA_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// TIMERDIAL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0"))) {
		retS = u_configTimerDial(argv[2]);

		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRMODE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRMODE\0"))) {

		if ((!strcmp_P(strupr(argv[2]), PSTR("CONTINUO")))) {
			retS = u_configPwrMode(PWR_CONTINUO);
		}

		if ((!strcmp_P(strupr(argv[2]), PSTR("DISCRETO")))) {
			retS = u_configPwrMode(PWR_DISCRETO);
		}

		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { u_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { u_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = u_wrRtc(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CONSIGNA
	// consigna {on|off} hhmm1,hhmm2,ch1,ch2
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { u_configConsignas( CONSIGNA_OFF,argv[3],argv[4], atoi(argv[5]),atoi(argv[6])); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { u_configConsignas( CONSIGNA_ON,argv[3],argv[4], atoi(argv[5]),atoi(argv[6])); }
		pv_snprintfP_OK();
		return;
	}

	//----------------------------------------------------------------------
	// COMANDOS USADOS PARA DIAGNOSTICO
	// DEBEMOS ESTAR EN MODO SERVICE
	//----------------------------------------------------------------------

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = pv_cmdWrEE( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// LED
	// write led 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("LED\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setLed_LogicBoard( (u08)atoi(argv[2]) );
		if ( atoi(argv[2]) == 1 ) {
			cbi(LED_KA_PORT, LED_KA_BIT);
		} else {
			sbi(LED_KA_PORT, LED_KA_BIT);
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsSw( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// termPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("TERMPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setTermPwr( (u08)atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// sensorPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("SENSORPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setSensorPwr( (u08)atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOGPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setAnalogPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// analogPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("CLEARQ\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		if ( atoi(argv[2]) == 0 ) {
			cbi(Q_PORT, Q0_CTL_PIN);
			vTaskDelay( ( TickType_t)( 1) );
			sbi(Q_PORT, Q0_CTL_PIN);
		}
		if ( atoi(argv[2]) == 1 ) {
			cbi(Q_PORT, Q1_CTL_PIN);
			vTaskDelay( ( TickType_t)( 1) );
			sbi(Q_PORT, Q1_CTL_PIN);
		}
		return;
	}

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
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

	 // OUTPUTS
	// {reset|noreset|sleep|nosleep},{enable|disable}{0..3},consigna{diurna|nocturna}ms
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUT\0"))) {

		// reset
		if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsReset();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// noreset
		if (!strcmp_P( strupr(argv[2]), PSTR("NORESET\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsNoReset();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// sleep
		if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsSleep();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// nosleep
		if (!strcmp_P( strupr(argv[2]), PSTR("NOSLEEP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			retS = MCP_outputsNoSleep();
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// enable 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			switch(atoi(argv[3])) {
			case 0:
				retS = MCP_output0Enable();
				break;
			case 1:
				retS = MCP_output1Enable();
				break;
			case 2:
				retS = MCP_output2Enable();
				break;
			case 3:
				retS = MCP_output3Enable();
				break;
			default:
				retS = FALSE;
				break;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// disable 0,1,2,3
		if (!strcmp_P( strupr(argv[2]), PSTR("DISABLE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			switch(atoi(argv[3])) {
			case 0:
				retS = MCP_output0Disable();
				break;
			case 1:
				retS = MCP_output1Disable();
				break;
			case 2:
				retS = MCP_output2Disable();
				break;
			case 3:
				retS = MCP_output3Disable();
				break;
			default:
				retS = FALSE;
				break;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// phase {0,1,2,3} 01|10
		if (!strcmp_P( strupr(argv[2]), PSTR("PHASE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			switch(atoi(argv[3])) {
			case 0:
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_output0Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_output0Phase_10(); }
				break;
			case 1:
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_output1Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_output1Phase_10(); }
				break;
			case 2:
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_output2Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_output2Phase_10(); }
				break;
			case 3:
				if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_output3Phase_01(); }
				if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_output3Phase_10(); }
				break;
			default:
				retS = FALSE;
				break;
			}
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// pulse {0,1,2,3} 01|10 ms
		if (!strcmp_P( strupr(argv[2]), PSTR("PULSE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
			if (!strcmp_P( strupr(argv[4]), PSTR("01\0"))) { retS = MCP_outsPulse((u08) (atoi(argv[3])), 0, (u16) (atoi(argv[5])) ); }
			if (!strcmp_P( strupr(argv[4]), PSTR("10\0"))) { retS = MCP_outsPulse((u08) (atoi(argv[3])), 1, (u16) (atoi(argv[5])) ); }
			retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
			return;
		}

		// consigna { diurna | nocturna }
		if (!strcmp_P( strupr(argv[2]), PSTR("CONSIGNA\0"))) {
			if (!strcmp_P( strupr(argv[3]), PSTR("DIURNA\0"))) {
				u_setConsignaDiurna(atoi(argv[4]));
				snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Consigna DIURNA..\r\n\0"));
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("NOCTURNA\0"))) {
				u_setConsignaNocturna(atoi(argv[4]));
				snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Consigna NOCTURNA..\r\n\0"));
			}
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
			pv_snprintfP_OK();
			return;
		}

	}

	// ATCMD
	// Envia un comando al modem.
	if (!strcmp_P( strupr(argv[1]), PSTR("ATCMD\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r\0",argv[2] );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
		FreeRTOS_write( &pdUART0, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[2] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
s08 pv_cmdWrEE(u08 *s0, u08 *s1)
{
u08 length = 0;
char *p;
s08 retS = FALSE;

	p = s1;
	while (*p != NULL) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	retS = EE_write( (u16)(atoi(s0)), s1, length );
	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrWrkMode(u08 *s0, u08 *s1)
{
s08 retS = FALSE;

	if ((!strcmp_P(strupr(s0), PSTR("SERVICE")))) {
		systemVars.wrkMode = WK_SERVICE;
		retS = TRUE;
		goto quit;
	}

	if ((!strcmp_P(strupr(s0), PSTR("MONITOR")))) {

		if ((!strcmp_P( strupr(s1), PSTR("SQE")))) {
			systemVars.wrkMode = WK_MONITOR_SQE;
			retS = TRUE;
			goto quit;
		}

		if ((!strcmp_P( strupr(s1), PSTR("FRAME")))) {
			systemVars.wrkMode = WK_MONITOR_FRAME;
			retS = TRUE;
			goto quit;
		}
	}

quit:

	if ( retS ) {
		// tk_aIn: Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkAIn, TKA_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}

	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrDebugLevel(u08 *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("+BASIC")))) {
		systemVars.debugLevel += D_BASIC;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-BASIC")))) {
		if ( ( systemVars.debugLevel & D_BASIC) != 0 ) {
			systemVars.debugLevel -= D_BASIC;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DATA")))) {
		systemVars.debugLevel += D_DATA;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DATA")))) {
		if ( ( systemVars.debugLevel & D_DATA) != 0 ) {
			systemVars.debugLevel -= D_DATA;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+MEM")))) {
		systemVars.debugLevel += D_MEM;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-MEM")))) {
		if ( ( systemVars.debugLevel & D_MEM) != 0 ) {
			systemVars.debugLevel -= D_MEM;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+EVENTOS")))) {
		systemVars.debugLevel += D_EVENTOS;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-EVENTOS")))) {
		if ( ( systemVars.debugLevel & D_EVENTOS) != 0 ) {
			systemVars.debugLevel -= D_EVENTOS;
			return(TRUE);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("+GPRS")))) {
		systemVars.debugLevel += D_GPRS;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-GPRS")))) {
		if ( ( systemVars.debugLevel & D_GPRS) != 0 ) {
			systemVars.debugLevel -= D_GPRS;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DIGITAL")))) {
		systemVars.debugLevel += D_DIGITAL;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DIGITAL")))) {
		if ( ( systemVars.debugLevel & D_DIGITAL) != 0 ) {
			systemVars.debugLevel -= D_DIGITAL;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("ALL")))) {
		systemVars.debugLevel = D_DATA + D_GPRS + D_MEM + D_DIGITAL + D_EVENTOS;
		return(TRUE);
	}

	return(FALSE);
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdRTC(void)
{
RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 pos;

	retS = RTC_read(&rtcDateTime);
	if (retS ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n"));
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdDCD(void)
{
u08 pin;
s08 retS = FALSE;
u08 pos;

	retS = MCP_queryDcd(&pin);
	if (retS ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n"));
		if ( pin == 1 ) { pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos),PSTR("DCD ON(1)\r\n\0")); }
		if ( pin == 0 ) { pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos),PSTR("DCD OFF(0)\r\n\0")); }
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdEE(void)
{
	// read ee address length
	// address: argv[2]
	// length: argv[3]

s08 retS = FALSE;

	memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	retS = EE_read( (u16)(atoi(argv[2])), cmd_printfBuff, atoi(argv[3]) );
	if ( retS ) {
		// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
		snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdADC(void)
{
	// read adc channel
	// channel: argv[2]

s08 retS = FALSE;
u16 adcRetValue;

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
/*------------------------------------------------------------------------------------*/
void pv_cmdRdMCP(void)
{
	// read mcp 0|1|2 addr
	// mcpDevide: argv[2]
	// devAddress: argv[3]

s08 retS = FALSE;
u08 regValue;

	switch( atoi(argv[2] )) {
	case 0:
		retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
		break;
	case 1:
		retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
		break;
	}
	if (retS ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdDIN(void)
{
s08 retS = FALSE;
u08 pin;

	switch( atoi(argv[2] )) {
	case 0:
		retS = MCP_queryDin0(&pin);
		break;
	case 1:
		retS = MCP_queryDin1(&pin);
		break;
	}
	if (retS ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\nDIN%d=%d\r\n\0"),atoi(argv[2]),pin);
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdTERMSW(void)
{
u08 pin;

	pin = ( TERMSW_PIN & _BV(TERMSW_BIT) ) >> TERMSW_BIT;
	if ( pin == 1 ) { snprintf_P( cmd_printfBuff,CHAR128,PSTR("OK\r\nTERMSW ON\r\n\0")); }
	if ( pin == 0 ) { snprintf_P( cmd_printfBuff,CHAR128,PSTR("OK\r\nTERMSW OFF\r\n\0")); }
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------

static u08 pv_makeArgv(void)
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
static void pv_readMemory(void)
{
StatBuffer_t pxFFStatBuffer;
frameData_t Aframe;
size_t bRead;
u08 pos, channel;

	FF_seek();
	while(1) {
		bRead = FF_fread( &Aframe, sizeof(Aframe));
		if ( bRead != sizeof(Aframe))
			break;

		// imprimo
		FF_stat(&pxFFStatBuffer);
		pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("frame::{" ));
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

		for ( channel = 0; channel < NRO_CHANNELS; channel++) {
			pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%.2f,"),systemVars.aChName[channel],Aframe.analogIn[channel] );
		}
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%sP=%.2f,%sL=%d,"), systemVars.dChName[0],Aframe.dIn.pulses[0],systemVars.dChName[0],Aframe.dIn.level[0]);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%sP=%.2f,%sL=%d"), systemVars.dChName[1],Aframe.dIn.pulses[1],systemVars.dChName[1],Aframe.dIn.level[1]);
		// Bateria
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR(",bt=%.2f}\r\n\0"),Aframe.batt );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
/*------------------------------------------------------------------------------------*/
