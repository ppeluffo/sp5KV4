/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include "sp5KV4.h"

static char ctl_printfBuff[CHAR128];

void pv_ledsInit(void);
void pv_flashLeds(void);
void pv_wdgInit(void);
void pv_checkWdg(void );
void pv_checkTerminal(void);

TimerHandle_t terminalTimer;
void  pv_terminalTimerCallBack( TimerHandle_t pxTimer );

TimerHandle_t oneMinTimer;
void  pv_1MinTimerCallBack( TimerHandle_t pxTimer );

s08 f_terminalPrendida;
s08 f_terminalCallback;

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime;
const TickType_t sleepTime = ( 1000 / portTICK_RATE_MS );
u16 ffRcd;
StatBuffer_t pxFFStatBuffer;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init();			// Esto prende la terminal.
	pv_ledsInit();
	pv_wdgInit();
	// inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// load systemVars
	if  ( u_loadSystemParams() == TRUE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Arranco el timer que va a resetear a diario el dlg
	if ( xTimerStart( oneMinTimer, 0 ) != pdPASS )
		u_panic(P_CTL_TIMERSTART);

	f_terminalPrendida = TRUE; 	// Se prende al inicializar el MCP.
	f_terminalCallback = FALSE;
	// Arranco el timer de control  de la terminal
	if ( xTimerStart( terminalTimer, 0 ) != pdPASS )
		u_panic(P_CTL_TIMERSTART);

	// Inicializo el watchdog del micro.
	wdt_enable(WDTO_8S);
//	cli();
	wdt_reset();
//	WDTCSR |= (1<<WDCE) | (1<<WDE);
//	WDTCSR = 0x61;
//	sei();

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Pongo en modo sleep los DRV de las valvulas ya que en esta version no tengo
	// tkOutputs.
	//MCP_outputsReset();
	MCP_outputsSleep();

	// Habilito arrancar las otras tareas
	startTask = TRUE;

	// Loop
	for( ;; )
	{

		u_clearWdg(WDG_CTL);

		// Ejecuto la rutina sincronicamente c/1s
		// Wait for 1000ms.
		vTaskDelayUntil( &xLastWakeTime, sleepTime );
		xLastWakeTime = xTaskGetTickCount();

		pv_flashLeds();
		pv_checkWdg();
		pv_checkTerminal();

	}
}
//------------------------------------------------------------------------------------
void tkControlInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Creo el timer de la terminal y lo arranco
	terminalTimer = xTimerCreate (  "TERM_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 120000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdFALSE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_terminalTimerCallBack
	                   );

	if ( terminalTimer == NULL )
		u_panic(P_CTL_TIMERCREATE);

	oneMinTimer = xTimerCreate (  "1MIN_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 60000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_1MinTimerCallBack
	                   );

	if ( oneMinTimer == NULL )
		u_panic(P_CTL_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_ledsInit(void)
{

	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);

}
//------------------------------------------------------------------------------------
void pv_flashLeds(void)
{
	// Ejecuto c/2 secs.
	// Prendo los leds por 20ms y los apago.

static u08 l_timer = 1;

	if (l_timer-- > 0 )
		return;

	l_timer = 1;


	// Prendo: solo si hay una terminal conectada
	if ( f_terminalPrendida ) {
		MCP_setLed_LogicBoard(1);		// Led placa logica
		cbi(LED_KA_PORT, LED_KA_BIT);	// Led placa analogica ( kalive )
		// ModemLed placa superior.
		if (u_modemPwrStatus() == PRENDIDO )
			cbi(LED_MODEM_PORT, LED_MODEM_BIT);
	}

	// no es necesario ya que lo que demora las MCP son suficientes.
	//vTaskDelay( 1 );

	// Apago
	MCP_setLed_LogicBoard(0);			// Led placa logica
	sbi(LED_KA_PORT, LED_KA_BIT);		// Led placa analogica ( kalive )
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);

}
//------------------------------------------------------------------------------------
void pv_wdgInit(void)
{
u08 pos;

	systemWdg = WDG_CTL + WDG_CMD + WDG_DIN + WDG_AIN + WDG_GPRS;
	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Watchdog init (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
}
//------------------------------------------------------------------------------------
void pv_checkWdg(void )
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 2;

	if (l_timer-- > 0 )
		return;

	l_timer = 1;
	if ( systemWdg == 0 ) {
		wdt_reset();
		systemWdg = WDG_CTL + WDG_CMD + WDG_CSG + WDG_DIN + WDG_AIN + WDG_GPRS + WDG_GPRSRX;
	}
}
//------------------------------------------------------------------------------------
void  pv_1MinTimerCallBack( TimerHandle_t pxTimer )
{
	// Funcion de callback que se invoca 1 vez por minuto.
	// Aqui controlo todas las cosas que quiero con 1m de granularidad del timer.

	// Daily reset: una vez por dia me debo resetear ( 1440 mins )
	// Si estoy en modo service, a los 30 mins debo resetearme.


static s16 resetCounter = T_DAILYRESET;
static s16 serviceModeCounter = T_EXITSERVICEMODE;

	// Una vez por dia me reseteo.
	if ( resetCounter-- <= 0 ) {

		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Going to daily reset..\r\n\0"));
		FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

		wdt_enable(WDTO_30MS);
		while(1) {}
	}

	// En modo service, a los 30 mins. me reseteo para salir solo
	if ( systemVars.wrkMode != WK_NORMAL ) {

		if ( serviceModeCounter-- <= 0 ) {

			snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Automatic exit of service mode..\r\n\0"));
			FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

			wdt_enable(WDTO_30MS);
			while(1) {}
		}

	}
}
//------------------------------------------------------------------------------------
void pv_checkTerminal(void)
{

static u08 l_termsw = 1; // Estado anterior del pin de la terminal

	// Si estoy en modo CONTINUO no apago la terminal ni dejo de flashear los leds
	if ( systemVars.pwrMode == PWR_CONTINUO) {

		// Si la terminal esta apagada la prendo
		if ( ! f_terminalPrendida ) {
			// prendo
			MCP_setTermPwr(1);
			f_terminalPrendida = TRUE;
			// y rearranco el timer
			xTimerReset( terminalTimer, 1 );
			f_terminalCallback = FALSE;
		}

		// Si expiro el timer lo rearranco.
		if ( f_terminalCallback ) {
			while ( xTimerReset( terminalTimer, 1 ) != pdPASS )
				taskYIELD();
			f_terminalCallback = FALSE;
		}

		goto quit;
	}

	if ( systemVars.pwrMode == PWR_DISCRETO) {

		// Si la terminal esta apagada y se activo el switch la prendo
		if ( f_terminalPrendida == FALSE ) {

			// Si el pin esta en 1 y antes estaba en 0 ( Indica que llego un flanco positivo )
			if ( ( systemVars.termsw == 1 ) && ( l_termsw == 0 ) ) {
				// prendo
				MCP_setTermPwr(1);
				f_terminalPrendida = TRUE;
				// y rearranco el timer
				xTimerReset( terminalTimer, 1 );
				f_terminalCallback = FALSE;
				snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Terminal going on ..\r\n\0"));
				vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
				FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
			}
			goto quit;
		}

		// Si la terminal estaba prendida y expiro el timer...
		if ( f_terminalPrendida == TRUE ) {

			if ( !f_terminalCallback ) {
				goto quit;
			}

			// Expiro el timer:
			// Caso 1: el switch esta activo: reinicio el timer
			if ( systemVars.termsw == 1 ) {
				while ( xTimerReset( terminalTimer, 1 ) != pdPASS )
					taskYIELD();
				f_terminalCallback = FALSE;
			} else {
				// Apago
				snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Terminal going off ..\r\n\0"));
				FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
				vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
				MCP_setTermPwr(0);
				f_terminalPrendida = FALSE;
			}
		}
	}

quit:

// Guardo el estado.
	l_termsw = systemVars.termsw;
	return;


}
//------------------------------------------------------------------------------------
void  pv_terminalTimerCallBack( TimerHandle_t pxTimer )
{
	// Luego de haber arrancado el FREERTOS, a los 2 mins. expira este timer.
	// y debo ver si apago la terminal o la dejo prendida.
	f_terminalCallback = TRUE;
}
//------------------------------------------------------------------------------------
s08 u_terminalPwrStatus(void)
{
	return(f_terminalPrendida);
}
//------------------------------------------------------------------------------------
void u_restartTimerTerminal(void)
{
	xTimerReset( terminalTimer, 1 );
}
//------------------------------------------------------------------------------------
