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
void pv_dailyReset(void);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime;
const TickType_t sleepTime = ( 1000 / portTICK_RATE_MS );
u16 ffRcd;
StatBuffer_t pxFFStatBuffer;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init();
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

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Habilito arrancar las otras tareas
	startTask = TRUE;

	// Inicializo el watchdog del micro.
	wdt_enable(WDTO_8S);
	wdt_reset();

	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount();

	// Loop
	for( ;; )
	{

		u_clearWdg(WDG_CTL);

		// Ejecuto la rutina sincronicamente c/2s
		// Wait for 1000ms.
		vTaskDelayUntil( &xLastWakeTime, sleepTime );
		xLastWakeTime = xTaskGetTickCount();

		pv_flashLeds();
		pv_checkWdg();
		pv_dailyReset();

	}
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
	// Prendo
	MCP_setLed_LogicBoard(1);		// Led placa logica
	cbi(LED_KA_PORT, LED_KA_BIT);	// Led placa analogica ( kalive )
	// ModemLed placa superior.
	if (u_modemPrendido() )
		cbi(LED_MODEM_PORT, LED_MODEM_BIT);

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

	systemWdg = WDG_CTL + WDG_CMD + WDG_DIN + WDG_OUT + WDG_AIN + WDG_GPRS;
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
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/2s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 1;

	if (l_timer-- > 0 )
		return;

	l_timer = 1;
	if ( systemWdg == 0 ) {
		wdt_reset();
		systemWdg = WDG_CTL + WDG_CMD + WDG_DIN + WDG_OUT + WDG_AIN + WDG_GPRS;
	}
}
//------------------------------------------------------------------------------------
void pv_dailyReset(void)
{
	// Una vez por dia el equipo se debe resetear.
	// Esto me asegura que en caso que pase algo no considerado, debería corregirse.

const u32 RESET_SECS = 86400;	// 24hs*60m*60s Segundos en 24hs.
static u32 resetCounter = 0;

	resetCounter += 1;
	if ( resetCounter > RESET_SECS ) {
		wdt_enable(WDTO_30MS);
		while(1) {}
	}
}
//------------------------------------------------------------------------------------
