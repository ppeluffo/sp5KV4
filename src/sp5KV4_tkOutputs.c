/*
 * sp5KV3_tkOutputs.c
 *
 *  Created on: 8/7/2015
 *      Author: pablo
 */


#include "sp5KV4.h"

#define CICLOSEN30S	300

static char out_printfBuff[CHAR128];
TimerHandle_t consignaTimer;

static void pv_setConsignaInicial ( void );
static void pv_checkAndSetConsigna(void);
void pv_consignaTimerCallback( TimerHandle_t pxTimer );

static RtcTimeType_t rtcDateTime;
static u16 now;
static s08 f_consignaCallBack;

#define set_f_consignaCallBack() ( f_consignaCallBack = TRUE )
#define reset_f_consignaCallBack() ( f_consignaCallBack = FALSE )
//------------------------------------------------------------------------------------

void tkOutput(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("starting tkOutputs..\r\n\0"));
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );

	// Configuracion INICIAL de las salidas
	pv_setConsignaInicial();
	f_consignaCallBack = FALSE;

	// Arranco el timer de consignas.
	if ( xTimerStart( consignaTimer, 0 ) != pdPASS )
		u_panic(P_OUT_TIMERSTART);

	// Loop
	for( ;; )
	{
		u_clearWdg(WDG_OUT);
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		if ( f_consignaCallBack ) {
			reset_f_consignaCallBack();
			pv_checkAndSetConsigna();
		}
	}
}
//------------------------------------------------------------------------------------
void tkOutputInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/30secs

	consignaTimer = xTimerCreate (  "CONS_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 30000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_consignaTimerCallback
	                   );

	if ( consignaTimer == NULL )
		u_panic(P_OUT_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_consignaTimerCallback( TimerHandle_t pxTimer )
{
	set_f_consignaCallBack();

}
//------------------------------------------------------------------------------------
static void pv_setConsignaInicial ( void )
{
	// Determino cual consigna corresponde aplicar y la aplico.
RtcTimeType_t rtcDateTime;
u16 now;


	if ( systemVars.consigna.status == CONSIGNA_OFF ) {
		return;
	}

	// Hora actual en minutos.
	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	if ( now <= systemVars.consigna.horaConsDia ) {
		u_setConsignaNocturna(CONSIGNA_PULSE_MS);
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("Consigna NOCTURNA..\r\n\0"));
		goto quit;
	}

	if ( ( now > systemVars.consigna.horaConsDia ) && ( now <= systemVars.consigna.horaConsNoc )) {
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("Consigna DIURNA..\r\n\0"));
		goto quit;
	}

	if ( now > systemVars.consigna.horaConsNoc ) {
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("Consigna NOCTURNA..\r\n\0"));
		goto quit;
	}

quit:
	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------
static void pv_checkAndSetConsigna(void)
{
RtcTimeType_t rtcDateTime;
u16 now;
//u32 tickCount;

	if ( systemVars.consigna.status == CONSIGNA_OFF ) {
		return;
	}

	// Hora actual en minutos.
	// Para la consigna solo importa la hora del dia ya que se aplica todos los dias
	// Convertimos la hora en minutos desde las 00:00

	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	if ( now == systemVars.consigna.horaConsDia ) {
		u_setConsignaDiurna(CONSIGNA_PULSE_MS);
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("Consigna DIURNA..\r\n\0"));
		goto quit;
	}

	if ( now == systemVars.consigna.horaConsNoc ) {
		u_setConsignaNocturna(CONSIGNA_PULSE_MS);
		snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR("Consigna NOCTURNA..\r\n\0"));
		goto quit;
	}

quit:
//	tickCount = xTaskGetTickCount();
//	snprintf_P( out_printfBuff,sizeof(out_printfBuff),PSTR(".[%06lu] Check consignas\r\n\0"), tickCount );
//	FreeRTOS_write( &pdUART1, out_printfBuff, sizeof(out_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------
