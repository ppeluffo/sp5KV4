#include "sp5KV4.h"

static char cons_printfBuff[CHAR256];

void pv_checkConsignas(void);
void pv_configConsignaInicial ( void );

//--------------------------------------------------------------------------------------
 void tkConsignas(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("starting tkConsignas..\r\n\0"));
	FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );

	MCP_outputA1Disable();
	MCP_outputA2Disable();
	MCP_outputB1Disable();
	MCP_outputB2Disable();
	MCP_outputsSleep();
	MCP_outputsNoReset();

	pv_configConsignaInicial();

	//
	for( ;; )
	{

		u_clearWdg(WDG_CSG);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TKC_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				pv_configConsignaInicial();
			}

		}

		// Una vez c/30s genero un mensaje
		pv_checkConsignas();

	}

}
// ------------------------------------------------------------------------------------
void pv_checkConsignas(void)
{
static u16 cTimer = 300;
RtcTimeType_t rtcDateTime;
u16 now;

	cTimer--;
	if (cTimer > 0 )
		return;

	cTimer = 300;

	if ( systemVars.consigna.status == CONSIGNA_OFF )
		return;

	// Hora actual en minutos.
	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	if ( now == systemVars.consigna.horaConsDia ) {
		u_setConsignaDiurna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Diurna\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( now == systemVars.consigna.horaConsNoc ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Nocturna\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

}
// ------------------------------------------------------------------------------------
void pv_configConsignaInicial ( void )
{
	// Determino cual consigna corresponde aplicar y la aplico.
RtcTimeType_t rtcDateTime;
u16 now;

	// Hora actual en minutos.
	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	if ( systemVars.consigna.status == CONSIGNA_OFF ) {
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s RELOAD: consignas OFF\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	if ( now <= systemVars.consigna.horaConsDia ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s RELOAD:Set Consigna Nocturna\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( ( now > systemVars.consigna.horaConsDia ) && ( now <= systemVars.consigna.horaConsNoc )) {
		u_setConsignaDiurna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s RELOAD:Set Consigna Diurna\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( now > systemVars.consigna.horaConsNoc ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s RELOAD:Set Consigna Nocturna\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

}
/*------------------------------------------------------------------------------------*/
