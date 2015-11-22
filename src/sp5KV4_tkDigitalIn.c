/*
 * sp5KV3_tkDigitalIn.c
 *
 *  Created on: 13/4/2015
 *      Author: pablo
 *
 *  La nueva modalidad es por poleo.
 *  Configuro el MCP para que no interrumpa
 *  C/100ms leo el registro GPIO del MCP.
 *  En reposo la salida de los latch es 1 por lo que debo detectar cuando se hizo 0.
 *  Para evitar poder quedar colgado, c/ciclo borro el latch.
 *  Esto implica que no importa la duracion del pulso ya que lo capturo con un flip-flop, pero
 *  no pueden venir mas rapido que 10/s.
 *
 *	Esta version solo puede usarse con placas SP5K_3CH que tengan latch para los pulsos, o sea
 *	version >= R003.
 *
 */


#include "sp5KV4.h"

static void pv_clearQ(void);
static void pv_pollQ(void);
static void pv_pollDcd(void);

static char dIn_printfBuff[CHAR64];	// Buffer de impresion
static dinData_t digIn;				// Estructura local donde cuento los pulsos.
static struct {
	s08 serviceMode;
	s08 msgReload;
} D_flags;

/*------------------------------------------------------------------------------------*/
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	// Los pines del micro que resetean los latches de caudal son salidas.
	sbi(Q_DDR, Q0_CTL_PIN);
	sbi(Q_DDR, Q1_CTL_PIN);

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();
	digIn.level[0] = 0;
	digIn.level[1] = 0;
	digIn.pulses[0] = 0;
	digIn.pulses[1] = 0;

	// Inicializo el valor del dcd.
	MCP_queryDcd(&systemVars.dcd);

	D_flags.msgReload = FALSE;
	D_flags.serviceMode = FALSE;

	for( ;; )
	{
		u_clearWdg(WDG_DIN);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {
			if ( ( ulNotifiedValue & TKD_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				D_flags.msgReload = TRUE;
			}
		}

		// Mensaje de cabio de configuracion. Releo la misma
		if ( D_flags.msgReload ) {
			D_flags.msgReload = FALSE;
			if ( systemVars.wrkMode != WK_NORMAL ) {
				D_flags.serviceMode = TRUE;
			}
		}

		// Solo poleo las entradas en modo normal. En modo service no para
		// poder manejarlas por los comandos de servicio.
		if ( ! D_flags.serviceMode) {
			pv_pollDcd();
			pv_pollQ();
		}
	}

}
/*------------------------------------------------------------------------------------*/
void u_readDigitalCounters( dinData_t *dIn , s08 resetCounters )
{
	// copio los valores de los contadores en la estructura dIn.
	// Si se solicita, luego se ponen a 0.

	memcpy( dIn, &digIn, sizeof(dinData_t)) ;
	if ( resetCounters == TRUE ) {
		digIn.level[0] = 0;
		digIn.level[1] = 0;
		digIn.pulses[0] = 0;
		digIn.pulses[1] = 0;
	}
}
/*------------------------------------------------------------------------------------*/
static void pv_pollDcd(void)
{
s08 retS = FALSE;
u08 pin;
u32 tickCount;

	retS = MCP_queryDcd(&pin);
	// Solo indico los cambios.
	if ( systemVars.dcd != pin ) {
		systemVars.dcd = pin;
		tickCount = xTaskGetTickCount();
		if ( pin == 1 ) { snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] tkDigitalIn: DCD off(%d)\r\n\0"), tickCount,pin );	}
		if ( pin == 0 ) { snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] tkDigitalIn: DCD on(%d)\r\n\0"), tickCount,pin );	}

		if ( (systemVars.debugLevel & D_DIGITAL) != 0) {
			FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
		}
	}
}
/*------------------------------------------------------------------------------------*/
static void pv_pollQ(void)
{

s08 retS = FALSE;
u08 din0 = 0;
u08 din1 = 0;
s08 debugQ = FALSE;
u32 tickCount;

	// Leo el GPIO.
	retS = MCP_query2Din( &din0, &din1 );
	if ( retS ) {
		// Levels
		digIn.level[0] = din0;
		digIn.level[1] = din1;

		// Counts
		debugQ = FALSE;
		if (din0 == 0 ) { digIn.pulses[0]++ ; debugQ = TRUE;}
		if (din1 == 0 ) { digIn.pulses[1]++ ; debugQ = TRUE;}
	} else {
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("tkDigitalIn: READ DIN ERROR !!\r\n\0"));
		if ( (systemVars.debugLevel & D_DIGITAL) != 0 ) {
			FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
		}
		goto quit;
	}

	if ( ((systemVars.debugLevel & D_DIGITAL) != 0) && debugQ ) {
		tickCount = xTaskGetTickCount();
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] tkDigitalIn: din0=%.0f,din1=%.0f\r\n\0"), tickCount, digIn.pulses[0],digIn.pulses[1] );
		FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );
	}

quit:
	// Siempre borro los latches para evitar la posibilidad de quedar colgado.
	pv_clearQ();
	return;

}
/*------------------------------------------------------------------------------------*/
static void pv_clearQ(void)
{
	// Pongo un pulso 1->0->1 en Q0/Q1 pin para resetear el latch
	// En reposo debe quedar en H.
	cbi(Q_PORT, Q0_CTL_PIN);
	cbi(Q_PORT, Q1_CTL_PIN);
	taskYIELD();
	//_delay_us(5);
	//asm("nop");
	sbi(Q_PORT, Q0_CTL_PIN);
	sbi(Q_PORT, Q1_CTL_PIN);
}
/*------------------------------------------------------------------------------------*/
