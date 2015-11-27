#include "sp5KV4.h"

static char cons_printfBuff[CHAR256];

u08 lineNbr;

void testConsignasTask(void);

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

	lineNbr = 0;

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
			}

		}

		// Una vez c/30s genero un mensaje
		testConsignasTask();


	}

}
// ------------------------------------------------------------------------------------
void testConsignasTask(void)
{
static u16 cTimer = 300;

	cTimer--;
	if (cTimer == 0 ) {
		cTimer = 300;

		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s DEBUG Testing tkConsignas( %d )\r\n\0"), u_now(), lineNbr );
		//FreeRTOS_write( &pdUART1, cons_printfBuff, sizeof(cons_printfBuff) );

		lineNbr++;

	}
}
// ------------------------------------------------------------------------------------

