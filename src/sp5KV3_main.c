/*
 * Revisar porque al reinicializar el fifo, ya no veo los comandos ingresados.
 * Como afecta esto a la busqueda.
 *
 *
 *
 * ----------------------------------------------------------------------------------------------------------------
 */

#include "sp5KV3.h"

static void initMPU(void);

//------------------------------------------------------------------------------------
int main(void)
{
unsigned int i,j;


	//----------------------------------------------------------------------------------------
	// Rutina NECESARIA para que al retornar de un reset por WDG no quede en infinitos resets.
	// Copiado de la hoja de datos.
	cli();
	wdt_reset();
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();

	// Genero un delay de 1s. para permitir que el micro se estabilize.
	// Lleva tiempo a los osciladores estabilizarse.
	for (i=0; i<1000; i++)
		for (j=0; j<1000; j++)
				;

	//----------------------------------------------------------------------------------------

	initMPU();
	//FreeRTOS_open(pUART0,0);
	FreeRTOS_open(pUART1, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pI2C, 0);

	/* Creo las tareas */
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	return 0;
}
/*------------------------------------------------------------------------------------*/
static void initMPU(void)
{
	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);
}
/*------------------------------------------------------------------------------------*/
void vApplicationIdleHook( void )
{

	for(;;) {

//		vCoRoutineSchedule();

	}

}
/*------------------------------------------------------------------------------------*/
