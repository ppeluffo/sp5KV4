/*
 * Revisar porque al reinicializar el fifo, ya no veo los comandos ingresados.
 * Como afecta esto a la busqueda.
 *
 * git commit -a -m "beta 20151107 001"
 * git remote add REM_SP5KV4 https://github.com/ppeluffo/sp5KV4.git
 * git push -u REM_SP5KV4 master
 *
 * A RESOLVER:
 * 1- La primera vez que se conecta, el router indica 'destination unreachable'.
 * Para acortar los tiempos, tal vez el modem debiera en vez de espera, reintentar el comando.
 *
 * 2- Watchgog
 * Guardo el estado por el que me reseteo y lo mando en el frame de INIT.
 *
 * 3- Tiempos:
 * La causa del delay en las operaciones de memoria viene por un lado por la espera de los
 * semaforos que incorporaba 10 ticks, y por otro lado en la espera de las respuestas del I2C que
 * incorporaba otros 10 ticks a c/u.
 * Lo soluciono bajando todo a 1 tick.
 *
 *
 * ----------------------------------------------------------------------------------------------------------------
 */

#include "sp5KV4.h"

static void pv_initMPU(void);

//------------------------------------------------------------------------------------
int main(void)
{
unsigned int i,j;


	//----------------------------------------------------------------------------------------
	// Rutina NECESARIA para que al retornar de un reset por WDG no quede en infinitos resets.
	// Copiado de la hoja de datos.

	// Lo primero que hago es leer la causa del reset.

	wdgStatus.resetCause = MCUSR;

	cli();
	wdt_reset();

	//MCUSR &= ~(1<<WDRF);
	MCUSR = 0x00;
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

	pv_initMPU();
	FreeRTOS_open(pUART0, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pUART1, ( UART_RXFIFO + UART_TXQUEUE ));
	FreeRTOS_open(pI2C, 0);

	/* Arranco el RTOS. */
	startTask = FALSE;

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutex();

	// Inicializacion de modulos de las tareas que deben hacerce antes
	// de arrancar el FRTOS
	tkOutputInit();
	tkAnalogInit();
	tkGprsInit();

	// Creo las tasks
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkDigitalIn, "DIN", tkDigitalIn_STACK_SIZE, NULL, tkDigitalIn_TASK_PRIORITY,  &xHandle_tkDigitalIn);
	xTaskCreate(tkOutput, "OUT", tkOutput_STACK_SIZE, NULL, tkOutput_TASK_PRIORITY,  &xHandle_tkOutput);
	xTaskCreate(tkControl, "CTL", tkControl_STACK_SIZE, NULL, tkControl_TASK_PRIORITY,  &xHandle_tkControl);
	xTaskCreate(tkAnalogIn, "AIN", tkAIn_STACK_SIZE, NULL, tkAIn_TASK_PRIORITY,  &xHandle_tkAIn);
	xTaskCreate(tkGprs, "GPRS", tkGprs_STACK_SIZE, NULL, tkGprs_TASK_PRIORITY,  &xHandle_tkGprs);

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);
	//return 0;
}
/*------------------------------------------------------------------------------------*/
static void pv_initMPU(void)
{

}
/*------------------------------------------------------------------------------------*/
void vApplicationIdleHook( void )
{

	for(;;) {

//		vCoRoutineSchedule();

	}

}
/*------------------------------------------------------------------------------------*/
