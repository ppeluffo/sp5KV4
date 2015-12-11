/*
 * Revisar porque al reinicializar el fifo, ya no veo los comandos ingresados.
 * Como afecta esto a la busqueda.
 *
 * git commit -a -m "beta 473293390 001"
 * git remote add REM_SP5KV4 https://github.com/ppeluffo/sp5KV4.git
 * git push -u REM_SP5KV4 master
 *
 * WATCHDOG:
 * Para hacer un mejor seguimiento de las fallas, agrego a c/estado un nro.
 * Por otro lado, el WDG lo manejo en modo interrupcion / reset de modo que ante
 * un problema, la interrupcion guarda el estado y luego se resetea.
 * Al arrancar, leo el estado y lo trasmito.
 *
 * Agrego en la funcion de espera de I2C un timeout de modo que salgo con FALSE y eso hace
 * que el resto de las funciones indique un error.
 * Esto en ppio. podria evitar un error de reset por wdg.
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

	// Lo primero que hago es leer la causa del reset para luego trasmitirla en un init.
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

	// Leo el estado anterior al reset.
//	eeprom_read_block((u08 *)&wdgStatusEE, (uint8_t *) EEADDR_WDG, sizeof( wdgStatus ));
	// y borro la flag.
//	eeprom_write_byte( (uint8_t *)( EEADDR_WDG + sizeof( wdgStatus ) - 1), 0 );

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
	tkControlInit();
	tkAnalogInit();
	tkGprsInit();

	// Creo las tasks
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkDigitalIn, "DIN", tkDigitalIn_STACK_SIZE, NULL, tkDigitalIn_TASK_PRIORITY,  &xHandle_tkDigitalIn);
	xTaskCreate(tkControl, "CTL", tkControl_STACK_SIZE, NULL, tkControl_TASK_PRIORITY,  &xHandle_tkControl);
	xTaskCreate(tkAnalogIn, "AIN", tkAIn_STACK_SIZE, NULL, tkAIn_TASK_PRIORITY,  &xHandle_tkAIn);
	xTaskCreate(tkGprs, "GPRS", tkGprs_STACK_SIZE, NULL, tkGprs_TASK_PRIORITY,  &xHandle_tkGprs);
	xTaskCreate(tkConsignas, "CONS", tkCons_STACK_SIZE, NULL, tkCons_TASK_PRIORITY,  &xHandle_tkConsignas);

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);
	//return 0;
}
/*------------------------------------------------------------------------------------*/
static void pv_initMPU(void)
{
	// Son acciones que se hacen antes de arrancar el RTOS

	// Leo la configuracion de la EEprom interna

	// Configuro el modo de Sleep.
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
}
/*------------------------------------------------------------------------------------*/
void vApplicationIdleHook( void )
{

	for(;;) {

//		vCoRoutineSchedule();
		if ( ( u_modemPwrStatus() == APAGADO) && (u_terminalPwrStatus() == FALSE ) && ( systemVars.pwrMode == PWR_DISCRETO)) {
			sleep_mode();
		}
	}

}
/*------------------------------------------------------------------------------------*/
//ISR( WDT_vect )
//{
/* Handler (ISR) de WDG.
 * Guarda el estado del sistema para que al reiniciarse se mande por INIT frame
*/

//	 wdgStatus.mcusr = MCUSR;
//	 wdgStatus.securityFlag = 'A';

	 // Salvo el estado
//	 eeprom_write_block( (u08 *)&wdgStatus, (uint8_t *) EEADDR_WDG, sizeof( wdgStatus ));
//	 // Y me reseteo.
//	 wdt_enable(WDTO_1S);
//	 while(1) {}
//}

/*------------------------------------------------------------------------------------*/
