/*
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
 * !! Agregar el salir automaticamente luego de 30 mins del modo service.
 *
 * V4.1.3:
 * - Agrego en el frame de datos un campo que indique la calidad del frame.
 *   Queda del tipo:
 *   CTL=106&ST=0&LINE=20140607,083358,pA>62.12,pB>62.12,pc>4.23,q0>103.28,v1>22.4,bt>11.29
 *
 * V4.1.2:
 * - Modifico la inicializacion de WDT siguiendo las recomendaciones de Atmel y verifico el
 *   funcionamiento de c/wdg.
 *   http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
 *   http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 * - El WDT se prende por fuses grabandolos en 0xFF(low), 0xC9(high), 0xFD(extended)
 *   Esto me asegura que no se van a borrar por hw.
 * - Borro todo lo referente a DCD ya que no lo uso.
 * - Agrego a tkAnalog una funcion que chequea la consistencia de los timers en c/ciclo y si estan
 *   mal resetea al uC
 * - Idem en tkGprs.
 * - Tambien controlo no exceder 12hs sin discar.
 * - Elimino el control del pin de la terminal de tkDigital y lo dejo todo en tkControl.
 * - En tkControl agrego la rutina pv_modemControl para controlar que el modem no quede
 *   prendido mas de 10mins. Con esto si alguna rutina queda en loop, no apagaria el modem y
 *   podriamos agarrar el problema.
 *
 * V4.0.9:
 * El problema es que el ADC no se prende y la lectura de los canales es erronea.
 * La razon es que el MCP se resetea a default y por lo tanto no prende los sensores ni los 3.3V.
 * Esto es porque el pigtail del modem irradia energia e introduce un pulso que resetea al MCP.
 * Lo resolvemos en 3 flancos diferentes:
 * 1- En la rutina MCP_testAndSet ( con la que prendemos los 3.3V ), verificamos que el MCP este configurado
 * y sino lo reconfiguramos.
 * 2- Hay veces que esto no es suficiente ya que entre tr02 y tr06, si se resetea el MCP no nos damos
 * cuenta hasta que leemos el ADC.
 * Entonces en tr02 mandamos un mensaje a tkGPRS que no trasmita, y en tr06 que puede trasmitir.
 * Con esto controlo no irradiar energia que resetee al MCP mientras estoy poleando ( 15s ).
 * En tkGPRS lo chequeo antes de perdir una IP y antes de abrir un socket.
 * 3- En caso que nada funcione, en el modulo tkAnalog si una medida me da error ( lectura del ADC ), descarto
 * la medida y la sustituyo por el valor anterior.
 * Como la medida de los sensores se hace en 4 pasos, si el primero ( tr05) da error ( por estar apagado el ADC ),
 * puedo corregirlo y prenderlo.
 *
 * V4.1.0:
 * Modifico el manejo de consignas para incorporar una FSM
 * El mismo problema de desconfiguracion puede ocurrir al setear las consignas.
 * Para esto entonces fijo que para setear las consignas el modem deba estar apagado.
 * La tarea tkConsignas consulta el estado del modem con u_modemPwrStatus()
 *
 * ----------------------------------------------------------------------------------------------------------------
 */

#include "sp5KV4.h"

static void pv_initMPU(void);

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
	// Leo el MCUSR para saber cual fue la causa del reset
	mcusr_mirror = MCUSR;
	// e inmediatamente lo pongo en 0 por si se resetea saber la causa
    MCUSR = 0;
    // y deshabilito el wdg para dejar que el micro arranque y no quede en un loop de resets
    //  wdt_disable();
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
    wdt_enable(WDTO_8S);
    return;
}
//------------------------------------------------------------------------------------

int main(void)
{
unsigned int i,j;

	//----------------------------------------------------------------------------------------
	// Rutina NECESARIA para que al retornar de un reset por WDG no quede en infinitos resets.
	// Copiado de la hoja de datos.

	// Lo primero que hago es leer la causa del reset para luego trasmitirla en un init.
	// Inicializamos el watchdog
//	cli();
	// Leo el MCUSR para saber cual fue la causa del reset
//	wdgStatus.resetCause = MCUSR;
	// e inmediatamente lo pongo en 0 por si se resetea saber la causa
//	MCUSR = 0;
	// y deshabilito el wdg para dejar que el micro arranque y no quede en un loop de resets
//	wdt_disable();
//	wdt_reset();
//	sei();

	wdgStatus.resetCause = mcusr_mirror;
	// Genero un delay de 1s. para permitir que el micro se estabilize.
	// Lleva tiempo a los osciladores estabilizarse.
	for (i=0; i<1000; i++)
		for (j=0; j<1000; j++)
				;

	//----------------------------------------------------------------------------------------

	wdt_reset();
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
	xTaskCreate(tkGprsRx, "GPRX", tkGprsRx_STACK_SIZE, NULL, tkGprsRx_TASK_PRIORITY,  &xHandle_tkGprsRx);

	systemWdg = WDG_CTL + WDG_CMD + WDG_CSG + WDG_DIN + WDG_AIN + WDG_GPRS + WDG_GPRSRX;

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

	// Configuracion de pines:
	// Los pines del micro que resetean los latches de caudal son salidas.
	sbi(Q_DDR, Q0_CTL_PIN);
	sbi(Q_DDR, Q1_CTL_PIN);

	// El pin de control de la terminal es entrada
	cbi(TERMSW_DDR, TERMSW_BIT);

	// El pin de DCD es entrada
	cbi(DCD_DDR, DCD_BIT);

	// Leds
	sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	sbi(LED_MODEM_DDR, LED_MODEM_BIT);	// El pin del led de KA ( PD6 ) es una salida.
	// inicialmente los led quedan en 0
	sbi(LED_KA_PORT, LED_KA_BIT);
	sbi(LED_MODEM_PORT, LED_MODEM_BIT);

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
