/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */


#include "sp5KV4.h"

static char aIn_printfBuff[CHAR256];
TimerHandle_t pollingTimer;

static u08 rdErrors;

// Estados
typedef enum {	tkdST_INIT 		= 0,
				tkdST_STANDBY	= 1,
				tkdST_PWRSETTLE	= 2,
				tkdST_POLLING	= 3
} t_tkData_state;

// Eventos
typedef enum {
	evINIT = 0, 			// Init
	evRELOADCONFIG,			// EV_MSGreload
	evSTART2POLL,			// EV_f_start2poll
	evSEC2PWRSETTLE_NOT_0,	// EV_counter_secs2pwrSettleNOT_0
	evPOLLCNT_NOT_0			// EV_pollCounterNOT_0

} t_tkData_eventos;

#define dEVENT_COUNT		5

static s08 dEventos[dEVENT_COUNT];

// transiciones
static int trD00(void);
static int trD01(void);
static int trD02(void);
static int trD03(void);
static int trD04(void);
static int trD05(void);
static int trD06(void);

static u08 tkAIN_state = tkdST_INIT;	// Estado
static u32 tickCount;					// para usar en los mensajes del debug.
static double rAIn[NRO_CHANNELS + 1];	// Almaceno los datos de conversor A/D
static frameData_t Aframe;
static s08 f_skipFrame;					// indico si el frame leido debe ser descartado ( MCP error )

static struct {
	s08 starting;			// flag que estoy arrancando
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 firstPoll;			// flag que voy a hacer el primer poleo
	s08 start2poll;			// flag que habilita a polear.
} AN_flags;

#define CICLOS_POLEO		3		// ciclos de poleo para promediar.
#define SECS2PWRSETTLE 		5
#define MAX_ERRORS			90		// maximo nro.poleos con error para resetear el micro.( equivale a 30 mins. )

static struct {
	u08 secs2pwrSettle;		// contador de segundos para que se estabilize la fuente
	u16 secs2poll;			// contador de segundos hasta el siguiente poleo ( habilitacion de la flag )
	u08 nroPoleos;			// Contador del nro de poleos
} AN_counters;

// Funciones generales
static void pv_AINgetNextEvent(void);
static void pv_AINfsm(void);
static void pv_AINprintExitMsg(u08 code);
static void pv_AinLoadParameters( void );
void pv_pollTimerCallback( TimerHandle_t pxTimer );

void catch_I2C_Error( u08 channel );

//--------------------------------------------------------------------------------------
 void tkAnalogIn(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("starting tkAnalogIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

	tkAIN_state = tkdST_INIT;		// Estado inicial.
	AN_flags.starting = TRUE;		// Evento inicial ( arranque ).
	AN_flags.msgReload = FALSE;		// No tengo ningun mensaje de reload pendiente.
	AN_flags.firstPoll = TRUE;		// Voy a hacer el primer poleo.
	AN_flags.start2poll = FALSE;

	AN_counters.nroPoleos = CICLOS_POLEO;
	AN_counters.secs2poll = 15;
	AN_counters.secs2pwrSettle = SECS2PWRSETTLE;

	rdErrors = 0;

	// Arranco el timer de poleo.
	if ( xTimerStart( pollingTimer, 0 ) != pdPASS )
		u_panic(P_AIN_TIMERSTART);

	//
	for( ;; )
	{

		wdgStatus.analogCP = 1;
		u_clearWdg(WDG_AIN);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TKA_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				AN_flags.msgReload = TRUE;
			}

			if ( ( ulNotifiedValue & TKA_READ_FRAME ) != 0 ) {
				// Mensaje de polear un frame ( estando en modo servicio )
				AN_flags.start2poll = TRUE;
			}
		}

		// Analizo los eventos.
		pv_AINgetNextEvent();
		// Corro la maquina de estados.
		pv_AINfsm();

		if ( rdErrors >= MAX_ERRORS ) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("%s Reset por I2C errors !!!\r\n\0"), u_now() );
			u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
			wdt_enable(WDTO_30MS);
			while(1) {}
		}
	}

}
/*------------------------------------------------------------------------------------*/
void tkAnalogInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/1sec

	pollingTimer = xTimerCreate (  "POLL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_pollTimerCallback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_OUT_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_pollTimerCallback( TimerHandle_t pxTimer )
{
	// Como el timer esta en reload c/1 sec, aqui contamos los secs para polear
	// en la variable secs2poll.

	wdgStatus.analogCP = 2;

	AN_counters.secs2poll--;

	if ( AN_counters.secs2poll <= 0 ) {	 	// programacion defensiva: uso <=
		// Reajusto el timer
		if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
			AN_flags.start2poll = TRUE;	// Habilito un nuevo poleo
			AN_counters.secs2poll = 15;
			return;
		}
		if ( systemVars.wrkMode == WK_NORMAL ) {
			AN_flags.start2poll = TRUE;	// Habilito un nuevo poleo
			AN_counters.secs2poll = systemVars.timerPoll;
			return;
		}

		// Default: sigo corriendo el timer pero no habilito a polear
		AN_counters.secs2poll = systemVars.timerPoll;
	}
}
//--------------------------------------------------------------------------------------
static void pv_AINgetNextEvent(void)
{
// Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
// Tenemos un array de eventos y todos se evaluan.

u08 i;

	wdgStatus.analogCP = 3;

	// Inicializo la lista de eventos.
	for ( i=0; i < dEVENT_COUNT; i++ ) {
		dEventos[i] = FALSE;
	}

	// Evaluo los eventos
	// EV00: INIT
	if ( AN_flags.starting == TRUE ) { dEventos[evINIT] = TRUE; }

	// EV01: EV_MSGreload: recargar la configuracion
	if ( AN_flags.msgReload == TRUE ) { dEventos[evRELOADCONFIG] = TRUE;	}

	// EV02: EV_f_start2poll
	if ( AN_flags.start2poll == TRUE ) { dEventos[evSTART2POLL] = TRUE; }

	// EV03: EV_counter_secs2pwrSettleNOT_0
	if ( AN_counters.secs2pwrSettle != 0 ) { dEventos[evSEC2PWRSETTLE_NOT_0] = TRUE; }

	// EV04: EV_pollCounterNOT_0
	if ( AN_counters.nroPoleos != 0 ) { dEventos[evPOLLCNT_NOT_0] = TRUE; }

}
/*------------------------------------------------------------------------------------*/
static void pv_AINfsm(void)
{
	// El manejar la FSM con un switch por estado y no por transicion me permite
	// priorizar las transiciones.
	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
	//
	wdgStatus.analogCP = 4;

	switch ( tkAIN_state ) {
	case tkdST_INIT:
		tkAIN_state = trD00();	// TR00
		break;

	case tkdST_STANDBY:
		if ( dEventos[evRELOADCONFIG] == TRUE  ) { tkAIN_state = trD01();break; }
		if ( dEventos[evSTART2POLL] == TRUE  ) { tkAIN_state = trD02();break; }
		break;

	case tkdST_PWRSETTLE:
		if ( dEventos[evSEC2PWRSETTLE_NOT_0] == TRUE  ) {
			tkAIN_state = trD03();
		} else {
			tkAIN_state = trD04();
		}
		break;

	case tkdST_POLLING:
		if ( dEventos[evPOLLCNT_NOT_0] == TRUE  ) {
			tkAIN_state = trD05();
		} else {
			tkAIN_state = trD06();
		}
		break;

	default:
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("tkAnalogIn::ERROR state NOT DEFINED..\r\n\0"));
		FreeRTOS_write( &pdUART1, aIn_printfBuff,sizeof(aIn_printfBuff) );
		tkAIN_state  = tkdST_INIT;
		break;

	}
}
/*------------------------------------------------------------------------------------*/
static int trD00(void)
{
	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo el sistema aqui
	// tkdST_INIT->tkdST_STANDBY

	wdgStatus.analogCP = 5;

	// Init (load parameters) & start pollTimer
	AN_flags.starting = FALSE;

	pv_AinLoadParameters();

	// Apagar los sensores.
	if ( ! MCP_setSensorPwr( 0 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD00 pwroff sensors ERROR !!\r\n\0"), tickCount);
		u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
	if ( ! MCP_setAnalogPwr( 0 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD00 pwroff analogPwr ERROR !!\r\n\0"), tickCount);
		u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD00 pwroff sensors\r\n\0"), tickCount);

	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	pv_AINprintExitMsg(0);
	return(tkdST_STANDBY);
}
/*------------------------------------------------------------------------------------*/
static int trD01(void)
{
	// MSG de autoreload
	// tkdST_STANDBY->tkdST_STANDBY

	wdgStatus.analogCP = 6;

	AN_flags.msgReload = FALSE;

	// Init (load parameters) & start pollTimer
	pv_AinLoadParameters();

	// En 15s hago un poleo.
	AN_counters.secs2poll = 15;

	// Apagar los sensores.
	if ( ! MCP_setSensorPwr( 0 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD01 pwroff sensors ERROR !!\r\n\0"), tickCount);
		u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
	if ( ! MCP_setAnalogPwr( 0 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD01 pwroff analogPwr ERROR !!\r\n\0"), tickCount);
		u_debugPrint(D_BASIC, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD01 pwroff sensors\r\n\0"), tickCount);

	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	pv_AINprintExitMsg(1);
	return(tkdST_STANDBY);
}
/*------------------------------------------------------------------------------------*/
static int trD02(void)
{
	// tkdST_STANDBY->tkdST_PWRSETTLE

	// Aqui comienza un ciclo de poleo: indico a tkGprs que no disque.
	while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_NO_DIAL , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	wdgStatus.analogCP = 7;

	AN_flags.start2poll = FALSE;
	// Inicialmente el frame esta bueno.
	f_skipFrame = FALSE;
	// Inicio el contador de segundos para que se estabilizen las fuentes.
	AN_counters.secs2pwrSettle = SECS2PWRSETTLE;

	// Siempre prendo las fuentes. Si estoy en modo continuo ya van a estar prendidas
	// por lo que no hace nada.
	if ( ! MCP_setSensorPwr( 1 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD02 pwrOn sensors ERROR !!\r\n\0"));
		u_debugPrint(D_DEBUG, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}
	if ( ! MCP_setAnalogPwr( 1 ) ) {
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD02 pwrOn analogPwr ERROR !!\r\n\0"));
		u_debugPrint(D_DEBUG, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	pv_AINprintExitMsg(2);
	return(tkdST_PWRSETTLE);
}
/*------------------------------------------------------------------------------------*/
static int trD03(void)
{
	// tkdST_PWRSETTLE->tkdST_PWRSETTLE
	// Espero 5s. que se estabilizen las fuentes.

	wdgStatus.analogCP = 8;

	if ( AN_counters.secs2pwrSettle > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		AN_counters.secs2pwrSettle--;
	}

	return(tkdST_PWRSETTLE);

}
/*------------------------------------------------------------------------------------*/
static int trD04(void)
{
	// tkdST_PWRSETTLE->tkdST_POLLING

	wdgStatus.analogCP = 9;

	AN_counters.nroPoleos = CICLOS_POLEO;

	// Init Data Structure
	rAIn[0] = 0;
	rAIn[1] = 0;
	rAIn[2] = 0;
	rAIn[3] = 0;	// Batt.

	// Dummy convert para prender el ADC ( estabiliza la medida).
//	vTaskDelay( (portTickType)(1500 / portTICK_RATE_MS) );

	pv_AINprintExitMsg(4);
	return(tkdST_POLLING);
}
/*------------------------------------------------------------------------------------*/
static int trD05(void)
{
	// tkdST_POLLING->tkdST_POLLING
	// Poleo

u16 adcRetValue;
s08 retS;

	wdgStatus.analogCP = 10;

	// Dummy convert para prender el ADC ( estabiliza la medida).
	tickCount = xTaskGetTickCount();
	retS = ADS7827_readCh0( &adcRetValue);
	if ( !retS ) {
		catch_I2C_Error(0);
		// Posiblemente el MCP esta bien pero ya no prendi al ADC. Lo prendo
		MCP_setSensorPwr( 1 );
		MCP_setAnalogPwr( 1 );
		vTaskDelay( ( TickType_t)( 1500 / portTICK_RATE_MS ) );
	}

	if ( AN_counters.nroPoleos > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		AN_counters.nroPoleos--;
	}

	// Dummy convert para prender el ADC ( estabiliza la medida).
	// Esta espera de 1.5s es util entre ciclos de poleo.
	//
	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD05:\r\n\0"), tickCount);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readCh0( &adcRetValue);	// AIN0->ADC3;
	if ( !retS ) {
		catch_I2C_Error(0);
	}
	rAIn[0] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_0,adc_3,val=%d,r0=%.0f\r\n\0"),adcRetValue, rAIn[0]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readCh1( &adcRetValue); // AIN1->ADC5;
	if ( !retS ) {
		catch_I2C_Error(1);
	}
	rAIn[1] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_1,adc_5,val=%d,r1=%.0f\r\n\0"),adcRetValue, rAIn[1]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readCh2( &adcRetValue); // AIN2->ADC7;
	if ( !retS ) {
		catch_I2C_Error(2);
	}
	rAIn[2] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_2,adc_7,val=%d,r1=%.0f\r\n\0"), adcRetValue, rAIn[2]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	retS = ADS7827_readBatt( &adcRetValue); // BATT->ADC1;
	if ( !retS ) {
		catch_I2C_Error(3);
	}
	rAIn[3] += adcRetValue;
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch_3,adc_1,val=%d,r1=%.0f\r\n\0"), adcRetValue, rAIn[3]);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	pv_AINprintExitMsg(5);
	return(tkdST_POLLING);

}
/*------------------------------------------------------------------------------------*/
static int trD06(void)
{
	// tkdST_POLLING->tkdST_STANDBY

double I,M;
u16 D;
u08 channel;
u16 pos = 0;
size_t bWrite;
StatBuffer_t pxFFStatBuffer;

	wdgStatus.analogCP = 11;

	//  En modo discreto debo apagar sensores
	if ( (systemVars.pwrMode == PWR_DISCRETO ) && ( systemVars.wrkMode == WK_NORMAL )) {
		if ( ! MCP_setSensorPwr( 0 ) ) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD06 pwroff sensors ERROR !!\r\n\0"));
			u_debugPrint(D_DEBUG, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}
		if ( ! MCP_setAnalogPwr( 0 ) ) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD06 pwroff analogPwr ERROR !!\r\n\0"));
			u_debugPrint(D_DEBUG, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}

		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD06 pwroff sensors\r\n\0"));
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Promedio canales analogicos 0..2 y bateria (3)
	for ( channel = 0; channel < (NRO_CHANNELS + 1); channel++) {
		rAIn[channel] /= CICLOS_POLEO;
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD06 AvgCh[%d]=%.02f\r\n\0"), tickCount, channel, rAIn[channel]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Convierto de ADC a magnitudes.
	for ( channel = 0; channel < NRO_CHANNELS ; channel++) {
		// Calculo la corriente medida en el canal
		I = rAIn[channel] * systemVars.Imax[channel] / 4096;
		// Calculo la pendiente
		M = 0;
		D = systemVars.Imax[channel] - systemVars.Imin[channel];
		if ( D != 0 ) {
			M = ( systemVars.Mmax[channel]  -  systemVars.Mmin[channel] ) / D;
			rAIn[channel] = systemVars.Mmin[channel] + M * ( I - systemVars.Imin[channel] );
		} else {
			// Error: denominador = 0.
			rAIn[channel] = -999;
		}
	}

	// Convierto la bateria.
	rAIn[NRO_CHANNELS] = (15 * rAIn[NRO_CHANNELS]) / 4096;	// Bateria

	// Paso al systemVars.
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("frame::{" ));

	// Inserto el timeStamp.
	RTC_read(&Aframe.rtc);
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

	// Valores analogicos
	for ( channel = 0; channel < NRO_CHANNELS; channel++) {
		// Si la lectura es correcta paso los datos al Aframe
		if ( f_skipFrame == FALSE ) {
			Aframe.analogIn[channel] = rAIn[channel];
		}
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%s=%.02f,"),systemVars.aChName[channel],Aframe.analogIn[channel] );
	}

	// Leo los datos digitales y los pongo en 0.
	u_readDigitalCounters( &Aframe.dIn, TRUE );
	// Convierto los pulsos a los valores de la magnitud.
	Aframe.dIn.pulses[0] *=  systemVars.magPP[0];
	Aframe.dIn.pulses[1] *=  systemVars.magPP[1];
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%sP=%.02f,%sL=%d,"), systemVars.dChName[0],Aframe.dIn.pulses[0],systemVars.dChName[0],Aframe.dIn.level[0],systemVars.dChName[0]);
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%sP=%.02f,%sL=%d"), systemVars.dChName[1],Aframe.dIn.pulses[1],systemVars.dChName[1],Aframe.dIn.level[1],systemVars.dChName[1]);

	// Bateria
	// Si la lectura es correcta paso los datos al Aframe
	if ( f_skipFrame == FALSE ) {
		Aframe.batt = rAIn[3];
	}
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(",bt=%.02f}\0"),Aframe.batt );

	xSemaphoreGive( sem_SYSVars );

	// EL primer poleo puede tener datos erroneos por lo que lo descarto.
	if ( AN_flags.firstPoll == TRUE ) {
		AN_flags.firstPoll = FALSE;
		FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0") );
		goto quit;
	}

	// BD SAVE FRAME solo en modo normal.
	if ( systemVars.wrkMode == WK_NORMAL ) {
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);
		// Error de escritura ??
		if ( bWrite != sizeof(Aframe) ) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
			goto quit;
		}
	}

	// En modo normal o monitor frame muestro el dato
	if ( systemVars.wrkMode == WK_NORMAL ) {
		// En modo normal agrego el mem.stats
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(" MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		FreeRTOS_write( &pdUART1, "POLL->\0", sizeof("POLL->\0") );
	} else if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("\r\n\0"));
		FreeRTOS_write( &pdUART1, "MON->\0", sizeof("MON->\0") );
	} else {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("\r\n\0"));
	}

	if ( f_skipFrame == TRUE ) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR("Skipped frame\r\n\0"));
	}

	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );



quit:

	// Aqui termino un ciclo de poleo: indico a tkGprs que puede discar.
	while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_CAN_DIAL , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	pv_AINprintExitMsg(6);
	return(tkdST_STANDBY);

}
/*------------------------------------------------------------------------------------*/
static void pv_AINprintExitMsg(u08 code)
{
	wdgStatus.analogCP = 12;

	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::exit TR%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_AinLoadParameters( void )
{
	// Dependiendo del modo de trabajo normal, service, idle, monitor, setea el
	// timer de poleo.

	wdgStatus.analogCP = 13;

	// Al comienzo poleo rapido aunque luego no lo salvo
	if ( AN_flags.firstPoll == TRUE ) {
		AN_counters.secs2poll = 15;
		return;
	}

	// En modo monitor poleo c/15s
	if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
		AN_counters.secs2poll = 15;
		return;
	}

	// En todos los otros casos, poleo con timerpoll
	AN_counters.secs2poll = systemVars.timerPoll;
	return;
}
/*------------------------------------------------------------------------------------*/
s16 u_readTimeToNextPoll(void)
{
s16 retVal = -1;

	wdgStatus.analogCP = 14;

	// Lo determina en base al time elapsed y el timerPoll.
	// El -1 indica un modo en que no esta poleando.
	if ( ( systemVars.wrkMode == WK_NORMAL ) || ( systemVars.wrkMode == WK_MONITOR_FRAME )) {
		retVal = AN_counters.secs2poll;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readAnalogFrame (frameData_t *dFrame)
{
	wdgStatus.analogCP = 15;

	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
/*------------------------------------------------------------------------------------*/
void catch_I2C_Error( u08 channel )
{
	// Invocada por un error en la lectura de un canal del ADC.
	// H1) El ADC anda mal
	// H2) El ADC esta apagado porque el MCP23018 no lo puede prender
	// H3) El ADC anda mal porque el TPS731XX no lo puede prender
	// H4) El bus I2C esta mal.

	// Verifico la H4 leyendo el RTC. Si el bus I2C esta mal, no voy a poder leer el RTC
	// tampoco.

	// 2016-01-18:
	// El problema que veo es que el MPC23018 esta desconfigurado y por eso los pines que
	// deben actuar como salidas no lo hacen.
	// Esto hace que no prenda las fuentes y por eso el ADC no es leido bien y se genera
	// el error.


//RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 regValue;

	// Cuento los errores para resetearme
	rdErrors++;
	f_skipFrame = TRUE;

//	retS = RTC_read(&rtcDateTime);
	retS = MCP_read( MCP1_ADDR, 0x01, &regValue );
	if ( !retS ) {
		// No pude leer el MCP. Asumo un problema en el bus.
		// Deshabilito la interfase TWI. La proxima lectura la voy a habilitar y veremos
		// si esto soluciona el problema.
		TWCR &= ~(1 << TWEN);

		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD05 I2C BUS ERROR !! Disable interface...\r\n\0"));
	} else {
		// El bus I2C anda bien y el problema esta en el ADC o en el MCP
		if ( regValue != 0x64) {
			// El MCP esta desconfigurado: Lo reprogramo
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD05 MCP ERROR !!\r\n\0"));
			pvMCP_init_MCP1(1);
		} else {
			// El problema esta en el ADC
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("**DEBUG::tkAnalogIn::trD05 CH%d ADC ERROR !!\r\n\0"),channel);
		}
	}

	u_debugPrint(D_DEBUG, aIn_printfBuff, sizeof(aIn_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
