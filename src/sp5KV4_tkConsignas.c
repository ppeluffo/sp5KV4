#include "sp5KV4.h"

static char cons_printfBuff[CHAR128];

//------------------------------------------------------------------------------------
float fuzzy_me ( float value, float xa, float xb, float xc, float xd);
float in_eBajo, in_eMedio, in_eAlto;	// Variables linguisticas de la entrada
float out_pBajo, out_pMedio, out_pAlto; // Salidas
void fuzzificar( float input );
void rule_evaluation(void);
float desfuzzificar(void);
float area ( float h, float xa, float xb, float xc, float xd);
float cog ( float h, float xa, float xb, float xc, float xd);

//------------------------------------------------------------------------------------

// Estados
typedef enum {  cST_INIT = 0,
				cST_Cons01,
				cST_Cons02,
				cST_Cons03,
				cST_Cons04,
				cST_Cons05,
				cST_Cons06
} t_tkConsignas_state;

// Eventos
typedef enum {
	evINIT = 0,
	evMODEM_PRENDIDO,
	evCTIMER_NOT_0,
	evCONSIGNA_DIA,
	evCONSIGNA_NOCHE
} t_tkConsignas_eventos;

#define cEVENT_COUNT		5

static s08 cEventos[cEVENT_COUNT];

static u32 tickCount;					// para usar en los mensajes del debug.

// transiciones
static int trC00(void);
static int trC01(void);
static int trC02(void);
static int trC03(void);
static int trC04(void);
static int trC05(void);
static int trC06(void);
static int trC07(void);
static int trC08(void);
static int trC09(void);
static int trC10(void);
static int trC11(void);
static int trC12(void);

static u08 tkCONS_state = cST_INIT;	// Estado

static struct {
	s08 starting;
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 consignaDia;
	s08 consignaNoche;
} CONS_flags;

static struct {
	u16 cTimer;			// contador de segundos hasta el siguiente poleo
} CONS_counters;

#define SECS2WAIT4CHECKCONSIGNAS	30

// Funciones generales
static void pv_CONSgetNextEvent(void);
static void pv_CONSfsm(void);
static void pv_CONSprintExitMsg(u08 code);
void pv_setConsignaInicial ( void );

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

	CONS_flags.starting = TRUE;
	CONS_flags.consignaDia = FALSE;
	CONS_flags.consignaNoche = FALSE;
	CONS_flags.msgReload = FALSE;
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
				CONS_flags.msgReload = TRUE;
			}

		}

		// Analizo los eventos.
		pv_CONSgetNextEvent();
		// Corro la maquina de estados.
		pv_CONSfsm();

	}
}
// ------------------------------------------------------------------------------------
 static void pv_CONSgetNextEvent(void)
 {
 // Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
 // Tenemos un array de eventos y todos se evaluan.

 u08 i;

  	// Inicializo la lista de eventos.
 	for ( i=0; i < cEVENT_COUNT; i++ ) {
 		cEventos[i] = FALSE;
 	}

 	// Evaluo los eventos
 	// EV00: INIT
 	if ( CONS_flags.starting == TRUE ) { cEventos[evINIT] = TRUE; }
 	if ( u_modemPwrStatus() == TRUE ) { cEventos[evMODEM_PRENDIDO] = TRUE;	}
 	if ( CONS_counters.cTimer != 0 ) { cEventos[evCTIMER_NOT_0] = TRUE; }
 	if ( CONS_flags.consignaDia == TRUE ) { cEventos[evCONSIGNA_DIA] = TRUE; }
 	if ( CONS_flags.consignaNoche == TRUE ) { cEventos[evCONSIGNA_NOCHE] = TRUE; }

 }
/*------------------------------------------------------------------------------------*/
 static void pv_CONSfsm(void)
 {
 	// El manejar la FSM con un switch por estado y no por transicion me permite
 	// priorizar las transiciones.
 	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
 	//

 	switch ( tkCONS_state ) {
 	case cST_INIT:
 		tkCONS_state = trC00();
 		break;

 	case cST_Cons01:
 		if ( cEventos[evMODEM_PRENDIDO] == TRUE  ) {
 			tkCONS_state = trC01();
 		} else {
 			tkCONS_state = trC02();
 		}
 		break;

 	case cST_Cons02:
 		if ( cEventos[evCTIMER_NOT_0] == TRUE  ) {
 			tkCONS_state = trC03();
 		} else {
 			tkCONS_state = trC04();
 		}
 		break;

 	case cST_Cons03:
 		if ( cEventos[evCONSIGNA_DIA] == TRUE  ) {
 			tkCONS_state = trC05();
 		} else {
 			tkCONS_state = trC06();
 		}
 		break;

 	case cST_Cons04:
 		if ( cEventos[evMODEM_PRENDIDO] == TRUE  ) {
 			tkCONS_state = trC09();
 		} else {
 			tkCONS_state = trC10();
 		}
 		break;

 	case cST_Cons05:
 		if ( cEventos[evCONSIGNA_NOCHE] == TRUE  ) {
 			tkCONS_state = trC07();
 		} else {
 			tkCONS_state = trC08();
 		}
 		break;

 	case cST_Cons06:
 		if ( cEventos[evMODEM_PRENDIDO] == TRUE  ) {
 			tkCONS_state = trC11();
 		} else {
 			tkCONS_state = trC12();
 		}
 		break;

 	default:
 		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("tkAnalogIn::ERROR state NOT DEFINED..\r\n\0"));
 		FreeRTOS_write( &pdUART1, cons_printfBuff,sizeof(cons_printfBuff) );
 		tkCONS_state  = cST_INIT;
 		break;

 	}
 }
/*------------------------------------------------------------------------------------*/
 static void pv_CONSprintExitMsg(u08 code)
 {
 	tickCount = xTaskGetTickCount();
 	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR(".[%06lu] tkConsignas::exit TR%02d\r\n\0"), tickCount,code);
 	u_debugPrint(D_CONSIGNA, cons_printfBuff, sizeof(cons_printfBuff) );
 }
/*------------------------------------------------------------------------------------*/
static int trC00(void)
 {
 	// cST_INIT->cST_Cons01

	CONS_flags.starting = FALSE;
 	pv_CONSprintExitMsg(0);
 	return(cST_Cons01);
}
/*------------------------------------------------------------------------------------*/
static int trC01(void)
{
 	// cST_Cons01->cST_Cons01
	// Espera mientras el modem esta prendido.

 	//pv_CONSprintExitMsg(1);
 	return(cST_Cons01);
}
/*------------------------------------------------------------------------------------*/
static int trC02(void)
 {
	 // cST_Cons01->cST_Cons02
	 // Fijo la consigna inicial.

	 pv_setConsignaInicial();

	 CONS_counters.cTimer = SECS2WAIT4CHECKCONSIGNAS;

	 pv_CONSprintExitMsg(2);
	 return(cST_Cons02);
 }
/*------------------------------------------------------------------------------------*/
static int trC03(void)
 {
	 // cST_Cons02->cST_Cons02
	 // Espero 30s

	 if ( CONS_counters.cTimer > 0 ) {
		 vTaskDelay( ( TickType_t)( 900 / portTICK_RATE_MS ) );
		 CONS_counters.cTimer--;
	 }

	 //pv_CONSprintExitMsg(3);
	 return(cST_Cons02);
 }
/*------------------------------------------------------------------------------------*/
static int trC04(void)
 {
	 // cST_Cons02->cST_Cons03
	 // Evaluo si la hora corresponde a setear alguna consigna y lo indico
	 // con la flag correspondiente

RtcTimeType_t rtcDateTime;
u16 now;

	CONS_flags.consignaDia = FALSE;
	CONS_flags.consignaNoche = FALSE;

	 // Si las consignas no estan configuradas, salgo
	 if ( systemVars.consigna.status == CONSIGNA_OFF )
	 	goto quit;

	 // Hora actual en minutos.
	 RTC_read(&rtcDateTime);
	 now = rtcDateTime.hour * 60 + rtcDateTime.min;

	 if ( now == systemVars.consigna.horaConsDia ) {
		 CONS_flags.consignaDia = TRUE;
		 goto quit;
	 }

	 if ( now == systemVars.consigna.horaConsNoc ) {
		CONS_flags.consignaNoche = TRUE;
	 	goto quit;
	 }

quit:
	 pv_CONSprintExitMsg(4);
	 return(cST_Cons03);
 }
/*------------------------------------------------------------------------------------*/
static int trC05(void)
{
 	// cST_Cons03->cST_Cons04

 	pv_CONSprintExitMsg(5);
 	return(cST_Cons04);
}
/*------------------------------------------------------------------------------------*/
static int trC06(void)
 {
 	// cST_Cons03->cST_Cons05

 	pv_CONSprintExitMsg(6);
 	return(cST_Cons05);
 }
/*------------------------------------------------------------------------------------*/
static int trC07(void)
 {
 	// cST_Cons05->cST_Cons06

 	pv_CONSprintExitMsg(7);
 	return(cST_Cons06);
 }
/*------------------------------------------------------------------------------------*/
static int trC08(void)
 {
	 // cST_Cons05->cST_Cons02
	 // Inicializo el timer para volver a chequear las consignas

	 CONS_counters.cTimer = SECS2WAIT4CHECKCONSIGNAS;

	 pv_CONSprintExitMsg(8);
	 return(cST_Cons02);
 }
/*------------------------------------------------------------------------------------*/
static int trC09(void)
 {
	 // cST_Cons04->cST_Cons04
	 // Espero hasta que el modem este apagado

	 // pv_CONSprintExitMsg(9);
	 return(cST_Cons04);
 }
/*------------------------------------------------------------------------------------*/
static int trC10(void)
{
	// cST_Cons04->cST_Cons02
	// Seteo la consigna DIURNA

	u_setConsignaDiurna();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Diurna\r\n\0"), u_now() );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	CONS_counters.cTimer = SECS2WAIT4CHECKCONSIGNAS;
	CONS_flags.consignaDia = FALSE;

 	pv_CONSprintExitMsg(10);
 	return(cST_Cons02);
 }
 /*------------------------------------------------------------------------------------*/
 static int trC11(void)
 {
	 // cST_Cons06->cST_Cons06
	 // Espero que el modem este apagado

	 pv_CONSprintExitMsg(11);
	 return(cST_Cons06);
 }
 /*------------------------------------------------------------------------------------*/
 static int trC12(void)
 {
 	// cST_Cons06->cST_Cons02
	// Seteo la consigna NOCTURNA

	u_setConsignaNocturna();
	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Set Consigna Nocturna\r\n\0"), u_now() );
	u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );

	CONS_counters.cTimer = SECS2WAIT4CHECKCONSIGNAS;
	CONS_flags.consignaNoche = FALSE;

 	pv_CONSprintExitMsg(12);
 	return(cST_Cons02);
 }
 /*------------------------------------------------------------------------------------*/
void pv_setConsignaInicial ( void )
{
	// Determino cual consigna corresponde aplicar y la aplico.
RtcTimeType_t rtcDateTime;
u16 now;

	// Hora actual en minutos.
	RTC_read(&rtcDateTime);
	now = rtcDateTime.hour * 60 + rtcDateTime.min;

	if ( systemVars.consigna.status == CONSIGNA_OFF ) {
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: OFF\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	if ( now <= systemVars.consigna.horaConsDia ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( ( now > systemVars.consigna.horaConsDia ) && ( now <= systemVars.consigna.horaConsNoc )) {
		u_setConsignaDiurna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Diurna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

	if ( now > systemVars.consigna.horaConsNoc ) {
		u_setConsignaNocturna();
		snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("%s Consigna Inicial: Nocturna\r\n\0"), u_now() );
		u_debugPrint(D_BASIC, cons_printfBuff, sizeof(cons_printfBuff) );
		return;
	}

}
/*------------------------------------------------------------------------------------*/
/*
 * CONTROL DE CONSIGNAS CONTINUAS CON FUZZY LOGIC
 *
 */

//------------------------------------------------------------------------------------
/*
float fuzzy_me ( float value, float xa, float xb, float xc, float xd)
{
	// Convierte una entrada a un valor de la variable linguistica
	// Utilizamos funciones trapezoidales.

float dom;

	if ( value < xa ) {
		dom = 0;
	} else if ( ( xa <= value) && ( value < xb)) {
		dom = ( value - xa ) / ( xb - xa );
	} else if ( ( xb <= value ) && ( value < xc )) {
		dom = 1;
	} else if ( ( xc <= value) && ( value < xd) ) {
		dom = ( xd - value) / ( xd -xc );
	} else {
		dom = 0;
	}

	return(dom);
}
//------------------------------------------------------------------------------------
void fuzzificar( float input )
{
	// Dado el valor de la entrada, hallo la componente de c/u de las variables
	// linguisticas.
	// Aqui es donde defino los puntos de la funcion de pertenencia

	in_eBajo = fuzzy_me( input, 0, 0, 0, 0.5);
	in_eMedio = fuzzy_me( input, 0, 0.5, 0.5, 1);
	in_eAlto = fuzzy_me( input, 0.5, 1, 6, 6);

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("fuzzificar: in=%l, e_bajo=%l, e_medio=%l, e_alto=%l\r\n\0"), input, in_eBajo, in_eMedio, in_eAlto );
	FreeRTOS_write( &pdUART1, cons_printfBuff,sizeof(cons_printfBuff) );

}
//------------------------------------------------------------------------------------
void rule_evaluation(void)
{
	// 1: if ( error BAJO ) then ( pulso CORTO)
	// 2: if ( error MEDIO ) then ( pulso MEDIO )
	// 3: if ( error ALTO ) then ( pulso ALTO )

	out_pBajo = in_eBajo;
	out_pMedio = in_eMedio;
	out_pAlto = in_eAlto;

}
//------------------------------------------------------------------------------------
float desfuzzificar(void)
{

float S_pBajo, S_pMedio, S_pAlto;
float cog_pBajo, cog_pMedio, cog_pAlto;

	S_pBajo = area(out_pBajo, 0, 0, 0, 0.5);
	S_pMedio = area(out_pMedio, 0, 0.5, 0.5, 1);
	S_pAlto = area(out_pAlto,  0.5, 1, 6, 6);

	snprintf_P( cons_printfBuff,sizeof(cons_printfBuff),PSTR("desfuzzificar: S_bajo=%l, S_medio=%l, S_alto=%l\r\n\0"), S_pBajo, S_pMedio, S_pAlto );
	FreeRTOS_write( &pdUART1, cons_printfBuff,sizeof(cons_printfBuff) );

	cog_pBajo = cog(out_pBajo, 0, 0, 0, 0.5);
	cog_pMedio = cog(out_pMedio, 0, 0.5, 0.5, 1);
	cog_pAlto = cog(out_pAlto,  0.5, 1, 6, 6);


}
//------------------------------------------------------------------------------------
float area ( float h, float xa, float xb, float xc, float xd)
{
	// La base no puede ser cero
float S = 0;

	if ( xd > xa ) {
		S = ( xd - xa ) * ( h - h*h / 2 );
	}
	return(S);

}
//------------------------------------------------------------------------------------
float cog ( float h, float xa, float xb, float xc, float xd)
{
	// Calculo el x del centro de gravedad a partir de http://mathworld.wolfram.com/Trapezoid.html

float a, b, c,d;

	c = sqrt ( square(h) + square( xb -xa) );
	d = sqrt ( square(h) + square( xd -xc) );

}
//------------------------------------------------------------------------------------
*/
