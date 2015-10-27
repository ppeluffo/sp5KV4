/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <sp5Klibs/avrlibdefs.h>
#include <sp5Klibs/avrlibtypes.h>
#include <sp5Klibs/global.h>			// include our global settings

#include "cmdline.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"
#include "sp5K_uart.h"
#include "sp5K_i2c.h"

#include "rtc_sp5K.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "3.0.5"
#define SP5K_DATE "@ 20150929"

#define SP5K_MODELO "sp5KV3 HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"


#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256
#define CHAR384	 	384

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkTimers_STACK_SIZE		256
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	256
#define tkEventos_STACK_SIZE	256
#define tkDigitalIn_STACK_SIZE	256
#define tkAIn_STACK_SIZE		512
#define tkGprs_STACK_SIZE		512
#define tkOutput_STACK_SIZE		256

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigitalIn_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkAIn_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkEventos_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkGprs_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkTimers_TASK_PRIORITY 		( tskIDLE_PRIORITY + 2 )
#define tkOutput_TASK_PRIORITY 		( tskIDLE_PRIORITY + 2 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkDigitalIn(void * pvParameters);
void tkAnalogIn(void * pvParameters);
void tkEventos(void * pvParameters);
void tkGprs(void * pvParameters);
void tkTimers(void * pvParameters);
void tkOutput(void * pvParameters);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkDigitalIn, xHandle_tkAIn, xHandle_tkEventos, xHandle_tkGprs, xHandle_tkTimers, xHandle_tkOutput;

// Mensajes entre tareas
#define RET2NORMAL_BIT	0x10		// return 2 normal wrkmode

#define AINMSG_RELOAD		0x01	// to tkAnalogIN: reload
#define AINMSG_POLL			0x02	// to tkAnalogIN: poll
#define CTLMSG_STARTEWM2N	0x04	// to tkControlIN: exitWrkMode2Normal

#define FRAMERDY_BIT		0x01	// to tkEventos: Frame Data Ready

#define GPRSMSG_RELOAD		0x01	// to tkGprsIN: reload

#define MSG2OUT_UPDATE		0x01	// to tkOutputIN: set outputs values

//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

typedef enum { WK_IDLE = 0, WK_NORMAL = 1, WK_SERVICE = 2, WK_MONITOR_FRAME = 3, WK_MONITOR_SQE = 4  } t_wrkMode;
typedef enum { PWR_CONTINUO = 0, PWR_DISCRETO = 1 } t_pwrMode;
typedef enum { OUTPUT_CONSIGNA = 0, OUTPUT_MANUAL = 1 } t_outMode;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON = 1 } t_pwrSave;

#define NRO_CHANNELS		3

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

typedef struct {
	u08 din0_level;					// 1
	double din0_pulses;				// 4
	u16 din0_width;					// 2

	u08 din1_level;					// 1
	double din1_pulses;				// 4
	u16 din1_width;					// 2
} dinDataType;		// 14 bytes

typedef struct {
	u08 tagByte;
	// size = 7+5+5+4+3*4+1 = 33 bytes
	RtcTimeType_t rtc;				// 7
	dinDataType dIn;				// 12
	double analogIn[NRO_CHANNELS];	// 12
	double batt;

} frameDataType;	// 38 bytes

typedef struct {
	u08 wrkMode;
	u08 dout[4];
	u16 horaConsDia;
	u16 horaConsNoc;
	u08 chVA;
	u08 chVB;
} outputsType;		// 11 bytes

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	u16 dummyBytes;
	u08 initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char serverPort[PORT_LENGTH];
	char serverAddress[IP_LENGTH];
	char serverIp[IP_LENGTH];
	char dlgIp[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	u08 csq;
	u08 dbm;
	u08 dcd;
	u08 ri;
	u08 termsw;

	u16 timerPoll;
	u32 timerDial;

	t_wrkMode wrkMode;
	t_pwrMode pwrMode;

	u08 logLevel;		// Nivel de info que presentamos en display.
	u08 debugLevel;		// Indica que funciones debugear.
	u08 gsmBand;

	u08 pwrSave;
	u16 pwrSaveStartTime;
	u16 pwrSaveEndTime;

	// Nombre de los canales
	char aChName[NRO_CHANNELS][PARAMNAME_LENGTH];
	char dChName[2][PARAMNAME_LENGTH];

	// Configuracion de Canales analogicos
	u08 Imin[NRO_CHANNELS];				// Coeficientes de conversion de I->magnitud (presion)
	u08 Imax[NRO_CHANNELS];
	u08 Mmin[NRO_CHANNELS];
	double Mmax[NRO_CHANNELS];
	double offmmin[NRO_CHANNELS];		// offset del ADC en mmin.
	double offmmax[NRO_CHANNELS];		// offset del ADC en mmax.

	// Configuracion de canales digitales
	double magPP[2];

	outputsType outputs;

} systemVarsType;	// 315 bytes

systemVarsType systemVars;

//------------------------------------------------------------------------------------

// Led de placa analogica ( PD6 )
#define LED_KA_PORT		PORTD
#define LED_KA_PIN		PIND
#define LED_KA_BIT		6
#define LED_KA_DDR		DDRD
#define LED_KA_MASK		0x40

#define LED_MODEM_PORT		PORTC
#define LED_MODEM_PIN		PINC
#define LED_MODEM_BIT		3
#define LED_MODEM_DDR		DDRC
#define LED_MODEM_MASK		0x04

// Q PINES
#define Q_PORT		PORTA
#define Q_DDR		DDRA
#define Q0_CTL_PIN	2
#define Q1_CTL_PIN	3

// DEBUG
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16, D_EVENTOS = 32 } t_debug;

//------------------------------------------------------------------------------------

s08 pvSaveSystemParamsInEE( systemVarsType *sVars );

#define SECS2TICKS	( 1000 / portTICK_PERIOD_MS )

char *byte_to_binary(int x);

void getDigitalInputs( dinDataType *dIn , s08 resetCounters );

s16 getTimeToNextPoll(void);
s32 getTimeToNextDial(void);

s08 gprsAllowSleep(void);

typedef enum { M_OFF, M_OFF_IDLE, M_ON_CONFIG, M_ON_READY } t_modemStatus;

u08 getGprsModemStatus(void);

void getAnalogFrame (frameDataType *dFrame);

typedef enum { OFF = 0, ON = 1 } t_onOff;
typedef enum { T_APAGADA = 0, T_PRENDIDA = 1 } t_terminalStatus;
typedef enum { INIT = 0, SET_ON = 1, SET_OFF = 2, STATUS = 3 } t_setOnOff;

// FUNCIONES DE USO GENERAL.
s08 pb_setParamPwrMode(u08 pwrMode);
s08 pb_setParamTimerPoll(char *p1);
s08 pb_setParamTimerDial(char *p1);
void pb_setParamPwrSave(u08 p1, char *p2, char *p3);
s08 pb_setParamAnalogCh( u08 channel, char *p1, char *p2, char *p3, char *p4, char *p5 );
void pb_setParamDigitalCh( u08 channel, char *p1, char *p2 );
s08 pb_setParamOutputs( u08 modo,char *p1,char *p2,char *p3,char *p4,char *p5,char *p6,char *p7,char *p8 );
s08 pb_outPulse( u08 channel, u08 phase, u16 delay );
void pb_outs2standby(void);
void pb_restartTimerTerminal(void);

u16 pv_convertHHMM2min(u16 HHMM );
u16 pv_convertMINS2hhmm ( u16 mins );

void pb_setConsignaDiurna ( u16 ms);
void pb_setConsignaNocturna ( u16 ms );
#define CONSIGNA_MS		250

//------------------------------------------------------------------------------------
// WATCHDOG
u08 systemWdg;

#define WDG_TIMERS		0x01
#define WDG_CTL			0x02
#define WDG_CMD			0x04
#define WDG_EVN			0x08
#define WDG_DIN			0x10
#define WDG_AIN			0x20
#define WDG_GPRS		0x40
#define WDG_OUT			0x80

void clearWdg( u08 wdgId );

//------------------------------------------------------------------------------------
// TERMINAL
// Pin de control de fuente de la terminal ( PD7)
#define TERMSW_PORT		PORTD
#define TERMSW_PIN		PIND
#define TERMSW_BIT		7
#define TERMSW_DDR		DDRD
#define TERMSW_MASK		0x80

#define SECS2TERMOFF	30		// 120 segundos para apagar la terminal.

s08 terminal_isApagada(void);			// Publica
s08 terminal_isPrendida(void);			// Publica
void terminal_restartTimer( u16 secs);	// Publica


char debug_printfBuff[CHAR128];

#endif /* SP5K_H_ */
