/*
 * sp5KV3_utils.c
 *
 *  Created on: 27/10/2015
 *      Author: pablo
 */

#include "sp5KV4.h"

u08 pv_paramLoad(u08* data, u08* addr, u16 sizebytes);
u08 pv_paramStore(u08* data, u08* addr, u16 sizebytes);
u08 pv_checkSum ( u08 *data,u16 sizebytes );

//----------------------------------------------------------------------------------------
void u_panic( u08 panicCode )
{
char msg[16];

	snprintf_P( msg,sizeof(msg),PSTR("\r\nPANIC(%d)\r\n\0"), panicCode);
	FreeRTOS_write( &pdUART1,  msg, sizeof( msg) );
	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );
	vTaskSuspendAll ();
	vTaskEndScheduler ();
	exit (1);
}
//----------------------------------------------------------------------------------------
s08 u_configAnalogCh( u08 channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax )
{
	// p1 = name, p2 = iMin, p3 = iMax, p4 = mMin, p5 = mMax

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.aChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.aChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_iMin != NULL ) { systemVars.Imin[channel] = atoi(s_iMin); }
	if ( s_iMax != NULL ) {	systemVars.Imax[channel] = atoi(s_iMax); }
	if ( s_mMin != NULL ) {	systemVars.Mmin[channel] = atoi(s_mMin); }
	if ( s_mMax != NULL ) {	systemVars.Mmax[channel] = atof(s_mMax); }

	xSemaphoreGive( sem_SYSVars );

	return(TRUE);

}
//----------------------------------------------------------------------------------------
s08 u_configDigitalCh( u08 channel, char *chName, char *s_magPP )
{
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( chName != NULL ) {
		memset ( systemVars.dChName[channel], '\0',   PARAMNAME_LENGTH );
		memcpy( systemVars.dChName[channel], chName , ( PARAMNAME_LENGTH - 1 ));
	}

	if ( s_magPP != NULL ) { systemVars.magPP[channel] = atof(s_magPP); }

	xSemaphoreGive( sem_SYSVars );
	return(TRUE);

}
//----------------------------------------------------------------------------------------
s08 u_configPwrMode(u08 pwrMode)
{

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrMode =  pwrMode;
	xSemaphoreGive( sem_SYSVars );

	// tk_aIn: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkAIn, TKA_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}
	// tk_Gprs:
	while ( xTaskNotify(xHandle_tkGprs, TKG_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}
	return(TRUE);
}
//----------------------------------------------------------------------------------------
s08 u_configTimerDial(char *s_tDial)
{
u32 tdial;

	tdial = abs( (u32) ( atol(s_tDial) ));
	if ( tdial < 120 ) { tdial = 120; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();
	systemVars.timerDial = tdial;
	xSemaphoreGive( sem_SYSVars );

	return(TRUE);
}
//----------------------------------------------------------------------------------------
void u_configPwrSave(u08 modoPwrSave, char *s_startTime, char *s_endTime)
{
// Recibe como parametros el modo ( 0,1) y punteros a string con las horas de inicio y fin del pwrsave
// expresadas en minutos.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrSave = modoPwrSave;
	if ( s_startTime != NULL ) { systemVars.pwrSaveStartTime = u_convertHHMM2min ( atol(s_startTime) ); }
	if ( s_endTime != NULL ) { systemVars.pwrSaveEndTime = u_convertHHMM2min ( atol(s_endTime) ); }

	xSemaphoreGive( sem_SYSVars );

}
//----------------------------------------------------------------------------------------
void u_configConsignas( u08 modo, char *s_horaConsDia,char *s_horaConsNoc,char *chVA,char *chVB )
{
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	switch(modo) {
	case CONSIGNA_OFF:
		systemVars.consigna.status = CONSIGNA_OFF;
		break;
	case CONSIGNA_ON:
		systemVars.consigna.status = CONSIGNA_ON;
		if ( s_horaConsDia != NULL ) { systemVars.consigna.horaConsDia =  u_convertHHMM2min ( atol(s_horaConsDia) ); }
		if ( s_horaConsNoc != NULL ) { systemVars.consigna.horaConsNoc =  u_convertHHMM2min ( atol(s_horaConsNoc) ); }
		if ( chVA != NULL ) { systemVars.consigna.chVA = atoi(chVA); }
		if ( chVB != NULL ) { systemVars.consigna.chVB = atoi(chVB); }
		break;
	}

	xSemaphoreGive( sem_SYSVars );

	// Le aviso a la tarea de las consignas
	while ( xTaskNotify(xHandle_tkConsignas, TKC_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
u16 u_convertHHMM2min(u16 HHMM )
{
u16 HH,MM,mins;

	HH = HHMM / 100;
	MM = HHMM % 100;
	mins = 60 * HH + MM;
	return(mins);
}
//----------------------------------------------------------------------------------------
u16 u_convertMINS2hhmm ( u16 mins )
{
u16 HH,MM, hhmm;

	HH = mins / 60;
	MM = mins % 60;
	hhmm = HH * 100 + MM;
	return(hhmm);
}
//----------------------------------------------------------------------------------------
void u_vopen ( u08 valveId )
{
	// Aplica un pulso de apertura de 100ms en una valvula dada

	 MCP_outsPulse( valveId , 0, 100 );	// Pulso de apertura

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_outputA1Disable();
	 MCP_outputA2Disable();
	 MCP_outputB1Disable();
	 MCP_outputB2Disable();

	 MCP_outputsSleep();
}
//----------------------------------------------------------------------------------------
void u_close ( u08 valveId )
{
	// Aplica un pulso de cierre de 100ms en una valvula dada

	 MCP_outsPulse( valveId , 1, 100 );	// Pulso de cierre

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_outputA1Disable();
	 MCP_outputA2Disable();
	 MCP_outputB1Disable();
	 MCP_outputB2Disable();

	 MCP_outputsSleep();
}
//----------------------------------------------------------------------------------------
void u_clearWdg( u08 wdgId )
{
	// Pone el correspondiente bit del wdg en 0.
	systemWdg &= ~wdgId ;

}
//------------------------------------------------------------------------------------
s08 u_saveSystemParams(void)
{
	// Salva el systemVars en la EE y verifica que halla quedado bien.
	// Hago hasta 3 intentos.

int EEaddr = 0; // posicion inicial
s08 retS = FALSE;
u08 storeChecksum = 0;
u08 loadChecksum = 0;
u08 i;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( i=0; i<3; i++ ) {
		storeChecksum = pv_paramStore( &systemVars, EEaddr, sizeof(systemVarsType));
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		pv_paramLoad( &tmpSV, EEaddr, sizeof(systemVarsType));
		loadChecksum = pv_checkSum(&tmpSV,sizeof(systemVarsType));

		if ( loadChecksum == storeChecksum ) {
			retS = TRUE;
			break;
		}
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
s08 u_loadSystemParams(void)
{
s08 retS = FALSE;
int EEaddr = 0; // posicion inicial
int i;
	// Leo la configuracion:  Intento leer hasta 3 veces.

	for ( i=0; i<3;i++) {
		retS = pv_paramLoad( &systemVars, EEaddr, sizeof(systemVarsType));
		if ( retS )
			break;
	}

	// Ajustes de inicio:
	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.dcd = 0;
	systemVars.ri = 0;
	systemVars.debugLevel = D_BASIC + D_GPRS;
	systemVars.wrkMode = WK_NORMAL;

	// Cuando arranca si la EE no esta inicializada puede dar cualquier cosa.
	// De este modo controlo el largo de los strings.
	systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
	systemVars.apn[APN_LENGTH - 1] = '\0';
	systemVars.serverPort[PORT_LENGTH - 1] = '\0';
	systemVars.serverAddress[IP_LENGTH - 1] = '\0';
	systemVars.serverIp[IP_LENGTH - 1] = '\0';
	systemVars.dlgIp[IP_LENGTH - 1] = '\0';
	systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
	systemVars.passwd[PASSWD_LENGTH - 1] = '\0';

	// Nombre de los canales
	systemVars.aChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[1][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[2][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.dChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.dChName[1][PARAMNAME_LENGTH - 1] = '\0';

	return(retS);

}
//------------------------------------------------------------------------------------
u08 pv_paramLoad(u08* data, u08* addr, u16 sizebytes)
{
u16 i;
u08 checksum_stored=0;
u08 checksum=0;

	// load parameters
	eeprom_read_block(data, addr, sizebytes);
	// load checksum
	eeprom_read_block(&checksum_stored, addr+sizebytes, sizeof(u08));

	// calculate own checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	if(checksum == checksum_stored)
		return TRUE;
	else
		return FALSE;
}
//------------------------------------------------------------------------------------
s08 u_configTimerPoll(char *s_tPoll)
{
u16 tpoll;

	tpoll = abs((u16) ( atol(s_tPoll) ));
	if ( tpoll < 15 ) { tpoll = 15; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = tpoll;
	xSemaphoreGive( sem_SYSVars );

	return(TRUE);
}
u08 pv_paramStore(u08* data, u08* addr, u16 sizebytes)
{
	// Almacena un string de bytes en la eeprom interna del micro

u16 i;
u08 checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	// store parameters
	eeprom_write_block(data, addr, sizebytes);
	// store checksum
	eeprom_write_block(&checksum, addr+sizebytes, sizeof(u08));

	return(checksum);
}
//------------------------------------------------------------------------------------
u08 pv_checkSum ( u08 *data,u16 sizebytes )
{
u16 i;
u08 checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;
	return(checksum);
}
//------------------------------------------------------------------------------------
void u_loadDefaults(void)
{
u08 channel;

// Configura el systemVars con valores por defecto.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.initByte = 0x49;
	strncpy_P(systemVars.dlgId, PSTR("DEF401\0"),DLGID_LENGTH);
	strncpy_P(systemVars.serverPort, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K.pl\0"),SCRIPT_LENGTH);

	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.gsmBand = 8;
	systemVars.dcd = 0;
	systemVars.ri = 0;
	systemVars.wrkMode = WK_NORMAL;
	systemVars.pwrMode = PWR_DISCRETO;

	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);

	// DEBUG
	systemVars.debugLevel = D_BASIC;

	strncpy_P(systemVars.serverAddress, PSTR("192.168.0.9\0"),IP_LENGTH);
	systemVars.timerPoll = 300;			// Poleo c/5 minutos
	systemVars.timerDial = 1800;		// Transmito c/3 hs.

	systemVars.pwrSave = modoPWRSAVE_OFF;
	systemVars.pwrSaveStartTime =u_convertHHMM2min(2230);	// 22:30 PM
	systemVars.pwrSaveEndTime =	u_convertHHMM2min(630);	// 6:30 AM

	// Todos los canales quedan por default en 0-20mA, 0-6k.
	for ( channel = 0; channel < NRO_CHANNELS ; channel++) {
		systemVars.Imin[channel] = 0;
		systemVars.Imax[channel] = 20;
		systemVars.Mmin[channel] = 0;
		systemVars.Mmax[channel] = 6.0;
	}

	strncpy_P(systemVars.aChName[0], PSTR("pA\0"),3);
	strncpy_P(systemVars.aChName[1], PSTR("pB\0"),3);
	strncpy_P(systemVars.aChName[2], PSTR("pC\0"),3);

	// Canales digitales
	strncpy_P(systemVars.dChName[0], PSTR("v0\0"),3);
	systemVars.magPP[0] = 0.1;
	strncpy_P(systemVars.dChName[1], PSTR("v1\0"),3);
	systemVars.magPP[1] = 0.1;

	// Salidas
	systemVars.consigna.status = CONSIGNA_OFF;
	systemVars.consigna.horaConsDia = u_convertHHMM2min(530);		// Consigna diurna
	systemVars.consigna.horaConsNoc = u_convertHHMM2min(2330);		// Consigna nocturna
	systemVars.consigna.chVA = 0;
	systemVars.consigna.chVB = 1;

	xSemaphoreGive( sem_SYSVars );


}
//------------------------------------------------------------------------------------
s08 u_wrRtc(char *s)
{
u08 dateTimeStr[11];
char tmp[3];
s08 retS;
RtcTimeType_t rtcDateTime;


	/* YYMMDDhhmm */
	if ( s == NULL )
		return(FALSE);

	memcpy(dateTimeStr, s, 10);
	// year
	tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
	rtcDateTime.year = atoi(tmp);
	// month
	tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
	rtcDateTime.month = atoi(tmp);
	// day of month
	tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
	rtcDateTime.day = atoi(tmp);
	// hour
	tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
	rtcDateTime.hour = atoi(tmp);
	// minute
	tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
	rtcDateTime.min = atoi(tmp);

	retS = RTC_write(&rtcDateTime);
	return(retS);
}
/*------------------------------------------------------------------------------------*/
char *u_now(void)
{

	// Devuelve un puntero a un string con la fecha y hora formateadas para usar en
	// los mensajes de log.

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);
	rtcDateTime.year -= 2000;
	snprintf_P( nowStr,sizeof(nowStr), PSTR("%02d/%02d/%02d %02d:%02d:%02d\0"),rtcDateTime.day,rtcDateTime.month,rtcDateTime.year,rtcDateTime.hour,rtcDateTime.min,rtcDateTime.sec );
	return(nowStr);
}
//------------------------------------------------------------------------------------
u08 u_readTermsw(u08 *pin)
{
	// El TERMSW se cablea a PD7.
	*pin = ( TERMSW_PIN & _BV(7) ) >> 7;
	return(TRUE);

}
//------------------------------------------------------------------------------------
void u_setConsignaDiurna ( void )
{
	// Una consigna es la activacion simultanea de 2 valvulas, en las cuales un
	// se abre y la otra se cierra.
	// Cierro la valvula 1
	// Abro la valvula 2
	// Para abrir una valvula debemos poner una fase 10.
	// Para cerrar es 01

	 MCP_outsPulse( systemVars.consigna.chVA , 1, 100 );	// Cierro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	 MCP_outsPulse( systemVars.consigna.chVB , 0, 100 );	// Abro la valvula 2

	 systemVars.consigna.consignaAplicada = CONSIGNA_DIURNA;

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_outputA1Disable();
	 MCP_outputA2Disable();
	 MCP_outputB1Disable();
	 MCP_outputB2Disable();

	 MCP_outputsSleep();
}
/*------------------------------------------------------------------------------------*/
void u_setConsignaNocturna ( void )
{
	// Abro la valvula 1
	// Cierro la valvula 2

	 MCP_outsPulse( systemVars.consigna.chVA , 0, 100 );	// Abro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	 MCP_outsPulse( systemVars.consigna.chVB , 1, 100 );	// Cierro la valvula 2

	 systemVars.consigna.consignaAplicada = CONSIGNA_NOCTURNA;

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_outputA1Disable();
	 MCP_outputA2Disable();
	 MCP_outputB1Disable();
	 MCP_outputB2Disable();

	 MCP_outputsSleep();

}
//------------------------------------------------------------------------------------
void u_debugPrint(u08 debugCode, char *msg, u16 size)
{

	if ( (systemVars.debugLevel & debugCode) != 0) {
		FreeRTOS_write( &pdUART1, msg, size );
	}
}
//------------------------------------------------------------------------------------
