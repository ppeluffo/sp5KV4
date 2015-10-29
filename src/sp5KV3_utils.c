/*
 * sp5KV3_utils.c
 *
 *  Created on: 27/10/2015
 *      Author: pablo
 */

#include <sp5KV3.h>

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

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
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
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
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

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();
	systemVars.pwrMode =  pwrMode;
	xSemaphoreGive( sem_SYSVars );
	return(TRUE);
}
//----------------------------------------------------------------------------------------
s08 u_configTimerPoll(char *s_tPoll)
{
u16 tpoll;

	tpoll = abs((u16) ( atol(s_tPoll) ));
	if ( tpoll < 15 ) { tpoll = 15; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();
	systemVars.timerPoll = tpoll;
	xSemaphoreGive( sem_SYSVars );

	return(TRUE);
}
//----------------------------------------------------------------------------------------
s08 u_configTimerDial(char *s_tDial)
{
u32 tdial;

	tdial = abs( (u32) ( atol(s_tDial) ));
	if ( tdial < 120 ) { tdial = 120; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
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

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	systemVars.pwrSave = modoPwrSave;
	if ( s_startTime != NULL ) { systemVars.pwrSaveStartTime = u_convertHHMM2min ( atol(s_startTime) ); }
	if ( s_endTime != NULL ) { systemVars.pwrSaveEndTime = u_convertHHMM2min ( atol(s_endTime) ); }

	xSemaphoreGive( sem_SYSVars );

}
//----------------------------------------------------------------------------------------
void u_configConsignas( u08 modo, char *s_horaConsDia,char *s_horaConsNoc,u08 chVA, u08 chVB )
{
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	switch(modo) {
	case CONSIGNA_OFF:
		systemVars.consigna.status = CONSIGNA_OFF;
		break;
	case CONSIGNA_ON:
		systemVars.consigna.status = CONSIGNA_ON;
		if ( s_horaConsDia != NULL ) { systemVars.consigna.horaConsDia =  u_convertHHMM2min ( atol(s_horaConsDia) ); }
		if ( s_horaConsNoc != NULL ) { systemVars.consigna.horaConsNoc =  u_convertHHMM2min ( atol(s_horaConsNoc) ); }
		if ( chVA != NULL ) { systemVars.consigna.chVA = chVA; }
		if ( chVB != NULL ) { systemVars.consigna.chVB = chVB; }
		break;
	}

	xSemaphoreGive( sem_SYSVars );

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
void u_setConsignaDiurna ( u16 ms_pulso )
{
	// Una consigna es la activacion simultanea de 2 valvulas, en las cuales un
	// se abre y la otra se cierra.
	// Cierro la valvula 1
	// Abro la valvula 2
	// Para abrir una valvula debemos poner una fase 10.
	// Para cerrar es 01

	 MCP_outsPulse( systemVars.consigna.chVA , 0, ms_pulso );	// Cierro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	 MCP_outsPulse( systemVars.consigna.chVB , 1, ms_pulso );	// Abro la valvula 2

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_output0Disable();
	 MCP_output1Disable();
	 MCP_output2Disable();
	 MCP_output3Disable();

	 MCP_outputsSleep();
}
/*------------------------------------------------------------------------------------*/
void u_setConsignaNocturna ( u16 ms_pulso )
{
	// Abro la valvula 1
	// Cierro la valvula 2

	 MCP_outsPulse( systemVars.consigna.chVA , 1, ms_pulso );	// Abro la valvula 1
	 vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	 MCP_outsPulse( systemVars.consigna.chVB , 0, ms_pulso );	// Cierro la valvula 2

	 // Dejo el sistema de salidas en reposo para que no consuma
	 MCP_output0Disable();
	 MCP_output1Disable();
	 MCP_output2Disable();
	 MCP_output3Disable();

	 MCP_outputsSleep();

}
/*------------------------------------------------------------------------------------*/
void u_clearWdg( u08 wdgId )
{
	// Pone el correspondiente bit del wdg en 0.
	systemWdg &= ~wdgId ;

}
/*------------------------------------------------------------------------------------*/
