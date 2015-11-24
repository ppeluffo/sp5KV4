/*
 *  sp5KFRTOS_mcp.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 *  Funciones del MCP23008 modificadas para usarse con FRTOS.
 */
//------------------------------------------------------------------------------------

#include <mcp_sp5K.h>
#include "FRTOS-IO.h"

// Funciones privadas del modulo MCP
static void pvMCP_init_MCP0(void);
static void pvMCP_init_MCP1(void);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
s08 MCP_write( u08 deviceId, u08 byteAddr, u08 value )
{
u08 regValue;
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos la direccion del dispositivo: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	// Por ultimo indico la direccion interna a leer
	// Por ultimo leemos 1 byte.
	xBytes = 1;
	regValue = value;
	xReturn = FreeRTOS_write(&pdI2C, &regValue, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( FALSE );
	}

	return(TRUE);

}
/*------------------------------------------------------------------------------------*/
s08 MCP_read( u08 deviceId, u08 byteAddr, u08 *retValue )
{
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	// Por ultimo leemos 1 byte.
	xBytes = 1;
	xReturn = FreeRTOS_read(&pdI2C, retValue, xBytes);
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( FALSE );
	}

	return(TRUE);

}
/*------------------------------------------------------------------------------------*/
s08 pvMCP_testAndSet( u08 deviceId, u08 byteAddr, u08 value, u08 bitMask )
{
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;
u08 regValue;
s08 retS = FALSE;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = deviceId;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos cuantos bytes queremos leer del dispositivo: largo.
	// En los MCP se lee y escribe de a 1 registro.
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// Ahora indicamos desde que posicion queremos leer: direccion
	val = byteAddr;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	// Por ultimo leemos.
	xBytes = 1;
	xReturn = FreeRTOS_read(&pdI2C, &regValue, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	// Modifico el registro
	if  (value == 0) {
		regValue &= ~BV(bitMask);
	} else {
		regValue |= BV(bitMask);
	}

	// Escribo en el MCP
	xBytes = 1;
	xReturn = FreeRTOS_write(&pdI2C, &regValue, xBytes);
	if (xReturn != xBytes ) {
		goto quit;
	}

	retS = TRUE;

quit:
	// Y libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);
	return(retS);
}
//------------------------------------------------------------------------------------
// Funciones particulares
//------------------------------------------------------------------------------------
void MCP_init(void)
{
	// inicializo los MCP para la configuracion de pines del HW sp5K.
	// Como accedo al bus I2C, debo hacerlo una vez que arranco el RTOS.

	// Inicializo los pines del micro como entradas para las interrupciones del MCP.
	cbi(MCP0_DDR, MCP0_BIT);
	cbi(MCP1_DDR, MCP1_BIT);

	pvMCP_init_MCP0();
	pvMCP_init_MCP1();

}
//------------------------------------------------------------------------------------
s08 MCP_queryDcd( u08 *pin)
{
// MCP23008 logic

s08 retS;
u08 regValue;

	// DCD es el bit1, mask = 0x02
	retS = MCP_read( MCP0_ADDR, MCP0_GPIO, &regValue);
	*pin = ( regValue & 0x02) >> 1;
	//*pin = ( regValue & _BV(1) ) >> 1;		// bit1, mask = 0x02
	return(retS);
}
//------------------------------------------------------------------------------------
s08 MCP_queryRi( u08 *pin)
{
// MCP23008 logic

s08 retS;
u08 regValue;

	// RI es el bit2, mask = 0x04
	retS = MCP_read( MCP0_ADDR, MCP0_GPIO, &regValue);
	*pin = ( regValue & 0x04) >> 2;
	//*pin = ( regValue & _BV(2) ) >> 1;		// bit2, mask = 0x04
	return(retS);
}
//------------------------------------------------------------------------------------
s08 MCP_queryTermsw( u08 *pin)
{
// MCP23008 logic

s08 retS;
u08 regValue;

	//  TERMSW es el bit 7, mask = 0x80
	retS = MCP_read( MCP0_ADDR, MCP0_GPIO, &regValue);
	*pin = ( regValue & 0x08) >> 7;
	//*pin = ( regValue & _BV(7) ) >> 1;		// bit7, mask = 0x80
	return(retS);
}
//------------------------------------------------------------------------------------
s08 MCP_queryDin0( u08 *pin)
{

s08 retS;
u08 regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*pin = ( regValue & 0x40) >> 6;
	return(retS);
}
//------------------------------------------------------------------------------------
s08 MCP_queryDin1( u08 *pin)
{

s08 retS;
u08 regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*pin = ( regValue & 0x20) >> 5;
	return(retS);
}
//------------------------------------------------------------------------------------
s08 MCP_query2Din( u08 *din0, u08 *din1 )
{

s08 retS;
u08 regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*din0 = ( regValue & 0x40) >> 6;
	*din1 = ( regValue & 0x20) >> 5;
	return(retS);
}
//------------------------------------------------------------------------------------
// Funciones particulares de OUTPUTS
//------------------------------------------------------------------------------------
s08 MCP_setOutsPhase(  u08 outId, u08 value )
{
// MCP23018	analog

	switch(outId) {
	case 0:
		// U12, J11 -> PH_A1
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_PHA1 ) );
		break;
	case 1:
		// U12, J8 -> PH_B1
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_PHB1 ) );
	case 2:
		// U11,J10 -> PH_A2
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATB, value, MCP1_PHA2 ) );
		break;
	case 3:
		// U11,J9 -> PH_B2
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_PHB2 ) );
		break;
	default:
		return(FALSE);
		break;
	}
}
//------------------------------------------------------------------------------------
s08 MCP_setOutsEnablePin(  u08 outId, u08 value )
{
// MCP23018	analog

	switch(outId) {
	case 0:
		// U12, J11 -> EN_A1
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_ENA1 ) );
		break;
	case 1:
		// U12, J8 -> EN_B1
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_ENB1 ) );
		break;
	case 2:
		// U11,J10 -> EN_A2
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_ENA2 ) );
		break;
	case 3:
		// U11,J9 -> EN_B2
		return( pvMCP_testAndSet( MCP1_ADDR, MCP1_OLATA, value, MCP1_ENB2 ) );
		break;
	default:
		return(FALSE);
		break;
	}

}
//------------------------------------------------------------------------------------
s08 MCP_outsPulse( u08 channel, u08 phase, u16 delay )
{
	// Genera un pulso en el canal 'channel', con la secuencia de fases 'phase' y
	// de duracion 'delay'
	// Deja el sistema en reposo ( sleep )

s08 retS = FALSE;

	MCP_outputsNoSleep();
	MCP_outputsNoReset();
	switch(channel) {
	case 0:
		if ( phase == 0 ) { retS = MCP_output0Phase_01(); }
		if ( phase == 1 ) { retS = MCP_output0Phase_10(); }
		MCP_output0Enable();
		vTaskDelay( ( TickType_t)( delay / portTICK_RATE_MS ) );
		MCP_output0Disable();
		break;
	case 1:
		if ( phase == 0 ) { retS = MCP_output1Phase_01(); }
		if ( phase == 1 ) { retS = MCP_output1Phase_10(); }
		MCP_output1Enable();
		vTaskDelay( ( TickType_t)( delay / portTICK_RATE_MS ) );
		MCP_output1Disable();
		break;
	case 2:
		if ( phase == 0 ) { retS = MCP_output2Phase_01(); }
		if ( phase == 1 ) { retS = MCP_output2Phase_10(); }
		MCP_output2Enable();
		vTaskDelay( ( TickType_t)( delay / portTICK_RATE_MS ) );
		MCP_output2Disable();
		break;
	case 3:
		if ( phase == 0 ) { retS = MCP_output3Phase_01(); }
		if ( phase == 1 ) { retS = MCP_output3Phase_10(); }
		MCP_output3Enable();
		vTaskDelay( ( TickType_t)( delay / portTICK_RATE_MS ) );
		MCP_output3Disable();
		break;
	default:
		retS = FALSE;
		break;
	}

	MCP_outputsSleep();
	return(retS);

}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pvMCP_init_MCP0(void)
{
	// inicializa el MCP23008 de la placa de logica
	// NO CONTROLO ERRORES.

u08 data;
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	// Luego indicamos el periferico i2c en el cual queremos leer
	val = MCP0_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
	// Luego indicamos en que posicion del periferico queremos leer: largo
	val = 1;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion

	// MCP0_IODIR: inputs(1)/outputs(0)
	val = MCP0_IODIR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	data |= ( BV(MCP0_GPIO_IGPRSDCD) | BV(MCP0_GPIO_IGPRSRI) );
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_IPOL: polaridad normal
	val = MCP0_IPOL;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_GPINTEN: inputs interrupt on change.
	val = MCP0_GPINTEN;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	//data |= ( BV(MCP_GPIO_IGPRSDCD) | BV(MCP_GPIO_IGPRSRI) | BV(MCP_GPIO_ITERMPWRSW) );
	data |=  BV(MCP0_GPIO_IGPRSDCD);
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_INTCON: Compara contra su valor anterior
	val = MCP0_INTCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_IOCON: INT active H
	val = MCP0_IOCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 2;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// MCP0_GPPU: pull-ups
	// Habilito los pull-ups en DCD
	val = MCP0_GPPU;
//	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
//	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// TERMPWR ON
	// Al arrancar prendo la terminal para los logs iniciales.
	val = MCP0_OLAT;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	data |= BV(MCP0_GPIO_OTERMPWR);	// TERMPWR = 1
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	FreeRTOS_write( &pdUART1, "MCP0 init OK\r\n\0", sizeof("MCP0 init OK\r\n\0") );
}
//------------------------------------------------------------------------------------
static void pvMCP_init_MCP1(void)
{
	// Inicializo el MCP23018 de la placa analogica

u08 data;
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	// Lo primero es obtener el semaforo
	FreeRTOS_ioctl(&pdI2C,ioctlOBTAIN_BUS_SEMPH, NULL);
	val = MCP1_ADDR;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_DEVADDRESS, &val);

	// IOCON
	val = MCP1_IOCON;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0x63; // 0110 0011
	//                      1->INTCC:Read INTCAP clear interrupt
	//                     1-->INTPOL: INT out pin active high
	//                    0--->ORDR: Active driver output. INTPOL set the polarity
	//                   0---->X
	//                 0----->X
	//                1------>SEQOP: sequential disabled. Address ptr does not increment
	//               1------->MIRROR: INT pins are ored
	//              0-------->BANK: registers are in the same bank, address sequential
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// DIRECCION
	// 0->output
	// 1->input
	val = MCP1_IODIRA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0x80; // 1000 0000 ( GPA0..GPA6: outputs, GPA7 input )
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_IODIRB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0x64; // 0110 0100
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// PULL-UPS
	// 0->disabled
	// 1->enabled
	val = MCP1_GPPUA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0xFF; // 1111 1111
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_GPPUB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0xFF; // 1111 1111
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// Valores iniciales de las salidas en 0
	val = MCP1_OLATA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0x00;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	val = MCP1_OLATB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0x00;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// GPINTEN: inputs interrupt on change.
	// Habilito que DIN0/1 generen una interrupcion on-change.
	// El portA no genera interrupciones
	val = MCP1_GPINTENA;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//data = 0x60; // 0110 0000
	//data |= ( BV(MCP1_GPIO_DIN0) | BV(MCP1_GPIO_DIN1) );
	val = MCP1_GPINTENB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	//
	// DEFVALB: valores por defecto para comparar e interrumpir
	//data = 0;
	//status = pvMCP_write( MCP1_DEFVALB, MCP_ADDR2, 1, &data);

	// INTCON: controlo como comparo para generar la interrupcion.
	// Con 1, comparo contra el valor fijado en DEFVAL
	// Con 0 vs. su valor anterior.
	val = MCP1_INTCONB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	//data |= ( BV(MCP1_GPIO_DIN0) | BV(MCP1_GPIO_DIN1) );
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);
	// Borro interrupciones pendientes
	val = MCP1_INTCAPB;
	FreeRTOS_ioctl(&pdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
	data = 0;
	xBytes = sizeof(data);
	xReturn = FreeRTOS_write(&pdI2C, &data, xBytes);

	// libero el semaforo.
	FreeRTOS_ioctl(&pdI2C,ioctlRELEASE_BUS_SEMPH, NULL);

	FreeRTOS_write( &pdUART1, "MCP1 init OK\r\n\0", sizeof("MCP1 init OK\r\n\0") );

}
//------------------------------------------------------------------------------------
