/*
 * FRTOS-IO.c
 *
 *  Created on: 2/10/2015
 *      Author: pablo
 *
 */

#include <sp5KV3.h>
#include "FRTOS-IO.h"

//------------------------------------------------------------------------------------
// FUNCIONES GENERALES FreeRTOS ( son las que usa la aplicacion )
//------------------------------------------------------------------------------------
Peripheral_Descriptor_t FreeRTOS_open(const u08 port, const u32 flags)
{

	switch(port) {

	case pUART0:
		pdUART0.portId = port;
		FreeRTOS_UART_open (&pdUART0, flags);
		break;
	case pUART1:
		pdUART1.portId = port;
		FreeRTOS_UART_open (&pdUART1, flags);
		break;
	case pI2C:
		pdI2C.portId = port;
		FreeRTOS_I2C_open (&pdI2C, flags);
		break;
	}

	// no retorno nada y el compilador se encarga.

}
//------------------------------------------------------------------------------------
portBASE_TYPE FreeRTOS_ioctl( Peripheral_Descriptor_t const xPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t *pxPeripheralControl = ( Peripheral_Control_t * ) xPeripheral;

	switch(pxPeripheralControl->portId)
	{
		case pUART0:
			FreeRTOS_UART_ioctl( xPeripheral, ulRequest, pvValue );
			break;
		case pUART1:
			FreeRTOS_UART_ioctl( xPeripheral, ulRequest, pvValue );
			break;
		case pI2C:
			FreeRTOS_I2C_ioctl( xPeripheral, ulRequest, pvValue );
			break;
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES DE UART PROVISTAS AL FREERTOS
//------------------------------------------------------------------------------------
portBASE_TYPE FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl, const u32 flags )

{
	/* Los dispositivos tipo UART requieren inicializar sus queues y un semaforo.
	 * Como es invocada antes de arrancar el RTOS y fuera de cualquier task, las colas y los
	 * semaforos no quedan definidos dentro de ningun stack.
	 *
	 */

int UARTx;
UART_device_control_t *pxNewUart;

	// Asigno las funciones particulares ed write,read,ioctl
	pxPeripheralControl->write = FreeRTOS_UART_write;
	pxPeripheralControl->read = FreeRTOS_UART_read;
	pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

	// Creo el semaforo del bus
	// En el caso de las UART el semaforo es de c/u.
	pxPeripheralControl->xBusSemaphore = xSemaphoreCreateMutex();
	pxPeripheralControl->xBlockTime = (50 / portTICK_RATE_MS );

	pxNewUart = ( int8_t * ) pvPortMalloc( sizeof(UART_device_control_t ));
	( flags && UART_RXFIFO) ?  ( pxNewUart->rxBufferType = FIFO ) : (pxNewUart->rxBufferType = QUEUE );
	( flags && UART_TXFIFO) ?  ( pxNewUart->txBufferType = FIFO ) : (pxNewUart->txBufferType = QUEUE );

	// Creo las estructuras ( queues) de TX/RX
	// Asigno los tamanios
	// Tipo de estructura de datos
	switch( pxPeripheralControl->portId ) {
	case pUART0:
		pxNewUart->rxBufferLength = UART0_RXBUFFER_LEN;
		pxNewUart->txBufferLength = UART0_TXBUFFER_LEN;
		// Creo las estructuras
		// RX
		switch ( pxNewUart->rxBufferType ) {
		case QUEUE:
			// Las queue no pueden ser mayores a 256 bytes.
			if ( pxNewUart->rxBufferLength > 0xFF )
				pxNewUart->rxBufferLength = 0xFF;
			pxNewUart->rxStruct = xQueueCreate( pxNewUart->rxBufferLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
			break;
		case FIFO:
			pxNewUart->rxStruct = xFifoCreate( pxNewUart->rxBufferLength, NULL );
			break;
		}
		// TX
		switch ( pxNewUart->txBufferType ) {
		case QUEUE:
			// Las queue no pueden ser mayores a 256 bytes.
			if ( pxNewUart->txBufferLength > 0xFF )
				pxNewUart->txBufferLength = 0xFF;
			pxNewUart->txStruct = xQueueCreate( pxNewUart->txBufferLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
			break;
		case FIFO:
			pxNewUart->txStruct = xFifoCreate( pxNewUart->txBufferLength, NULL );
			break;
		}
		break;

	case pUART1:
		pxNewUart->rxBufferLength = UART1_RXBUFFER_LEN;
		pxNewUart->txBufferLength = UART1_TXBUFFER_LEN;
		// Creo las estructuras
		// RX
		switch ( pxNewUart->rxBufferType  ) {
		case QUEUE:
			// Las queue no pueden ser mayores a 256 bytes.
			if ( pxNewUart->rxBufferLength > 0xFF )
				pxNewUart->rxBufferLength = 0xFF;
			pxNewUart->rxStruct = xQueueCreate( pxNewUart->rxBufferLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
			break;
		case FIFO:
			pxNewUart->rxStruct = xFifoCreate( pxNewUart->rxBufferLength, NULL );
		}
		// TX
		switch ( pxNewUart->txBufferType ) {
		case QUEUE:
			// Las queue no pueden ser mayores a 256 bytes.
			if ( pxNewUart->txBufferLength > 0xFF )
				pxNewUart->txBufferLength = 0xFF;
			pxNewUart->txStruct = xQueueCreate( pxNewUart->txBufferLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
			break;
		case FIFO:
			pxNewUart->txStruct = xFifoCreate( pxNewUart->txBufferLength, NULL );
			break;
		}
		break;
	}

	if( pxNewUart != NULL )
	{
		pxPeripheralControl->phDevice = pxNewUart;
	}

	// Inicializo el puerto.
	UARTx = pxPeripheralControl->portId;
	pvUARTInit(UARTx);

	// UARTS enable ( use ioctl )
	sbi(UARTCTL_DDR, UARTCTL);
	cbi(UARTCTL_PORT, UARTCTL);

}
//------------------------------------------------------------------------------------
size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision.
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar;
char *p;
int UARTx;
size_t bytes2tx;
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
UART_device_control_t *pUart;

	pUart = pxPeripheralControl->phDevice;
	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;
	if ( pUart->txBufferLength < xBytes ) {
		bytes2tx = pUart->txBufferLength;
	}

	// Espero el semaforo en forma persistente.
	while ( xSemaphoreTake(pxPeripheralControl->xBusSemaphore, ( TickType_t ) 10 ) != pdTRUE )
		taskYIELD();

	// Trasmito.
	// Espero que halla lugar en la cola de trasmision. ( La uart se va limpiando al trasmitir )
	if ( pUart->txBufferType == QUEUE ) {
		while  ( uxQueueSpacesAvailable( pUart->txStruct ) < bytes2tx )
			taskYIELD();
	} else {
		while  ( uxFifoSpacesAvailable( pUart->txStruct ) < bytes2tx )
			taskYIELD();
	}

	// Cargo el buffer en la cola de trasmision.
	p = pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {
		cChar = *p;
		if ( pUart->txBufferType == QUEUE ) {
			xQueueSend( pUart->txStruct, &cChar, ( TickType_t ) 10  );
		} else {
			xFifoSend( pUart->txStruct, &cChar, ( TickType_t ) 10  );
		}
		p++;
	}

	// Luego inicio la trasmision invocando la interrupcion.
	UARTx = pxPeripheralControl->portId;
	vUartInterruptOn(UARTx);

	xSemaphoreGive( pxPeripheralControl->xBusSemaphore );

	return xBytes;	// Puse todos los caracteres en la cola.


}
//------------------------------------------------------------------------------------
size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{

	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xBytesReceived = 0U;
portTickType xTicksToWait;
xTimeOutType xTimeOut;
UART_device_control_t *pUart;

	pUart = pxPeripheralControl->phDevice;

	xTicksToWait = pxPeripheralControl->xBlockTime;
	vTaskSetTimeOutState( &xTimeOut );

	/* Are there any more bytes to be received? */
	while( xBytesReceived < xBytes )
	{
		/* Receive the next character. */
		if ( pUart->rxBufferType == QUEUE ) {
			if( xQueueReceive( pUart->rxStruct, &( pvBuffer[ xBytesReceived ] ), xTicksToWait ) == pdPASS ) {
				xBytesReceived++;
			}
		} else {
			if( xFifoReceive( pUart->rxStruct, &( pvBuffer[ xBytesReceived ] ), xTicksToWait ) == pdPASS ) {
				xBytesReceived++;
			}
		}

		/* Time out has expired ? */
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return xBytesReceived;

}
//------------------------------------------------------------------------------------
portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
portBASE_TYPE xReturn = pdPASS;
UART_device_control_t *pUart;

	pUart = pxPeripheralControl->phDevice;

	switch( ulRequest )
	{
		case ioctlUART_ENABLE:
			cbi(UARTCTL_PORT, UARTCTL);
			break;
		case ioctlUART_DISABLE:
			sbi(UARTCTL_PORT, UARTCTL);
			break;
		case ioctlOBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(pxPeripheralControl->xBusSemaphore, ( TickType_t ) 10 ) != pdTRUE )
				taskYIELD();
			break;
		case ioctlRELEASE_BUS_SEMPH:
			xSemaphoreGive( pxPeripheralControl->xBusSemaphore );
			break;
		case ioctlSET_TIMEOUT:
			pxPeripheralControl->xBlockTime = pvValue;	// REVISAR
			break;
		case ioctl_UART_CLEAR_RX_BUFFER:
			if ( pUart->rxBufferType == QUEUE) {
				xQueueReset(pUart->rxStruct);
			} else {
				xFifoReset(pUart->rxStruct);
			}
			break;
		case ioctl_UART_CLEAR_TX_BUFFER:
			if ( pUart->txBufferType == QUEUE) {
				xQueueReset(pUart->txStruct);
			} else {
				xFifoReset(pUart->txStruct);
			}
			break;
		default :
			xReturn = pdFAIL;
			break;
	}
	return xReturn;

}
//------------------------------------------------------------------------------------
void pvFreeRTOS_UART1_writeChar (char *c)
{
	// Funcion intermedia necesaria por cmdline para escribir de a 1 caracter en consola
	// El tema es que el prototipo de funcion que requiere cmdlineSetOutputFunc no se ajusta
	// al de FreeRTOS_UART_write, por lo que defino esta funcion intermedia.

	FreeRTOS_UART_write(&pdUART1,&c, sizeof(char));
}
//------------------------------------------------------------------------------------
char *FreeRTOS_UART_getFifoPtr(Peripheral_Control_t *UART)
{
	// Retorna un puntero al comienzo de buffer de la fifo de una UART.
	// Se usa para imprimir dichos buffers
	// Funcion PELIGROSA !!!

UART_device_control_t *uartDevice;
fifo_handle_s *uartFifo;
char *p;

	uartDevice = (UART_device_control_t *) UART->phDevice;
	if ( uartDevice->rxBufferType != FIFO )
		return(NULL);

	uartFifo = (fifo_handle_s *) uartDevice->rxStruct;
	p = uartFifo->buff;
	return(p);
}
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// FUNCIONES DE I2C ( TWI)
//------------------------------------------------------------------------------------
portBASE_TYPE FreeRTOS_I2C_open( Peripheral_Control_t * const pxPeripheralControl, const u32 flags )
{

	// Todos los dispositivos I2C comparten el mismo semaforo por lo tanto lo pongo como una
	// variable estatica y paso el puntero a los nuevos dispositivos.

I2C_device_control_t *pxNewI2C;

		// Asigno las funciones particulares ed write,read,ioctl
		pxPeripheralControl->write = FreeRTOS_I2C_write;
		pxPeripheralControl->read = FreeRTOS_I2C_read;
		pxPeripheralControl->ioctl = FreeRTOS_I2C_ioctl;

		// Creo el semaforo del bus I2C
		pxPeripheralControl->xBusSemaphore = xSemaphoreCreateMutex();
		pxPeripheralControl->xBlockTime = (50 / portTICK_RATE_MS );
		//
		// Todas las operaciones son por poleo por lo que no uso buffers extra
		pxNewI2C = ( int8_t * ) pvPortMalloc( sizeof(I2C_device_control_t ));
		pxPeripheralControl->phDevice = pxNewI2C;
		// Abro e inicializo el puerto I2C solo la primera vez que soy invocado
		i2c_init();
}
//------------------------------------------------------------------------------------
size_t FreeRTOS_I2C_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
I2C_device_control_t *pI2C;
size_t xReturn = 0U;

	pI2C = pxPeripheralControl->phDevice;

#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("FRTOS_I2C_WR: 0x%02x,0x%02x,0x%02x\r\n\0"),pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress);
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ( I2C_masterWrite(pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress, (char *)pvBuffer, xBytes) == TRUE ) {
		xReturn = xBytes;
	}
	return(xReturn);
}
//------------------------------------------------------------------------------------
size_t FreeRTOS_I2C_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
I2C_device_control_t *pI2C;
size_t xReturn = 0U;

	pI2C = pxPeripheralControl->phDevice;

#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("FRTOS_I2C_RD: 0x%02x,0x%02x,0x%02x\r\n\0"),pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress);
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
	if ( I2C_masterRead(pI2C->devAddress, pI2C->byteAddressLength, pI2C->byteAddress, (char *)pvBuffer, xBytes) == TRUE ) {
		xReturn = xBytes;
	}
	return(xReturn);
}
//------------------------------------------------------------------------------------
portBASE_TYPE FreeRTOS_I2C_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
portBASE_TYPE xReturn = pdPASS;
I2C_device_control_t *pI2C;
u16 *p;

		pI2C = pxPeripheralControl->phDevice;
		p = pvValue;

#ifdef DEBUG_I2C
		snprintf_P( debug_printfBuff,CHAR128,PSTR("FRTOS_I2C_IOCTL: 0x%02x,0x%02x\r\n\0"),(u08)ulRequest, (u08)(*p));
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
#endif
		switch( ulRequest )
		{
			case ioctlOBTAIN_BUS_SEMPH:
				// Espero el semaforo en forma persistente.
				while ( xSemaphoreTake(pxPeripheralControl->xBusSemaphore, ( TickType_t ) 10 ) != pdTRUE )
					taskYIELD();
				break;
			case ioctlRELEASE_BUS_SEMPH:
				xSemaphoreGive( pxPeripheralControl->xBusSemaphore );
				break;
			case ioctlSET_TIMEOUT:
				pxPeripheralControl->xBlockTime = *p;
				break;
			case ioctl_I2C_SET_DEVADDRESS:
				pI2C->devAddress = (int8_t)(*p);
				break;
			case ioctl_I2C_SET_BYTEADDRESS:
				pI2C->byteAddress = (u16)(*p);
				break;
			case ioctl_I2C_SET_BYTEADDRESSLENGTH:
				pI2C->byteAddressLength = (int8_t)(*p);
				break;
			default :
				xReturn = pdFAIL;
				break;
		}
		return xReturn;

}
//------------------------------------------------------------------------------------
