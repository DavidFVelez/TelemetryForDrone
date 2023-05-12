#include <avr/io.h>
#include <avr/interrupt.h>


// Funcion de configuración de la USART en modo asincronico(UART).
void UART_begin(void);              

//Función para la recepción de caracteres.
unsigned char UART_read(void); 

//Función para la transmisión de caracteres.
void UART_transmit(unsigned char);

//Función para la transmision de strings 
void UART_transmitStr(char*);  

#define F_OSC 16000000
#define BAUD 9600

void UART_begin(void) {

  UBRR0 = (F_OSC / 8 / BAUD) - 1;  //BAUD RATE de 9600 a 16Mhz
  // UBRR0 = 103;
  UCSR0A = (0 << TXC0) | (1 << U2X0) | (0 << MPCM0);
  /*
  TXC0: 1 datos por leer, 0 sin datos por leer.
  U2X0 en 1:  modo asíncrono de velocidad doble.
  MPCM0 en 0: deshabilita el modo comunicación multiprocesador.
  */
  UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (0 << UCSZ02);
  /* 
  RXCIE0: habilita interrupciones por recepción de datos.
  TXCIE0 en 0: deshabilita interrupciones por transmisión de datos.
  UDRIE0 en 0: deshabilita interrupciones si el buffer de transmisión.
  esta vacío.
  RXEN0 y TXEN0 en 1: habilitan el receptor y transmisor.
  UCSZ02 junto a UCSZ01 y UCSZ00 determinan el tamaño de los
  bits de datos.
  TXB80 en 0: se deshabilita transmisión de datos de 9 bits y
  el noveno bit del registro de datos de transmisión.
  */
  UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);
  /* 
  UMSEL01 y UMSEL00 en 0: modo de operación asincrona.
  UPM01 Y UPM00 en 0: paridad desactivada.
  USBS0 en 0: un solo bit de stop.
  UCSZ01 y UCSZ00 en 1: tamaño de datos de 8 bits.
  UCPOL0 en 0: se consideran los flancos de bajada para muestrear.
  */

  DDRD &= ~(1 << PD2);
  DDRD |= (1 << PD3); // Configurar pin 3 (PD3) como salida

  //DDRD |= (1 << 1);   //Se establece el pin D1 como Tx, bit en 1.
  //DDRD &= ~(1 << 0);  //Se establece el pin D0 como Rx, bit en 0.
   //pinMode(3, OUTPUT);  // Configurar pin 3 como salida (TX)
   //pinMode(4, INPUT);   // Configurar pin 4 como entrada (RX)
}

unsigned char UART_read(void) {
  //Cuando el bit RXC0(7) esta en 1 significa que se recibió un dato, y esta almacenado en el registro UDR0 por lo tanto se retorna.
  if (UCSR0A & (1 << RXC0))  
    return UDR0;          
  else
    return 0;
}

void UART_transmit(unsigned char data) {
  //Se espera a que el buffer de transmisión(UDR0) este vacío(UDRE0(5) en 1)
  while (!(UCSR0A & (1 << UDRE0)))
    ; 
  //Se asigna el dato y se envía
  UDR0 = data; 
}

void UART_transmitStr(char* data)  
{
  while (*data != 0x00) {
    UART_transmit(*data);  
    data++;                                  
  }
  /*Se itera sobre cada caracter del string y lo transmite a través de la UART. En cada iteración, el puntero avanza en memoria
  hasta llegar al carácter nulo, que indica el final del string y el fin del loop.*/
}

// #endif // UART_H
