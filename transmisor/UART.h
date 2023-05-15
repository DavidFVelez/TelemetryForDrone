#ifndef UART_H
#define UART_H

#include <TinyGPS++.h>
#define BAUD_RATE 9600

// Función de configuración de la USART en modo asincronico(UART).
void UART_begin(void);    
//Función para verificar si hay datos para leer en serial1
bool UART_available();
//Función para la recepción de caracteres.
unsigned char UART_receive(void); 
//Función para la transmisión de caracteres.
void UART_transmit(unsigned char);

void UART_begin() {
  //Se configura Baud Rate(9600) para el UART del Serial1
  uint16_t valUBRR1H = F_CPU / 16 / BAUD_RATE - 1;
  UBRR1H = (valUBRR1H >> 8);
  UBRR1L = valUBRR1H;

  // Se habilita el Tx y Rx del serial 1
  UCSR1B = (1 << RXEN1) | (1 << TXEN1);

  //UCSZ11 y UCSZ10 en 1: tamaño de datos de 8 bits.
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

bool UART_available() {
  //Se verifica si hay datos para leer en el serial1
  return (UCSR1A & (1 << RXC1));
}

unsigned char UART_receive() {
  
  // Se espera a que en el buffer de recepción lleguen datos
  while (!(UCSR1A & (1 << RXC1)))
    ;
  // Se retorna el dato que se encuentre en el buffer
  return UDR1;
}

void UART_transmit(unsigned char data) {
    // Mientras el buffer de transmisión este vacío no se realiza ninguna acción 
  while (!(UCSR1A & (1 << UDRE1)))
    ;
  // Si hay datos en el buffer de transmisión se envían a través del serial1
  UDR1 = data;
}

#endif /* UART_H */



