#include <TinyGPS++.h>
#define BAUD_RATE 9600

// Función de configuración de la USART en modo asincronico(UART).
void UART_begin(void);    

//Función para la recepción de caracteres.
unsigned char UART_read(void); 

//Función para la transmisión de caracteres.
void UART_transmit(unsigned char);

//Función para verificar si hay datos para leer en serial1
bool UART_available();

void UART_begin() {
  //Se configura Baud Rate para el UART del Serial1
  uint16_t ubrr_value = F_CPU / 16 / BAUD_RATE - 1;
  UBRR1H = (ubrr_value >> 8);
  UBRR1L = ubrr_value;

  // Se habilita el Tx y Rx del serial 1
  UCSR1B = (1 << RXEN1) | (1 << TXEN1);
  // Se estable el tamaño de datos a 8 bits
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



