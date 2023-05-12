#include <TinyGPS++.h>

#include <UART.h>
#define BAUD_RATE 9600


TinyGPSPlus gps;

void loop() {
  // Leer datos del GPS
  while (UART_available()) {
    char data = UART_receive();

    // Procesar los datos del GPS
    if (gps.encode(data)) {
      displayInfo();
    }
  }
}

void setup() {
  Serial.begin(57600);      // Inicializar la comunicación serial para imprimir resultados
  UART_begin();            // Inicializar la comunicación serial personalizada para Serial1

  // Esperar 2 segundos para que el GPS se inicie correctamente
  delay(2000);
}

void displayInfo() {
  if (gps.location.isUpdated()) {
    if (gps.location.isValid()) {
      Serial.print("Latitud: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", Longitud: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Datos de ubicación inválidos");
    }
  }
}
