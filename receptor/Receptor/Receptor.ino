// Receptor
#include <SPI.h>
#include <nRF24L01.h>  //Librería para el modulo RF RF24L01
#include <RF24.h>

#define PIN_CE 8    //
#define PIN_CSN 10  //

const unsigned char direccionReceptor[5] = { 'T', 'L', 'M', 'T', 'R' };    //Dirección de recepción
const unsigned char direccionTransmisor[5] = { 'T', 'L', 'M', 'T', 'T' };  //Dirección de transmisión
RF24 recepcionDatos(PIN_CE, PIN_CSN);
uint8_t contTrama = 0;
uint8_t recibido = 0;
uint8_t modoRx = 1;       //Activación del modo Rx o Tx
float datosRecibidos[8];  //Para guardado de segundo paquete
float auxRecibidos[8];    //Para guardado del primer paquete
unsigned long int tiempoAnterior = 0;
float verifFrame[4] = { 1234.88, 4321.88, 6789.99, 9876.99 };  //Arreglo para verificar si los paquetes contienen estos valores definidos como protocolo


void obtenerDatos();
void mostrarDatos();
void enviarDatos();
void delayMillis(unsigned long int time);


void setup() {

  Serial.begin(115200);
  if (recepcionDatos.begin() != NULL) {
    // Serial.println("RECEPTOR INICIALIZADO\n");
    // Serial.println("|Lat      |Lng       |aX        |aY        |aZ        |gX        |gY        |gZ        |mX        |mY        |mZ        |Otro     |");

    recepcionDatos.openReadingPipe(1, direccionReceptor);  // Se definine canal y dirección para lectura de datos
    recepcionDatos.setDataRate(RF24_250KBPS);              // Se define la velocidad de recepcion de datos
    recepcionDatos.openWritingPipe(direccionTransmisor);   // Se define canal para tansmisión
    recepcionDatos.setRetries(15, 10);                     // Número de reintentos y  tiempo de espera entre reintentos
    recepcionDatos.startListening();                       // Se establece el módulo como receptor
    // Serial.println("Escuchando canal: ");
  } else
    Serial.println("Error en la inicialización del módulo");
}

void loop() {
  // Verificamos el modo en el que se encuentra el modulo si Rx o Tx.
  if (modoRx) {
    // Llamamos a la función obtenerDatos para leer y guardar datos del canal
    obtenerDatos();
  } else {
    //Si el modo es Tx llamamos la función enviarDatos para transmitir los PWM de los motores
    enviarDatos();
  }
}
//Función que permite obtener recibir los datos enviados por el transmisor
//Return Nada.
void obtenerDatos() {
  //Vericar si hay datos disponibles
  if (recepcionDatos.available()) {
    recepcionDatos.read(datosRecibidos, sizeof(datosRecibidos));  //Se leen los datos y se guardan
    if (datosRecibidos[0] == 1234.88) {                           //Se verifica si ha llegado el primer paquete, al comparar con el flotante definido en el protocolo
      recibido = 1;
    } else {
      recibido = 0;
    }

    if (datosRecibidos[0] == 6789.99) {  //Se verifica si ha llegado el segundo paquete, al comparar con el flotante definido en el protocolo
      contTrama = 2;
    }
    mostrarDatos();
  } else {
    //Cuando se han recibido los 2 paquetes se procede enviar las señales de control(PWMs)
    recepcionDatos.openReadingPipe(1, direccionReceptor);  // Se definine canal y dirección para lectura de datos
    recepcionDatos.setDataRate(RF24_250KBPS);              // Se define la velocidad de recepcion de datos
    recepcionDatos.startListening();
    delayMillis(50);
  }
}
//Función permite imprimir los datos recibidos
//Return Nada.
void mostrarDatos() {

  //Se verifica si llego un primer paquete y se guardan los datos en un arreglo auxiliar para posteriormente imprimirlos
  if (recibido && datosRecibidos[0] == verifFrame[0] && datosRecibidos[7] == verifFrame[1]) {
    for (int i = 0; i < sizeof(datosRecibidos) / 4; i++) {
      auxRecibidos[i] = datosRecibidos[i];
    }
  }
  //Se verifica si llego un segundo paquete
  if (contTrama == 2 && datosRecibidos[0] == verifFrame[2] && datosRecibidos[7] == verifFrame[3]) {
    {
      //Se imprimen los datos del paquete1 recibidos en el serial, para comprobación y generación de graficas
      for (int j = 0; j < sizeof(datosRecibidos) / 4 - 2; j++) {
        Serial.print(auxRecibidos[j + 1], 6);
        Serial.print(",");
      }
      //Se imprimen los datos del paquete2 recibidos en el serial, para comprobación y generación de graficas
      for (int k = 0; k < sizeof(datosRecibidos) / 4 - 2; k++) {
        Serial.print(datosRecibidos[k + 1], 6);
        if(k == 5){
          Serial.print('\n');
        }else{ 
        Serial.print(","); }
      }
      
      delayMillis(50);
      //Se frena la lectura da datos
      recepcionDatos.stopListening();
      modoRx = 0;
      contTrama = 0;
    }
  }
}

//Función para enviar las señales de control(pwm1,pwm2,pwm3 y pwm4)
//Return Nada
void enviarDatos() {
  //Se crea un array de 8 valores enteros, cada valor corresponde a un valor de pwm que se enviaran para el control de los motores
  uint8_t pwm[4];
  // En este caso se generan los 4 pwm aleatoriamente
  pwm[0] = random(100);
  pwm[1] = random(100);
  pwm[2] = random(100);
  pwm[3] = random(100);

  //Se imprimen los valrores por consola para comprobación de valores
  // Serial.print(pwm[0]);
  // Serial.print(pwm[1]);
  // Serial.print(pwm[2]);
  // Serial.print(pwm[3]);
  // Serial.println();

  //Se envia el paquete de enteros
  recepcionDatos.write(&pwm, sizeof(pwm));

  delayMillis(10);
  //Se reactiva el modo de recepción
  modoRx = 1;
  recepcionDatos.startListening();
}

//Función para generar tiempos de espera
void delayMillis(unsigned long int time) {
  unsigned long int beforeTime = millis();
  unsigned long int currentTime = millis();
  while (currentTime - beforeTime < time) {
    currentTime = millis();
  }
}
