// Transmisor

// Librerias estandar C
#include <stdio.h>
#include <stdint.h>
// Librerías para modulo IMU
// #include "MPU9250.h"

//  #include "I2Cdev.h"
#include "MPU9250.h"
#include "Wire.h"
// Librerías GPS
#include <TinyGPS++.h>
// #include <SoftwareSerial.h>
#include "UART.h"
// Librerías para módulo NRF24
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Declaraciones para la IMU

// #define MEASSURE_PERIOD_MS 100
// uint8_t calibrationSamples = 10;
// MPU9250 mpu;

// // GYROSCOPE
// struct GyrData
// {
//     float pitch;
//     float roll;
//     float yaw;
// };
// struct GyrOffset
// {
//     float pitch;
//     float roll;
//     float yaw;
// };

// // ACELEROMETER
// struct AccData
// {
//     float x;
//     float y;
//     float z;
// };
// struct AccOffset
// {
//     float x;
//     float y;
//     float z;
// };

// // MAGNETOMETER
// struct MagData
// {
//     float x;
//     float y;
//     float z;
// };
// struct MagOffset
// {
//     float x;
//     float y;
//     float z;
// };

// // GYROSCOPE
// GyrData gyroscopeData;
// GyrOffset gyroscopeOffset;

// // ACELEROMETER
// AccData accelerometerData;
// AccOffset accelerometerOffset;

// // MAGNETOMETER
// MagData magnetometerData;
// MagOffset magnetometerOffset;

// //**********
// //                                    CALIBRATION
// //**********

// /*
//  * @brief this function is use to calibrate the acelerometer, it is necessary to leave the IMU over a flat surface.
//  * basically it takes the average of the acelerometer readings over a period of time, and use it for the offset.
//  * @param void
//  * @return void
//  */
// void calibrateAcelerometer()
// {

//     Serial.println("Calibrating Acceleromter ...");
//     delay(1000);
//     float tempX = 0, tempY = 0, tempZ = 0;
//     int auxSamples = 0;

//     while (1)
//     {
//         if (mpu.update())
//         {
//             static uint32_t prev_ms = millis();
//             if (millis() > prev_ms + 25)
//             {
//                 tempX += mpu.getAccX();
//                 tempY += mpu.getAccY();
//                 tempZ += mpu.getAccZ();
//                 prev_ms = millis();
//                 auxSamples++;
//             }
//         }
//         if (auxSamples == calibrationSamples)
//             break;
//     }
//     accelerometerOffset.x = tempX / (float)calibrationSamples;
//     accelerometerOffset.y = tempY / (float)calibrationSamples;
//     accelerometerOffset.z = tempZ / (float)calibrationSamples;
// }

// /*
//  * @brief this function is use to calibrate the gyroscope, it is necessary to leave the IMU over a flat surface.
//  * basically it takes the average of the gyroscope readings over a period of time, and use it for the offset.
//  * @param void
//  * @return void
//  */
// void calibrateGyroscope()
// {

//     Serial.println("Calibrating gyroscope ...");
//     delay(5000);
//     float tempX = 0, tempY = 0, tempZ = 0;
//     int auxSamples = 0;

//     while (1)
//     {
//         if (mpu.update())
//         {
//             static uint32_t prev_ms = millis();
//             if (millis() > prev_ms + 25)
//             {
//                 tempX += mpu.getPitch();
//                 tempY += mpu.getRoll();
//                 tempZ += mpu.getYaw();
//                 prev_ms = millis();
//                 auxSamples++;
//             }
//         }
//         if (auxSamples == calibrationSamples)
//             break;
//     }
//     gyroscopeOffset.pitch = tempX / (float)calibrationSamples;
//     gyroscopeOffset.roll = tempY / (float)calibrationSamples;
//     gyroscopeOffset.yaw = tempZ / (float)calibrationSamples;
// }

// //**********
// //                            GET IMU DATA
// //**********

// /*
//  *@brief this  updates acceletometer data
//  *@param void
//  *@return void
//  */
// void updateAccelerometerData()
// {
//     accelerometerData.x = mpu.getAccX() - accelerometerOffset.x;
//     accelerometerData.y = mpu.getAccY() - accelerometerOffset.y;
//     accelerometerData.z = mpu.getAccZ() - accelerometerOffset.z;
// }

// /*
//  *@brief this function return the complete gyroscope data
//  *@param void
//  *@return GyrData
//  */
// void updateGyroscopeData()
// {
//     gyroscopeData.pitch = mpu.getPitch() - gyroscopeOffset.pitch;
//     gyroscopeData.roll = mpu.getRoll() - gyroscopeOffset.roll;
//     gyroscopeData.yaw = mpu.getYaw() - gyroscopeOffset.yaw;
// }

// /*
//  *@brief this function return the complete magnetometer data
//  *@param void
//  *@return MagData
//  */
// void updateMagnetometerData()
// {
//     magnetometerData.x = mpu.getMagX() - magnetometerOffset.x;
//     magnetometerData.y = mpu.getMagY() - magnetometerOffset.y;
//     magnetometerData.z = mpu.getMagZ() - magnetometerOffset.z;

//     return magnetometerData;
// }

// void printData()
// {

//     updateAccelerometerData();
//     updateGyroscopeData();
//     updateMagnetometerData();

//     Serial.print(accelerometerData.x, 2);
//     Serial.print(", ");
//     Serial.print(accelerometerData.y, 2);
//     Serial.print(", ");
//     Serial.print(accelerometerData.z, 2);
//     Serial.print(", ");

//     Serial.print(gyroscopeData.yaw, 2);
//     Serial.print(", ");
//     Serial.print(gyroscopeData.pitch, 2);
//     Serial.print(", ");
//     Serial.print(gyroscopeData.roll, 2);
//     Serial.print(", ");

//     Serial.print(magnetometerData.x, 2);
//     Serial.print(", ");
//     Serial.print(magnetometerData.y, 2);
//     Serial.print(", ");
//     Serial.print(magnetometerData.z, 2);
//     Serial.print(", ");
// }

// Declaraciones NRF24 transmisor
#define PERIODO_MUESTREO_NRF24 1000
#define PIN_CE 8   // Pin para habilitar el modulo NRF24
#define PIN_CSN 53 // Pin para seleccionar el dispositivo NRF24 durante la comunicación SPI.
float gpsLatitud = 0;
float gpsLongitud = 0;

// SoftwareSerial ss(18, pTX);

const unsigned char direccionReceptor[5] = {'T', 'L', 'M', 'T', 'R'};
const unsigned char direccionTransmisor[5] = {'T', 'L', 'M', 'T', 'T'};
RF24 transmisionDatos(PIN_CE, PIN_CSN);                       // Se activa la comunicación con el MCU y se selecciona la interfaz SPI
float paquete[6] = {23.444444, 45.6, 12.2, 73.4, 85.6, 92.2}; // Valores float a enviar
float paquete2[3] = {10.87876, 26.4, 80.7};
unsigned long tiempoAnterior = 0;
unsigned long tiempoAntGPS = 0;
unsigned long tiempoAntNRF24 = 0;
unsigned long tiempoAntIMU = 0;

TinyGPSPlus gps;

void setup()
{
    Serial.begin(115200);
    //   Serial1.begin(9600)
    UART_begin();
    // Wire.begin();

    //Setup para la IMU
//     if (!mpu.setup(0x68))
//     {
//         while (1)
//         {
//             Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//             // delay(5000);
//             while (1)
//             {
//                 if ((millis() - tiempoAntIMU) >= 5000)
//                 {
//                     tiempoAntIMU = millis();
//                     break;
//                 }
//             }
//         }
//     }

//     // calibrate anytime you want to
//     Serial.println("Accel Gyro calibration will start in 5sec.");
//     Serial.println("Please leave the device still on the flat plane.");
//     mpu.verbose(true);
//     //   delay(5000);
//     while (1)
//     {
//         if ((millis() - tiempoAntIMU) >= 5000)
//         {
//             tiempoAntIMU = millis();
//             break;
//         }
//     }
//     mpu.calibrateAccelGyro();

//     Serial.println("Mag calibration will start in 5sec.");
//     Serial.println("Please Wave device in a figure eight until done.");
//     // delay(5000)
//     while (1)
//     {
//         if ((millis() - tiempoAntIMU) >= 5000)
//         {
//             tiempoAntIMU = millis();
//             break;
//         }
//     }
//     mpu.calibrateMag();
//     // print_calibration();
//     mpu.verbose(false);

//     /*
//   mpu.setMagneticDeclination(-0.36);  // Latitude: 9° 32' 55.5" N, Longitude: 100° 3' 9" E, Thailand, Magnetic Declination: -0° 36'
//   mpu.setFilterIterations(20);        // The default value is 1. Generally 10-20 is good for stable yaw estimation. Please see [this discussion](https://github.com/kriswiner/MPU9250/issues/420) for the detail.
//   mpu.setAccBias(-64.05, 19.68, 43.55);
//   mpu.setGyroBias(-0.24, 0.16, 0.51);
//   mpu.setMagBias(-43.15, 123.07, -19.13);
//   mpu.setMagScale(0.87, 0.95, 1.25);

// */

//     // calibrate anytime you want to
//     calibrateAcelerometer();

    // Setup para el GPS
    //  Se da un tiempo de espera de 2 segundos para que el GPS se inicie
    while (1)
    {
        if ((millis() - tiempoAntGPS) >= 2000)
        {
            tiempoAntGPS = millis();
            break;
        }
    }

    // Setup para el NRF24
    if (transmisionDatos.begin() != NULL)
    {
        Serial.println("TRANSMISOR INICIALIZADO\n");
        Serial.println("Orden de valores para localización y telemtría ");
        Serial.println("|Lat      |Lng       |aX        |aY        |aZ        |gX        |gY        |gZ        |mX        |mY        |mZ        |Otro     |");
        transmisionDatos.setDataRate(RF24_250KBPS);
        transmisionDatos.setRetries(3, 5);
        transmisionDatos.openWritingPipe(direccionReceptor);
    }
    else
        Serial.println("Error en la inicialización del módulo\n");

    // transmisionDatos.begin();
    transmisionDatos.setDataRate(RF24_250KBPS);
    transmisionDatos.openReadingPipe(1, direccionTransmisor);
    // transmisionDatos.stopListening();
    transmisionDatos.startListening();
}

void loop()
{
    // Obtención de datos para la IMU
    // if (mpu.update())
    // {
    //     static uint32_t prev_ms = millis();
    //     if (millis() > prev_ms + 25)
    //     {
    //         printData();
    //         prev_ms = millis();
    //     }
    // }

    // Configuración para el modulo GPS
    // while (Serial1.available() > 0)
    while (UART_available())
    {
        char frame = UART_receive();
        // while (UART_read()> 0) {
        // while (uartDriverAvailable()){

        if (gps.encode(frame))
            // if (gps.encode(UART_read()))

            // if (gps.encode(uartDriverRead()))
            localizacion();
    }
    // if (millis() > 5000 && gps.charsProcessed() < 10) {
    //   Serial.println(F("No GPS detected: check wiring."));
    //   while (true)
    //     ;
    // }

    // Configuración para el modulo NRF24

    if (millis() - tiempoAnterior >= PERIODO_MUESTREO_NRF24)
    {
        transmitir();
        tiempoAnterior = millis();
    }

    if (transmisionDatos.available())
    {

        recibir();
    }

    // else {
    //   Serial.println( " no recibio ");
    // }

    // Configuración para el módulo
    // Configuración de la IMU
    // Configuración del módulo
}

// FUNCIONES PARA GPS
void localizacion()
{
    // Serial.print(("Location: "));
    if (gps.location.isUpdated())
    {

        if (gps.location.isValid())
        {

            // Guardado de datos de localización en el paquete a transmitir con 6 decimales cada flotante
            paquete[0] = gps.location.lat();
            paquete[1] = gps.location.lng();
            Serial.print(paquete[0], 6);
            Serial.print(",");
            // Serial.println(paquete[1], 6);
        }
        else
        {
        Serial.println("Esperando GPS");
        }
    }
}

// FUNCIONES PARA NRF24
void transmitir()
{

    bool frame1;
    bool frame2;
    // delay(500);
    while (1)
    {
        if ((millis() - tiempoAntNRF24) >= 500)
        {
        tiempoAntNRF24 = millis();
        break;
        }
    }
    transmisionDatos.stopListening();

    frame1 = transmisionDatos.write(paquete, sizeof(paquete));
    if (frame1)
        Serial.println(); // Serial.println("  Trama 1. Transmitida\n");
    else
        Serial.println("  Transmisión de la trama 1 falló\n");

    // delay(1000);

    while (1)
    {
        if ((millis() - tiempoAntNRF24) >= 1000)
        {
        tiempoAntNRF24 = millis();
        break;
        }
    }
    frame2 = transmisionDatos.write(paquete2, sizeof(paquete2));
    if (frame1 && frame2)
    {
        // Serial.println("  Trama 2. Transmitida\n");
        // Serial.println("Mensaje enviado: ");

        for (int i = 0; i < (sizeof(paquete) / 4 + sizeof(paquete2) / 4); i++)
        {
        if (i < sizeof(paquete) / 4)
        {
            Serial.print(paquete[i], 6);
            Serial.print(",");
        }
        else
        {
            Serial.print(paquete2[i - 6], 6);
            Serial.print(",");
        }
        }
        // Serial.println();
    }
    else
    {
        Serial.println("  Transmisión del paquete 2 falló\n");
    }
    // delay(100);

    while (1)
    {
        if ((millis() - tiempoAntNRF24) >= 100)
        {
        tiempoAntNRF24 = millis();
        break;
        }
    }
    transmisionDatos.startListening();
}

void recibir()
{
    uint8_t pwm[4];
    transmisionDatos.read(pwm, sizeof(pwm));
    Serial.print(pwm[0]);
    Serial.print(pwm[1]);
    Serial.print(pwm[2]);
    Serial.println(pwm[3]);
}
