/*  Tansmisor Abejas sin Aguijon Nodo 1
 *  TRANSIMISOR DE PAQUETES LORA CON PANTALLA OLED 
 *  TFE UNIR
 *  Catherin Toro - Jose Cortes
 * 
 */
//////////////////////////////////////////
//////////  Librerias ///////////////////
/////////////////////////////////////////
#include <LoRa.h>
#include <SPI.h>
#include <Math.h>
//Libraries para comunicar con y dibujar en la pantalla OLED integrada
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // Sensor de Temperatura y Humedad
#include <DFRobot_BMI160.h> // Sensor Acelerometro
#include <MQ135.h> // Sensor CO2
#include "HX711.h" // Peso
#include "arduinoFFT.h" // Microfono
#include <esp_sleep.h>  // Dormir el ESP

////////////////////////////////////////////
//////////  Modulo LoRa  ///////////////////
///////////////////////////////////////////
//Debemos definir los pines que se utilizaran por el modulo LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26
const int nodeid=1; // Tansmisor 1 - Abejas sin aguijon
String dataLoRa;
//Aqui­ definimos una frecuencia de operacion segun nuestra ubicación. 433E6 para Asia, 866E6 para Europa, 915E6 para America
#define BAND 915E6 // Colombia


////////////////////////////////////////////////////////////////////
//////////  Sensor BME280 - Temperatura y Humedad ///////////////////
////////////////////////////////////////////////////////////////////

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
///Temperatura
//int sensorHumedad = 34;int lectura = 0;
float Temperatura=0; float Humedad=0;
////////////////////////////////////////////////////////////////////
//////////  Sensor BMI160 - Acelerometro //////////////////////////
////////////////////////////////////////////////////////////////////
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
// Variables for orientation calculations
float ax, ay, az;
float gx, gy, gz;
float pitch, roll, yaw;
unsigned long lastTime;
float dt = 0.01; // Time interval in seconds (10 ms)
////////////////////////////////////////////////////////////////////
//////////  MAX4466 - Microfono  ///////////////////////////////////
////////////////////////////////////////////////////////////////////
// Microfono MAX4466
#define CHANNEL 36
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 4000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

////////////////////////////////////////////////////////////////////
//////////  Sensor MQ135 -  Calidad del Aire CO2 ///////////////////
////////////////////////////////////////////////////////////////////
/* Los valores de co2 recomendados en interiores son:
    
    Hasta 350 ppm:                Alta calidad de aire 
    Entre 500 y 800 ppm:          Moderada calidad de aire 
    Entre 800 y 1200 ppm:         Baja calidad de aire 
    Nivel superior a 1200 ppm:    Mala calidad de aire 
    */
 #define MQ135_POWER 15
 #define PIN_MQ 39
  MQ135 mq135_sensor(PIN_MQ);

////////////////////////////////////////////////////////////////////
//////////  Sensor HX711 - Peso  /////// //////////////////////////
////////////////////////////////////////////////////////////////////

#define DOUT 25
#define CLK 4
HX711 balanza;

// ⭐ Variables de peso persistente
//double pesoAcumulado = 0;      // Peso que se guarda entre resets
double pesoInstantaneo = 0;    // Peso leído en esta ejecución
//double pesoTotal = 0;
//Adafruit_SSD1306 display(ANCHOPANTALLA, ALTOPANTALLA, &Wire, OLED_RST);
// ----- ACUMULADO DE PESO EN RTC -----
RTC_DATA_ATTR double pesoAcumulado = 0;

////////////////////////////////////////////////////////////////////
//////////  setup  ()  ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

void setup() {
  
 
   Serial.begin(115200);//inicia monitor serial  
  // Power MQ135 OFF
  pinMode(MQ135_POWER, OUTPUT);
  digitalWrite(MQ135_POWER, HIGH);
  ////////////////////////////////////////////////////////////////////
////////// LoRa     /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
        SPI.begin(SCK, MISO, MOSI, SS);  //Definimos pines SPI
        LoRa.setPins(SS, RST, DIO0); //Configuramos el LoRa para enviar
  
        if (!LoRa.begin(BAND)) {//Intenta transmitir en la banda elegida
          Serial.println("Error iniciando LoRa");//Si no puede transmitir, marca error
          while (1);
        }
        Serial.println("Inicio exitoso de LoRa!");//Mensaje de todo bien en puerto serial
        delay(2000);//Esperamos un par de segundos
////////////////////////////////////////////////////////////////////
//////////   Sensor BME280 Temp y Humedad///////////////////////////
////////////////////////////////////////////////////////////////////
        bool status;
        status = bme.begin(0x76);  
        if (!status) {
          Serial.println("No se detecta BME280 sensor, Temp y Hum!");
          while (1);
        }
////////////////////////////////////////////////////////////////////
//////////   Sensor BMI160  Acelerometro y Giroscopio///////////////
////////////////////////////////////////////////////////////////////

      if (bmi160.softReset() != BMI160_OK) {
          Serial.println("reset false");
          while (1);
        }
      
        if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
          Serial.println("init false");
          while (1);
        }
       lastTime = millis();
////////////////////////////////////////////////////////////////////
//////////   Microfono MAX4466  ////////////////////////////////////
////////////////////////////////////////////////////////////////////
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.println("Ready Microfono");

////////////////////////////////////////////////////////////////////
//////////  HX711 - PESO        ////////////////////////////////////
////////////////////////////////////////////////////////////////////

  balanza.begin(DOUT, CLK);
  Serial.print("Lectura del valor del ADC: ");
  Serial.println(balanza.read());
  Serial.println("Destarando...");
  balanza.set_scale(1274.7);
  balanza.tare(20);

  Serial.print("Peso acumulado recuperado (RTC): ");
  Serial.println(pesoAcumulado, 3);

}
////////////////////////////////////////////////////////////////////
//////////  loop()             ////////////////////////////////////
////////////////////////////////////////////////////////////////////
void loop() {
   ////////////////////////////////////////////////////////////
  // ENCENDER MQ135 Y ESPERAR 60s PARA CALENTAMIENTO
  ////////////////////////////////////////////////////////////
  Serial.println("Encendiendo MQ135...");
  digitalWrite(MQ135_POWER, HIGH);
  delay(120000);  // 120 segundos calentamiento
  

////////////////////////////////////////////////////////////
  // LECTURA BME280 Temp y Humedad
  ////////////////////////////////////////////////////////////

  delay(100); // estabilización
  Temperatura=bme.readTemperature();
  Humedad=bme.readHumidity();
  Serial.println("///////// BME280");
  Serial.print("Temperatura= ");Serial.print(Temperatura);
  Serial.print("     Humedad= ");Serial.println(Humedad);
  
////////////////////////////////////////////////////////////
  // LECTURA BMI160
  ////////////////////////////////////////////////////////////
  delay(25); // estabilización
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Update dt
  lastTime = currentTime;

  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    ax = accelGyro[3] / 16384.0;
    ay = accelGyro[4] / 16384.0;
    az = accelGyro[5] / 16384.0;
    // Calculate pitch and roll from accelerometer data
    ////pitch = atan2(ay, az) * 180 / 3.14;
    ////roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / 3.14;
    // Integrate gyroscope data to get yaw
    ////yaw += gz * dt * 180 / 3.14;
    
    }
    Serial.println("///////// Giroscopio");
    Serial.print("ax= ");Serial.print(ax);
    Serial.print("   ay= ");Serial.print(ay);
    Serial.print("   az= ");Serial.println(az);
  /////////////////////////////////////////  
 // Microfono MAX4466
  /////////////////////////////////////////  
       /*SAMPLING*/
        microseconds = micros();
        for(int i=0; i<samples; i++)
        {
            vReal[i] = analogRead(CHANNEL);
            vImag[i] = 0;
            while(micros() - microseconds < sampling_period_us){
              //empty loop
            }
            microseconds += sampling_period_us;
        }
       
        /* Print the results of the sampling according to time */

        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
        FFT.compute(FFTDirection::Forward); /* Compute FFT */
        FFT.complexToMagnitude(); /* Compute magnitudes */
        double vmax=10000; // Resetear el valor máximo antes de empezar
        double f=0;
        for (int i =5; i < samples; i++) {
          if (vReal[i] > vmax) {
            vmax = vReal[i];
             f = ((i * 1.0 * samplingFrequency) / samples);
          }
        }
        Serial.println("Microfono:");
        Serial.print("Voltaje Mayor= ");
        Serial.print(vmax);
        Serial.print("     Frecuencia= ");
        Serial.println(f);
////////////////////////////////////////////////////////////
  // MQ135 – CO2
  ////////////////////////////////////////////////////////////
     //delay(60000);  // 60 segundos calentamiento
     float ppm = mq135_sensor.getPPM();
     float correctedPPM = mq135_sensor.getCorrectedPPM(Temperatura, Humedad);
     Serial.println("///////// AIRE CO2");
     Serial.print("CO2=  ");
     Serial.println(correctedPPM);
  ////////////////////////////////////////////////////////////
//Sensor de Peso
  ////////////////////////////////////////////////////////////
      pesoInstantaneo=-0.0271*(balanza.get_units(20)+0.32);
     // Nuevo peso real = peso guardado + peso relativo actual
     pesoAcumulado = pesoAcumulado + pesoInstantaneo;
  Serial.println("///////// BALANZA");
  Serial.print("Peso instantáneo: "); Serial.print(pesoInstantaneo, 1); Serial.println(" kg");
  Serial.print("Peso acumulado (RTC): "); Serial.println(pesoAcumulado, 3);
     delay(150);
  ////////////////////////////////////////////////////////////
  // Envío de Datos LORA
  ////////////////////////////////////////////////////////////
  dataLoRa=String(nodeid)+','+String(Temperatura)+','+String(Humedad)+','+String(ax)+','+String(ay)+','+String(az)+','+String(f)+','+String(ppm)+','+String(pesoAcumulado);
  
  LoRa.beginPacket();//Inicia protocolo
  LoRa.print(dataLoRa);//Manda cuenta actual
  LoRa.endPacket();//Fin de paquete enviado
  
 
 ////////////////////////////////////////////////////////////
  // APAGAR MQ135 - y Modo Deep Sleep
  ////////////////////////////////////////////////////////////
  digitalWrite(MQ135_POWER, LOW);  
// -------- APAGAR PERIFÉRICOS --------
LoRa.end();
SPI.end();
Wire.end();

// Si usas pantalla OLED 
// display.ssd1306_command(SSD1306_DISPLAYOFF);

// -------- ENTRAR EN DEEP SLEEP --------
uint64_t sleepTime = 20ULL * 60ULL * 1000000ULL; // 20 minutos en microsegundos
//uint64_t sleepTime = 2ULL * 60ULL * 1000000ULL;  // 2 min
Serial.println("Entrando en Deep Sleep por 20 minutos...");
esp_sleep_enable_timer_wakeup(sleepTime);
esp_deep_sleep_start();

}
