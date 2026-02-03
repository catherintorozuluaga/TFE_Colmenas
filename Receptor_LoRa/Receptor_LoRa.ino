/*  Gateway
 *  RECEPTOR DE PAQUETES LORA CON PANTALLA OLED 
 *  TFE UNIR
 *  Catherin Toro - Jose Cortes
 * 
 */

//////////////////////////////////////////
//////////  Librerias ///////////////////
/////////////////////////////////////////
#include <LoRa.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <InfluxDbClient.h> // Para conectar la base de datos InfluxDB CLOUD
#include <InfluxDbCloud.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
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

//Aquí­ definimos una frecuencia de operacion segun nuestra ubicacion. 433E6 para Asia, 866E6 para Europa, 915E6 para America
#define BAND 915E6 //Colombia
#define MQ135Alerta 25 // Led Alerta
// Data point
Point sensor("DatLoRa2");// Sin aguiojo
Point sensor1("DatLoRa3"); //Con Aguijon
String DatoLoRa;//Cadena de texto para recibir datos del otro LoRa.
////////////////////////////////////////////////////////////////////
//////////  WIFI  ////////////////////////////// ///////////////////
////////////////////////////////////////////////////////////////////
// WiFi AP SSID
  #define WIFI_SSID "Familia 2Toro"
  // WiFi password
  #define WIFI_PASSWORD "17156716"
////////////////////////////////////////////////////////////////////
//////////  INFLUXDB CLOUD ////////////////////////////// //////////
////////////////////////////////////////////////////////////////////
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "mEipQ6gpJECqbt93CVqmqqXcSLB45VJP9GFoN_Oh2BhaPh1TK4w2HuD1_XEj9Vkmrva86Kt--HspLYtr9FBmUw=="
#define INFLUXDB_ORG "8dc798ee68627ead"
#define INFLUXDB_BUCKET "BIoT"
// Time zone info
  #define TZ_INFO "<-05>5" //BOgota
// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

////////////////////////////////////////////////////////////////////
//////////  setup() ///// ////////////////////////////// //////////
////////////////////////////////////////////////////////////////////
void setup() { 
  //initialize Serial Monitor
  Serial.begin(115200);
  pinMode(MQ135Alerta, OUTPUT);
  digitalWrite(MQ135Alerta, LOW);
  Serial.println("Prueba de recepcion LoRa");
  
  SPI.begin(SCK, MISO, MOSI, SS);  //Definimos pines SPI
  LoRa.setPins(SS, RST, DIO0); //Configuramos el LoRa para enviar
  
  if (!LoRa.begin(BAND)) {//Intenta transmitir en la banda elegida
    Serial.println("Error iniciando LoRa");//Si no puede transmitir, marca error
    while (1);
  }
  Serial.println("Inicio exitoso de LoRa!");//Mensaje de todo bien en puerto serial
  delay(200);//Esperamos un par de segundos
   // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  // Add tags
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());
 // Syncing progress and the time will be printed to Serial.
    timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
 // Check server connection
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());
    } else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
    }
 
}
////////////////////////////////////////////////////////////////////
//////////  loop() ///// ////////////////////////////// //////////
////////////////////////////////////////////////////////////////////
void loop() {

  // RECIBIR PAQUETE LoRa
  int tamanoPaquete = LoRa.parsePacket();  //analizamos paquete
  if (tamanoPaquete) {//Si nos llega paquete de datos
    
    if (tamanoPaquete) {
    // Leer el paquete completo
    String datosRecibidos = "";
    while (LoRa.available()) {
      datosRecibidos += (char)LoRa.read();
    }
    Serial.println("Paquete recibido "+datosRecibidos);//Muestra confirmacion
    int index = 0;
    //Separador de comas del dato recibido
    String valores[10]; // hasta 10 valores
    int start = 0;
    for (int i = 0; i < datosRecibidos.length(); i++) {
      if (datosRecibidos.charAt(i) == ',') {
        valores[index++] = datosRecibidos.substring(start, i);
        start = i + 1;
      }
    }
    valores[index] = datosRecibidos.substring(start); // último valor

    // Asignar variables
    String nodeid = valores[0];
    float temperatura = valores[1].toFloat();
    float humedad     = valores[2].toFloat();
    float ax          = valores[3].toFloat();
    float ay          = valores[4].toFloat();
    float az          = valores[5].toFloat();
    float micro       = valores[6].toFloat();
    float CO2         = valores[7].toFloat();
    float peso        = valores[8].toFloat();

    // Mostrar en Serial
    Serial.println("Datos Nodo: " + nodeid);
    Serial.println("Temperatura: " + String(temperatura));
    Serial.println("Humedad: " + String(humedad));
    Serial.println("ax: " + String(ax) + " ay: " + String(ay) + " az: " + String(az));
    Serial.println("Micro: " + String(micro));
    Serial.println("CO2: " + String(CO2));
    Serial.println("Peso: " + String(peso));

    if (CO2>=450){
      digitalWrite(MQ135Alerta, HIGH);
      delay(350);
      } else{digitalWrite(MQ135Alerta, LOW);delay(200);}
////////////////////////////////////////////////////////////////////
//////////  Enviar Datos al InfluxDB Cloud ///// ///////////////////
////////////////////////////////////////////////////////////////////
    if (nodeid=="1"){// sin aguijon Datalora 2
       // Store measured value into point Envio Datos Influx
      sensor.clearFields();
      sensor.addField("TempLoRa1",temperatura);
      sensor.addField("HumLoRa1",humedad);
      sensor.addField("axLoRa1",ax);
      sensor.addField("ayLoRa1",ay);
      sensor.addField("azLoRa1",az);
      sensor.addField("MicroLoRa1",micro);
      sensor.addField("CO2LoRa1",CO2);
      sensor.addField("PesoLoRa1",peso);
        if (!client.writePoint(sensor))     //Write data point
      {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
    }
    if (nodeid=="2"){//con aguijon Datalora3
       // Store measured value into point Envio Datos Influx
      sensor1.clearFields();
      sensor1.addField("TempLoRa2",temperatura);
      sensor1.addField("HumLoRa2",humedad);
      sensor1.addField("axLoRa2",ax);
      sensor1.addField("ayLoRa2",ay);
      sensor1.addField("azLoRa2",az);
      sensor1.addField("MicroLoRa2",micro);
      sensor1.addField("CO2LoRa2",CO2);
       if (!client.writePoint(sensor1))     //Write data point
      {
        Serial.print("InfluxDB write failed Sensor 1: ");
        Serial.println(client.getLastErrorMessage());
      }
    }
   if (wifiMulti.run() != WL_CONNECTED)                               //Check WiFi connection and reconnect if needed
    Serial.println("Wifi connection lost");


  
  delay(10000);
  }
}
}
