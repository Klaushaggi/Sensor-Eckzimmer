// DHT22-Werte per MQTT zum iobroker
// und deep_sleep

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SimpleDHT.h>
#include <Arduino.h>

// for DHT22
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT22 = 5;   // -> GPIO5
SimpleDHT22 dht22;

float temperature = 0;
float humidity = 0;
float rssi = 0;
int err = SimpleDHTErrSuccess;
double spg  = 0;

const char* ssid = "****";
const char* password = "****";
const char* mqtt_server = "*****";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msgtemp[50];
char msgluft[50];
char msgrssi[50];
char msgspg[50];
int value = 0;
//int status = WL_IDLE_STATUS;

void setup_wifi() 
{
  int versuch=1;
  delay(10);
  // We start by connecting to a WiFi network
 /* Serial.println();
  Serial.print("Try to connecting to ");
  Serial.println(ssid);
*/
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    if(WiFi.status() != WL_CONNECTED)
    { 
      //delay(250); 
      //digitalWrite(D4, LOW); 
      //delay(250);
      //digitalWrite(D4, HIGH);
        delay(500);
      //Serial.print(".");
      if(++versuch==30) 
      {
        //Serial.println("Keine WiFi-Verbindung möglich");
        return;
      }
    }
  }

}

void reconnect()
{
  int versuch=1;

  // Loop until we're reconnected
  while (!client.connected()) {
    /*Serial.print("Versuch Nr. ");Serial.printf("%i: ",versuch);
    Serial.println("Attempting MQTT connection...");  */
    // Attempt to connect
    if (client.connect("EckzimmerESP")) {
     // Serial.println("connected");
    } else {
     /* Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");  */
      // Wait 2 seconds before retrying
      delay(2000);

      if(++versuch==4)
         return;
    }
  }
}

void setup()
{

  pinMode(15,OUTPUT);  // GPIO15, zur Spg.-Vers. DHT22
  digitalWrite(15,0);  // einmal kurz aus und wieder an
  delay(500);
  digitalWrite(15,1);

                               // Division mit float beachten: 242.0 statt 242
  spg = analogRead (A0)/242.0; // Analog Values 0 to 1024
  //Serial.println (spg,2);      // 2 Nachkommenstellen 
  
  //pinMode(D4, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
 // digitalWrite(D4, HIGH);  // LED ist aus
  //Serial.begin(115200);
  setup_wifi();
  
  if(WiFi.status() == WL_CONNECTED) // nur falls Wifi verbunden
  { /*
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());  */
      //Received Signal Strength Indicator
    rssi=WiFi.RSSI();
    /*Serial.print("RSSI= ");Serial.print(rssi);
       Serial.println("dBm");  */
    client.setServer(mqtt_server, 1883);

    //digitalWrite(D4, LOW);
    if (!client.connected()) 
    {
      reconnect();
    }

   if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) 
  {
    // Serial.print("Read DHT22 failed, err="); 
    // Serial.println(err);
    delay(2000);
    // nochmal versuchen
    dht22.read2(pinDHT22, &temperature, &humidity, NULL);
  }
   /*
   Serial.print((float)temperature,1); Serial.print(" *C, ");  // serial.print(wert,Nachkommastellen)
   Serial.print((float)humidity,1); Serial.println(" RH%");
   */
    if(client.connected())
    {
    dtostrf(temperature, 0, 1, msgtemp);
    dtostrf(humidity, 0, 1, msgluft);
    dtostrf(rssi, 0, 1, msgrssi);
    dtostrf(spg, 0, 2, msgspg);
    /*
    Serial.println("Publish message: ");
    Serial.print("Temp.: "); Serial.println(msgtemp);    
    Serial.print("Luft: ");Serial.println(msgluft);
    Serial.print("RSSI: ");Serial.println(msgrssi);
    Serial.print("Spg: ");Serial.println(msgspg);  */
    client.publish("Sensoren/Eckzimmer/RSSI", msgrssi); 
    delay(50); 
    client.publish("Sensoren/Eckzimmer/Luft", msgluft);
    delay(50);
    client.publish("Sensoren/Eckzimmer/Spg", msgspg);
    delay(50);
    client.publish("Sensoren/Eckzimmer/Temp", msgtemp);
    delay(50);
    } /*
    else
    {
      Serial.println("Nichts gesendet, keine Verbindung zum Broker"); 
    } */
    
  }
    digitalWrite(15,0);  // Sensor aus
    ESP.deepSleep(3600e6); //3.600.000.000us oder 60min=3.600s
    delay(100);
}


void loop() {  
}
