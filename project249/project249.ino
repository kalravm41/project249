#include<WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include<DHT.h>

//  rgb led details
byte rpin = 25;
byte gpin = 26;
byte bpin = 27;
byte rchannel = 0;
byte gchannel = 1;
byte bchannel = 2;
byte resolution = 8;
int frequency = 5000;

byte Red;
byte Green;
byte Blue;

//  dht details
byte dht_pin = 4;
#define dht_type DHT11
DHT dht(dht_pin , dht_type);

//  wifi credentials

//  io details
#define IO_USERNAME  "Bhatt_Archana"
#define IO_KEY       "aio_yfRL37aIlyiftj95TbImqzAN3uFi"
#define IO_BROKER    "io.adafruit.com"
#define IO_PORT       8883

#define ssid       "archanabhatt"
#define password       "10Archana#68"

//  client details
WiFiClient wificlient;
Adafruit_MQTT_Client mqtt(&wificlient , IO_BROKER , IO_PORT , IO_USERNAME , IO_KEY);

Adafruit_MQTT_Publish temp_c = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Temperature(c)");
Adafruit_MQTT_Publish temp_f = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Temperature(F)");
Adafruit_MQTT_Publish temp_k = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Temperature(K)");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish red = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Red");
Adafruit_MQTT_Publish green = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Green");
Adafruit_MQTT_Publish blue = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/Blue");

void mqttconnect();

void setup()
{
  Serial.begin(115200);

  //  connecting with wifi
  Serial.print("Connecting with : ");
  Serial.println(ssid);
  WiFi.begin(ssid , password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected !");
  Serial.print("IP assigned by AP : ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //  RGB led setup
  ledcSetup(rchannel , frequency , resolution);
  ledcSetup(gchannel , frequency , resolution);
  ledcSetup(bchannel , frequency , resolution);

  //  attaching pins with channel
  ledcAttachPin(rpin , rchannel);
  ledcAttachPin(gpin , gchannel);
  ledcAttachPin(bpin , bchannel);

  //  dht setup
  dht.begin();

  //  feeds to be subscribed
   mqtt.subscribe(&red);
   mqtt.subscribe(&green);
   mqtt.subscribe(&blue);


  
}

void loop()
{
  //  connecting with server
  mqttconnect();

  //  reading values from dht sensor
  float tempc = dht.readTemperature();
  float tempf = dht.readTemperature(true);
  float tempk = tempc + 273.15;
  float humidity = dht.readHumidity();
  float dew_point = (tempc - (100 - humidity) / 5);  //  dew point in celcius

  if (isnan(tempc)  ||  isnan(tempf)  ||  isnan(humidity))
  {
    Serial.println("Sensor not working!");
    delay(1000);
    return;
  }

  //  printing these values on serial monitor
  String val = String(tempc) + " *C" + "\t" + String(tempf) + " *F" + "\t" + String(tempk) + " *K" + "\t" + 
               String(humidity) + " %RH" + "\t" + String(dew_point) + " *C";
  Serial.println(val);
  

  //  publish the DHT data to the feeds
  Adafruit_MQTT_Subscribe subscription;
  //  subscribe the slider values to control RGB

    while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &red) {
      //Serial.print(F("Got: "));
      //Serial.println((char *)sw1.lastread);
      //digitalWrite(led1, HIGH);

      String stringOne = (char *)red.lastread;
      Serial.print(F("stringOne: "));
      Serial.println(stringOne);

      Red = stringOne;
    }

    if (subscription == &Green) {
      //Serial.print(F("Got: "));
      //Serial.println((char *)sw2.lastread);
      //digitalWrite(led2, HIGH);

      stringTwo = (char *)Green.lastread;
      Serial.print(F("stringTwo: "));
      Serial.println(stringTwo);

       Green = stringTwo;

    }

    if (subscription == &Blue) {
      //Serial.print(F("Got: "));
      //Serial.println((char *)sw2.lastread);
      //digitalWrite(led2, HIGH);

      stringThree = (char *)Blue.lastread;
      Serial.print(F("stringThree: "));
      Serial.println(stringThree);

       Blue = stringThree;

    }

    makecolor(Red,Green,Blue);
  }
  
}

void mqttconnect()
{
  //  if already connected, return
  if (mqtt.connected())return;

  //  if not, connect
  else
  {
    while (true)
    {
      int connection = mqtt.connect();  //  mqq client has all details of client, port , username, key
      if (connection  ==  0)
      {
        Serial.println("Connected to IO");
        break;  //  connected
      }
      else
      {
        Serial.println("Can't Connect");
        mqtt.disconnect();
        Serial.println(mqtt.connectErrorString(connection));  //  printing error message
        delay(5000);  //  wait for 5 seconds
      }
    }
  }


  //  wait for some time
  delay(5000);
}

void makecolor(byte r , byte g , byte b)
{
  //  printing values
  Serial.print("RED : ");
  Serial.print(r);
  Serial.print('\t');
  Serial.print("GREEN : ");
  Serial.print(g);
  Serial.print('\t');
  Serial.print("BLUE : ");
  Serial.println(b);

  //  writing values
  ledcWrite(rchannel , r);
  ledcWrite(gchannel , g);
  ledcWrite(bchannel , b);
}
