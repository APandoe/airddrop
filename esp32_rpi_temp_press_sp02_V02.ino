/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>

// SSID/Password combination
const char* ssid = "GL-AR300M-4a2";
const char* password = "goodlife";

// MQTT Broker IP address
const char* mqtt_server = "192.168.8.189";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


#define DHTTYPE DHT22     // DHT 22 (AM2302)
#define DHTPIN 27         // Digital pin connected to the DHT sensor
#define THERMOCOUPLE 14   // Digital pin connected to DS18S20
#define PRESSUREPIN 36    // Analog pin connected to ADP5120
#define RESPIN 12         // Reset pin
#define MFIOPIN 26        // MFIO pin

DHT dht(DHTPIN, DHTTYPE);

// Temperature chip i/o
OneWire ds(THERMOCOUPLE);  // on digital pin 14

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(RESPIN, MFIOPIN); 

bioData body;  

float temperature = 0;
float humidity = 0;
float pressure = 0;

// LED Pin
const int ledPin = 25;

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);

  dht.begin();

  // For SP02 readings
  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!!!");
 
  Serial.println("Configuring Sensor....");
  int error = bioHub.configBpm(MODE_TWO); // Configuring just the BPM settings. 
  if(error == 0){ // Zero errors
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Data lags a bit behind the sensor, if your finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data....");
  
  delay(100); // just here to slow down the output so it is easier to read
}

// For DS18S20
float getTemp(){
  
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return 2;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return 3;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return 4;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); 
  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
  
}

// For wifi 
void setup_wifi() {
  delay(10);
  
  // Connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // More if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  
  if (String(topic) == "esp32/output"){
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  body = bioHub.readBpm();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Information from the readBpm function will be saved to our "body"
    // variable. Extracted later.
    char heartRateString[8];
    dtostrf(body.heartRate, 1, 2, heartRateString);
    Serial.print("Heartrate: ");
    Serial.println(heartRateString);
    if (heartRateString != 0) {
      client.publish("sen15219/heartrate", heartRateString);
    }
    
    Serial.print("Confidence: ");
    Serial.println(body.confidence); 
    Serial.print("Oxygen: ");
    Serial.println(body.oxygen); 
    Serial.print("Status: ");
    Serial.println(body.status); 
    Serial.print("Extended Status: ");
    Serial.println(body.extStatus); 
    Serial.print("Blood Oxygen R value: ");
    Serial.println(body.rValue); 
    // Slow it down or your heart rate will go up trying to keep up
    // with the flow of numbers
//     delay(250);
    
    // Temperature in Celsius
    temperature = dht.readTemperature();
    // For Fahrenheit 
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
    // for the DS18S20
    char tempStringDS18[8];
    float temperature_DS18 = getTemp();
    dtostrf(temperature_DS18, 1, 2, tempStringDS18);
    Serial.print("Temperature from DS18B210: ");
    Serial.println(tempStringDS18);
    client.publish("ds18b20/temperature", tempStringDS18);
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

    humidity = dht.readHumidity();
    
    // Convert the value to a char array
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);


    // finding pressure value
    pressure = analogRead(PRESSUREPIN);
    char pressString[8];
    dtostrf(pressure, 1, 2, pressString);
    Serial.print("Pressure: ");
    Serial.println(pressString);
    client.publish("adp5120/pressure", pressString);
  }
}
