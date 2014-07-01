/*
 * Arduino-Terrarium.cpp
 *
 * Main Program to control in-home Terrarium
 */
#include "Arduino.h"
#include "Timer.h"
#include <Dht11.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Ethernet.h>
#include <avr/wdt.h>
#include <JsonArray.h>
#include <JsonHashTable.h>
#include <JsonObjectBase.h>
#include <JsonParser.h>

Timer t;

byte temperature = 0;
byte humidity = 0;

#define SERIALCOM

enum {
    // The data I/O pin connected to the DHT11 sensor
    DHT_DATA_PIN = 9,
    // The baud rate of the serial interface
    SERIAL_BAUD  = 9600,

    COLOR_SELECT_INPUT = A0,
    DIMMER_INPUT = A1,
    HEATER_RELAY_PIN = 8,
};

Dht11 sensor(DHT_DATA_PIN);

#define LIGHT_COLORS 3
enum {
  REDLIGHT_PIN = 2,
  GREENLIGHT_PIN = 3,
  BLUELIGHT_PIN = 4,
  COLOR_DIVISIONS = (int)(1024 / (LIGHT_COLORS+1))+2,
};
byte lightPins[] = {REDLIGHT_PIN,GREENLIGHT_PIN,BLUELIGHT_PIN};

enum {
  I2C_ADDR = 0x3F,
  BACKLIGHT_PIN = 3,
  En_pin = 2,
  Rw_pin = 1,
  Rs_pin = 0,
  D4_pin = 4,
  D5_pin = 5,
  D6_pin = 6,
  D7_pin = 7,
};
LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin,3, POSITIVE);

byte mac[] = { 0xDE, 0xAD, 0xB0, 0xEF, 0xFE, 0xAF };
IPAddress ip(192,168,1, 16);
EthernetServer server(80);
String HTTP_req;
EthernetClient client;
EthernetClient serverajax;

boolean ServerConnected = false;
int ServerTimeout;

JsonParser<32> parser;
JsonHashTable hashTable;

//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
	return 1.8 * celsius + 32;
}

/*
 * setup
 *
 * One-time initialization of the module.
 */
void setup() {
  #ifdef SERIALCOM
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("Serial Connection Established"));
  #endif
  
  lcd.begin (20,4);
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);

  //Set up color select pins
  pinMode(COLOR_SELECT_INPUT, INPUT);
  pinMode(DIMMER_INPUT, INPUT);
  pinMode(REDLIGHT_PIN, OUTPUT);
  pinMode(GREENLIGHT_PIN, OUTPUT);
  pinMode(BLUELIGHT_PIN, OUTPUT);
  
  //Setup Outputs
  pinMode(HEATER_RELAY_PIN, OUTPUT);

  //Timer-based Function
  //These will set a timer upon their completion
  updateTempHum();
  ServerTime();

  Ethernet.begin(mac, ip);
  server.begin();
}

int memoryTest() {
  int byteCounter = 0; // initialize a counter
  byte *byteArray; // create a pointer to a byte array
  // More on pointers here: http://en.wikipedia.org/wiki/Pointer#C_pointers

  // use the malloc function to repeatedly attempt allocating a certain number of bytes to memory
  // More on malloc here: http://en.wikipedia.org/wiki/Malloc
  while ( (byteArray = (byte*) malloc (byteCounter * sizeof(byte))) != NULL ) {
    byteCounter++; // if allocation was successful, then up the count for the next try
    free(byteArray); // free memory after allocating it
  }

  free(byteArray); // also free memory after the function finishes
  return byteCounter; // send back the highest number of bytes successfully allocated
}

void invalidateTempReadings(){
  #ifdef SERIALCOM
    Serial.println(F("Invalidating Temperature Readings"));
  #endif
  temperature = 0;
  humidity = 0;
  digitalWrite(HEATER_RELAY_PIN, LOW);
}

void updateTempHum(){
  static byte TempUpdateTimer = 255;

  switch (sensor.read()) {
    case Dht11::OK:
      if(sensor.getTemperature() == 0){
        t.after(200,updateTempHum);
        return;
      }
      temperature = Fahrenheit(sensor.getTemperature());
      humidity = sensor.getHumidity();
      lcd.setCursor(0,1);
      lcd.print(F("Temperature: "));
      lcd.print(temperature);
      lcd.print(F(" F"));
      lcd.setCursor(0,2);
      lcd.print(F("Humidity: "));
      lcd.print(humidity);
      lcd.print(F(" %"));
      //Refresh a timer to invalidate the temp/hum readings
      t.stop(TempUpdateTimer);
      TempUpdateTimer = t.after(300000,invalidateTempReadings);
      #ifdef SERIALCOM
        Serial.print(F("Humidity (%): "));
        Serial.println(humidity);
        Serial.print(F("Temperature (F): "));
        Serial.println(temperature);
      #endif
      break;
    case Dht11::ERROR_CHECKSUM:
      #ifdef SERIALCOM
        Serial.println(F("Checksum error"));
        t.after(200,updateTempHum);
        return;
      #endif
      break;
    case Dht11::ERROR_TIMEOUT:
      #ifdef SERIALCOM
        Serial.println(F("Timeout error"));
        t.after(200,updateTempHum);
        return;
      #endif
      break;
    default:
      #ifdef SERIALCOM
        Serial.println(F("Unknown error"));
        t.after(200,updateTempHum);
        return;
      #endif
      break;
  }

  //Decide what to do with the heater
  heaterLogic();
  t.after(60000,updateTempHum);
}

void heaterLogic(){
  if(temperature == 0){
    //error, make no decisions
    return;
  }
  if(temperature < 74){
    digitalWrite(HEATER_RELAY_PIN, HIGH);
    #ifdef SERIALCOM
      Serial.println(F("Turning Heater On"));
    #endif
  }else{
    digitalWrite(HEATER_RELAY_PIN, LOW);
    #ifdef SERIALCOM
      Serial.println(F("Turning Heater Off"));
    #endif
  }
}

void httpServer(){
  //Web Server
  client = server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;

        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: application/json"));
          client.println(F("Connection: close"));  // the connection will be closed after completion of the response
          client.println();

          if(HTTP_req.indexOf("/get_temperature") == 4){
            client.print(F("{\"result\":true,\"temperature\":"));
            client.print(temperature);
            client.print(F(",\"humidity\":"));
            client.print(humidity);
            client.print(F("}"));
          }
          HTTP_req = "";
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}

char* Ajax(char *url){
  String message;
  String submessage;
  char c;
  message = "";
  if(serverajax.connect("soundspawn.com",80)){
    serverajax.println("GET /proxy/proxy.php?url=https%3A//jarvis.soundspawn.com/terrarium/get_time HTTP/1.0");
    serverajax.println("Host: soundspawn.com");
    serverajax.println("Connection: close");
    serverajax.println();
    while(serverajax.connected()){
      if(serverajax.available()){
        c = serverajax.read();
        message += c;
      }
    }
    submessage = message.substring(message.indexOf("{"),message.length());
    submessage += "}";
    char d2[message.length()];
    submessage.toCharArray(d2,submessage.length());
    serverajax.stop();
    if(submessage.length() == 1){
      return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Abandoned Connection before response\"}");
    }
    return d2;
  }
  return (char*)F("{\"result\":false,\"message\":\"Arduino Ajax Connection Error\"}");
}

void ServerTime(){
  char* ajax = Ajax("");
  hashTable = parser.parseHashTable(ajax);
  if(!hashTable.success()){
    t.after(5000,ServerTime);
    return;
  }
  unsigned long time = hashTable.getLong("now");
  lcd.setCursor(0,3);
  lcd.print(time);
  lcd.print(F("    "));
  lcd.print(memoryTest());

  t.after(60000,ServerTime);
}

/*
 * loop
 *
 * Code to be executed repeatedly.
 */
void loop() {
  static double sensorMod = 0;
  static int colorSelect = 0;
  
  static int dimmer;
  
  t.update();

  //Receive http request
  httpServer();
   
  colorSelect = lightPins[(int)(analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS)];
  dimmer = analogRead(DIMMER_INPUT);
  analogWrite(colorSelect, dimmer / 4);
  if((int)analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS < 3){
    delay(200);
    lcd.setCursor(0,0);
    lcd.print(F("Color "));
    lcd.print((int)analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS);
    lcd.print(F(" = "));
    lcd.print(dimmer / 4);
  }
}
