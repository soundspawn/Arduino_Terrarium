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

Timer t;

byte temperature = 0;
byte humidity = 0;

#define SERIAL

enum {
    // The data I/O pin connected to the DHT11 sensor
    DHT_DATA_PIN = 13,
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
byte lightPins[] PROGMEM = {REDLIGHT_PIN,GREENLIGHT_PIN,BLUELIGHT_PIN};

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
  #ifdef SERIAL
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

  //Set Temperature/Humidity Update Timer
  t.every(60000, updateTempHum);
  updateTempHum();

  Ethernet.begin(mac, ip);
  server.begin();
}

void invalidateTempReadings(){
  #ifdef SERIAL
    Serial.println(F("Invalidating Temperature Readings"));
  #endif
  temperature = 0;
  humidity = 0;
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
      #ifdef SERIAL
        Serial.print(F("Humidity (%): "));
        Serial.println(humidity);
        Serial.print(F("Temperature (F): "));
        Serial.println(temperature);
      #endif
      break;
    case Dht11::ERROR_CHECKSUM:
      #ifdef SERIAL
        Serial.println(F("Checksum error"));
        t.after(200,updateTempHum);
      #endif
      break;
    case Dht11::ERROR_TIMEOUT:
      #ifdef SERIAL
        Serial.println(F("Timeout error"));
        t.after(200,updateTempHum);
      #endif
      break;
    default:
      #ifdef SERIAL
        Serial.println(F("Unknown error"));
        t.after(200,updateTempHum);
      #endif
      break;
  }

  //Decide what to do with the heater
  heaterLogic();
}

void heaterLogic(){
  if(temperature == 0){
    //error, make no decisions
    return;
  }
  if(temperature < 74){
    digitalWrite(HEATER_RELAY_PIN, HIGH);
    #ifdef SERIAL
      Serial.println(F("Turning Heater On"));
    #endif
  }else{
    digitalWrite(HEATER_RELAY_PIN, LOW);
    #ifdef SERIAL
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

void httpClient(){
  if (serverajax.available()) {
    char c = serverajax.read();
    ServerTimeout = 0;
    #ifdef SERIAL
      Serial.print(c);
    #endif
  }
  if(!serverajax.connected() && ServerConnected){
    #ifdef SERIAL
      Serial.println(F("disconnecting."));
    #endif
    serverajax.stop();
  }
  ServerConnected = serverajax.connected();
  if(ServerConnected){
    ServerTimeout++;
    delay(1);
  }

  //Server Response Timeout
  if(ServerTimeout > 3000 && ServerConnected){
    serverajax.stop();
    ServerConnected = false;
    #ifdef SERIAL
      Serial.println(F("timeout"));
    #endif
  }
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
  httpClient();
   
  colorSelect = lightPins[(int)(analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS)];
  dimmer = analogRead(DIMMER_INPUT);
  analogWrite(colorSelect, dimmer / 4);

  delay(200);
  lcd.setCursor(0,0);
  lcd.print(F("Color "));
  lcd.print((int)analogRead(COLOR_SELECT_INPUT) / COLOR_DIVISIONS);
  lcd.print(F(" = "));
  lcd.print(dimmer / 4);
}
