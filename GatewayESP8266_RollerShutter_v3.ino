/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * Contribution by a-lurker and Anticimex,
 * Contribution by Norbert Truchsess <norbert.truchsess@t-online.de>
 * Contribution by Ivo Pullens (ESP8266 support)
 *
 * DESCRIPTION
 * The EthernetGateway sends data received from sensors to the WiFi link.
 * The gateway also accepts input on ethernet interface, which is then sent out to the radio network.
 *
 * VERA CONFIGURATION:
 * Enter "ip-number:port" in the ip-field of the Arduino GW device. This will temporarily override any serial configuration for the Vera plugin.
 * E.g. If you want to use the defualt values in this sketch enter: 192.168.178.66:5003
 *
 * LED purposes:
 * - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs in your sketch, only the LEDs that is defined is used.
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error
 *
 * See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
 * nRF24L01+  ESP8266
 * VCC        VCC
 * CE         GPIO4
 * CSN/CS     GPIO15
 * SCK        GPIO14
 * MISO       GPIO12
 * MOSI       GPIO13
 * GND        GND
 *
 * Not all ESP8266 modules have all pins available on their external interface.
 * This code has been tested on an ESP-12 module.
 * The ESP8266 requires a certain pin configuration to download code, and another one to run code:
 * - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
 * - Connect GPIO15 via 10K pulldown resistor to GND
 * - Connect CH_PD via 10K resistor to VCC
 * - Connect GPIO2 via 10K resistor to VCC
 * - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
 *
  * Inclusion mode button:
 * - Connect GPIO5 via switch to GND ('inclusion switch')
 *
 * Hardware SHA204 signing is currently not supported!
 *
 * Make sure to fill in your ssid and WiFi password below for ssid & pass.
 */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Use a bit lower baudrate for serial prints on ESP8266 than default in MyConfig.h
//#define MY_BAUD_RATE 9600

// Enables and select radio type (if attached)
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_GATEWAY_ESP8266

#define MY_ESP8266_SSID "UPC8268667"
#define MY_ESP8266_PASSWORD "Natalia8Marek8"

// Enable UDP communication
//#define MY_USE_UDP

// Set the hostname for the WiFi Client. This is the hostname
// it will pass to the DHCP server if not static.
#define MY_ESP8266_HOSTNAME "ESP8266sensor-gateway"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_ADDRESS 192,168,0,213

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,0,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0

// The port to keep open on node server mode
#define MY_PORT 5003

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 5

// Controller ip address. Enables client mode (default is "server" mode).
// Also enable this if MY_USE_UDP is used and you want sensor data sent somewhere.
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE

// Enable Inclusion mode button on gateway
// #define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  0

#define MY_NODE_ID 1
// Set blinking period
// #define MY_DEFAULT_LED_BLINK_PERIOD 300

// Flash leds on rx/tx/err
// Led pins used if blinking feature is enabled above
#define MY_DEFAULT_ERR_LED_PIN 2  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  2  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  2  // the PCB, on board LED

#if defined(MY_USE_UDP)
#include <WiFiUdp.h>
#endif

#include <ESP8266WiFi.h>
#include <SPI.h>
#include <MySensors.h>  
#include <Bounce2.h>
#include <Wire.h>
#include <MySensors.h>


#define BUTTON_UP_PIN 4  // Arduino Digital I/O pin number for up button
#define BUTTON_DOWN_PIN 5  // Arduino Digital I/O pin number for down button
//#define BUTTON_STOP_PIN 5  // Arduino Digital I/O pin number for stop button
//#define RELAY_DIR_PIN 6  // Arduino Digital I/O pin number for direction relay
//#define RELAY_POWER_PIN 7  // Arduino Digital I/O pin number for power relay
#define RELAY_UP_PIN 12 
#define RELAY_DOWN_PIN 14
#define RELAY_ON 1
#define RELAY_OFF 0
//#define RELAY_DOWN 1
//#define RELAY_UP 0
#define DIRECTION_DOWN 0
#define DIRECTION_UP 1
#define SKETCH_NAME "Cover by Marek ESP8266"
#define SKETCH_VER "1.0"
#define CHILD_ID_COVER 0   // sensor Id of the sensor child
#define CHILD_ID_ROLLER 1
#define STATE_UP 100 // 100 is open - up
#define STATE_DOWN 0 // 0 is closed - down
//#define CHILD_ID_CALIBRATE 1   // sensor Id of the sensor child to calibrate
#define CHILD_ID_SET 2   // sensor Id of the sensor child to init the roll time
#define PRESENT_MESSAGE "Roller shutter ESP8266"
const int LEVELS = 100; //the number of levels
float rollTime = 20.0; //the overall rolling time of the shutter
const bool IS_ACK = false; //is to acknowlage
static bool initial_state_sent = false;//for hass we need at list one state send at begining

// debouncing parameters
int value = 0;
int oldValueUp = 0;
int oldValueDown = 0;
int oldValueStop = 0;
//static unsigned long last_interrupt_time_up = 0;
//static unsigned long last_interrupt_time_down = 0;
//static unsigned long debounce_time = 200;

Bounce debouncerUp = Bounce();
Bounce debouncerDown = Bounce();
Bounce debouncerStop = Bounce();

// shutter position parameters
float timeOneLevel = rollTime / LEVELS;
int requestedShutterLevel = 0;
int currentShutterLevel = 0;
unsigned long lastLevelTime = 0;
bool isMoving = false;
int directionUpDown;

enum CoverState {
  STOP,
  UP, // Window covering. Up.
  DOWN, // Window covering. Down.
};

static int coverState = STOP;

MyMessage msgUp(CHILD_ID_COVER, V_UP);
MyMessage msgDown(CHILD_ID_COVER, V_DOWN);
MyMessage msgStop(CHILD_ID_COVER, V_STOP);
MyMessage msgPercentage(CHILD_ID_COVER, V_PERCENTAGE);
//MyMessage msgRoller(CHILD_ID_ROLLER, V_DIMMER);
//MyMessage msgCode(CHILD_ID_SET, V_IR_SEND);

void sendState() {
  // Send current state and status to gateway.
//  send(msgUp.set(coverState == UP));
//  send(msgDown.set(coverState == DOWN));
//  send(msgStop.set(coverState == STOP));
  send(msgPercentage.set(currentShutterLevel));
//  send(msgRoller.set(currentShutterLevel));
#ifdef MY_DEBUG
  Serial.print("Send State: ");
  Serial.print(currentShutterLevel);
#endif  
}


void shuttersUp(void) {
  #ifdef MY_DEBUG
  Serial.println("Shutters going up");
  #endif
  if (digitalRead(RELAY_DOWN_PIN) == RELAY_ON) {
    digitalWrite(RELAY_DOWN_PIN, RELAY_OFF);
    wait(50);
  }
  digitalWrite(RELAY_UP_PIN, RELAY_ON);

  directionUpDown = DIRECTION_UP;
  isMoving = true;
  coverState = UP;
  sendState();
}

void shuttersDown(void) {
  #ifdef MY_DEBUG
  Serial.println("Shutters going down");
  #endif
  if (digitalRead(RELAY_UP_PIN) == RELAY_ON) {
    digitalWrite(RELAY_UP_PIN, RELAY_OFF);
    wait(50);
  }
  digitalWrite(RELAY_DOWN_PIN, RELAY_ON);

  directionUpDown = DIRECTION_DOWN;
  isMoving = true;
  coverState = DOWN;
  sendState();
}

void shuttersHalt(void) {
#ifdef MY_DEBUG
  Serial.println("Shutters halted");
#endif
  digitalWrite(RELAY_UP_PIN, RELAY_OFF);
  digitalWrite(RELAY_DOWN_PIN, RELAY_OFF);

  isMoving = false;
  requestedShutterLevel = currentShutterLevel;
#ifdef MY_DEBUG
  Serial.println("saving state to: ");
  Serial.println(String(currentShutterLevel));
#endif
  saveState(CHILD_ID_COVER, currentShutterLevel);
  coverState = STOP;
  sendState();
}

void changeShuttersLevel(int level) {
  int dir = (level > currentShutterLevel) ? DIRECTION_UP : DIRECTION_DOWN;
  if (isMoving && dir != directionUpDown) {
    shuttersHalt();
  }
  requestedShutterLevel = level;
}

void initShutters() {
#ifdef MY_DEBUG
  Serial.println("Init Cover");
#endif
  shuttersUp();
  wait((rollTime + timeOneLevel * LEVELS) * 1000);
  currentShutterLevel = STATE_UP;
  requestedShutterLevel = currentShutterLevel;
}

void receive(const MyMessage &message) {
//#ifdef MY_DEBUG
  Serial.println("recieved incomming message");
  Serial.println("Recieved message for sensor: ");
  Serial.println(String(message.sensor));
  Serial.println("Recieved message with type: ");
  Serial.println(String(message.type));
//#endif
  if (message.sensor == CHILD_ID_COVER) {
    switch (message.type) {
      case V_UP:
        //Serial.println(", New status: V_UP");
        changeShuttersLevel(STATE_UP);
        //state = UP;
        //sendState();
        break;

      case V_DOWN:
        //Serial.println(", New status: V_DOWN");
        changeShuttersLevel(STATE_DOWN);
        //state = DOWN;
        //sendState();
        break;

      case V_STOP:
        //Serial.println(", New status: V_STOP");
        shuttersHalt();
        //state = IDLE;
        //sendState();
        break;

      case V_PERCENTAGE:
        //Serial.println(", New status: V_PERCENTAGE");
        //          if (!initial_state_sent) {
        //            #ifdef MY_DEBUG
        //            Serial.println("Receiving initial value from controller");
        //            #endif
        //            initial_state_sent = true;
        //          }
        int per = message.getInt();
        if (per > STATE_UP) {
          per = STATE_UP;
        }
        changeShuttersLevel(per);
        //InitShutters(message.getInt());//send value < 0 or > 100 to calibrate
        //sendState();
        break;
    }
  } 
else if (message.sensor ==  CHILD_ID_SET) {

    if (message.type == V_VAR1) {
      #ifdef MY_DEBUG
      Serial.println(", New status: V_VAR1, with payload: ");
      #endif      
      String strRollTime = message.getString();
      rollTime = strRollTime.toFloat();
      #ifdef MY_DEBUG
      Serial.println("rolltime value: ");
      Serial.println(String(rollTime));
      #endif
      saveState(CHILD_ID_SET, rollTime);
    }
  }
#ifdef MY_DEBUG
  Serial.println("exiting incoming message");
#endif
  return;
}

void before() {

  // Setup the button
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  // Activate internal pull-up
//  digitalWrite(BUTTON_UP_PIN, HIGH);
  //  attachInterrupt(digitalPinToInterrupt(BUTTON_UP_PIN), upButtonPress, FALLING);

  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  // Activate internal pull-up
//  digitalWrite(BUTTON_DOWN_PIN, HIGH);
  //  attachInterrupt(digitalPinToInterrupt(BUTTON_DOWN_PIN), downButtonPress, FALLING);

//  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  // Activate internal pull-up
//  digitalWrite(BUTTON_STOP_PIN, HIGH);

  // After setting up the button, setup debouncer
  debouncerUp.attach(BUTTON_UP_PIN);
  debouncerUp.interval(5);
  // After setting up the button, setup debouncer
  debouncerDown.attach(BUTTON_DOWN_PIN);
  debouncerDown.interval(5);
  // After setting up the button, setup debouncer
//  debouncerStop.attach(BUTTON_STOP_PIN);
//  debouncerStop.interval(5);

  // Make sure relays are off when starting up
  digitalWrite(RELAY_UP_PIN, RELAY_OFF);
  // Then set relay pins in output mode
  pinMode(RELAY_UP_PIN, OUTPUT);

  // Make sure relays are off when starting up
  digitalWrite(RELAY_DOWN_PIN, RELAY_OFF);
  // Then set relay pins in output mode
  pinMode(RELAY_DOWN_PIN, OUTPUT);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VER);
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_COVER, S_COVER, PRESENT_MESSAGE, IS_ACK);
  present(CHILD_ID_ROLLER, S_DIMMER, PRESENT_MESSAGE, IS_ACK);  
  // present(CHILD_ID_SET, S_CUSTOM);
}

void setup(void) {
  //set up roll time if the saved value is not 255
  #ifdef MY_DEBUG
  Serial.println("getting rolltime from eeprom: ");
  #endif
  float tmpRollTime = loadState(CHILD_ID_SET);
  if (tmpRollTime != 0xff) {
    rollTime = tmpRollTime;
  }
  #ifdef MY_DEBUG
  Serial.println(String(rollTime));
  #endif
  
  int state = loadState(CHILD_ID_COVER);
  
  #ifdef MY_DEBUG
  Serial.println("getting state from eeprom: ");
  Serial.println(String(state));
  #endif
  
  if (state == 0xff) {
    initShutters();
  } else {
    changeShuttersLevel(state);
  }
}

void loop(void) {
  if (!initial_state_sent) {
#ifdef MY_DEBUG
    Serial.println("Sending initial value");
#endif
    sendState();
    
   // send(msgCode.set('20.0'));
    //    #ifdef MY_DEBUG
    //    Serial.println("Requesting initial value from controller");
    //    #endif
    //    request(CHILD_ID_COVER, V_PERCENTAGE);
    //    wait(2000, C_SET, V_PERCENTAGE);
    initial_state_sent = true;
  }

  debouncerUp.update();
  value = debouncerUp.read();
  if (value == 0 && value != oldValueUp) {
    if(isMoving){
      shuttersHalt();
    }  
    else{
    changeShuttersLevel(STATE_UP);
    }
    //state = UP;
    //sendState();
  }
  oldValueUp = value;

  debouncerDown.update();
  value = debouncerDown.read();
  if (value == 0 && value != oldValueDown) {
    if(isMoving){
      shuttersHalt();
    }  
    else{
    changeShuttersLevel(STATE_DOWN);
    }    
    //state = DOWN;
    //sendState();
  }
  oldValueDown = value;

/*  debouncerStop.update();
  value = debouncerStop.read();
  if (value == 0 && value != oldValueStop) {
    shuttersHalt();
    //state = IDLE;
    //sendState();
  }
  oldValueStop = value;
*/

  if (isMoving) {
    unsigned long _now = millis();
    if (_now - lastLevelTime >= timeOneLevel * 1000) {
      if (directionUpDown == DIRECTION_UP) {
        currentShutterLevel += 1;
      } else {
        currentShutterLevel -= 1;
      }
      #ifdef MY_DEBUG
      Serial.println(String(requestedShutterLevel));
      Serial.println(String(currentShutterLevel));
      #endif
      lastLevelTime = millis();
      send(msgPercentage.set(currentShutterLevel));
    }
    if (currentShutterLevel == requestedShutterLevel) {
      shuttersHalt();
    }
  } 
  else if (requestedShutterLevel != currentShutterLevel) {
    if (requestedShutterLevel > currentShutterLevel) {
      shuttersUp();
    }
    else {
      shuttersDown();
    }
    lastLevelTime = millis();
  }
}

