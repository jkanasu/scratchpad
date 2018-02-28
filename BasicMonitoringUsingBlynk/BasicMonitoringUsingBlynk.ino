/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  =>
  =>          USB HOWTO: http://tiny.cc/BlynkUSB
  =>

  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT DebugSerial

// You could use a spare Hardware Serial on boards that have it (like Mega)
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(2, 3); // RX, TX

#include <BlynkSimpleStream.h>

// To Select/Enable a pin, connect it to gnd
#define pinDoorState  8   // arbitrarily chose digital pin D8
#define pinTemperature A3 // we know in our setup analog pin A3 is free

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "762e2abc93414fa0966dc4b101885452";

void setup()
{
  // Debug console
  DebugSerial.begin(9600);

  // Blynk will work through Serial
  // Do not read or write this serial manually in your sketch
  Serial.begin(9600);
  Blynk.begin(Serial, auth);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //to identify whether the door is open or not, configure pin as an input and enable the internal pull-up resistor
  //if the pin is HIGH it means it is open, if it is grounded/LOW it means it is closed.
  pinMode(pinDoorState, INPUT_PULLUP);
}

void loop()
{
  Blynk.run();
}

// This function is called when there is a Widget
// which is writing data to Virtual Pin (0)
BLYNK_WRITE(V0)
{
  // This command reads the value from Virtual Pin (0)
  //Blynk.virtualWrite(V1, millis() / 1000);
  int requiredLedState = param.asInt();
  if(requiredLedState == 0) digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  else digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

// This function is called when there is a Widget
// which is requesting data from Virtual Pin (1)
BLYNK_READ(V1)
{
  // This command writes Arduino's uptime in seconds to Virtual Pin (5)
  Blynk.virtualWrite(V1, millis() / 1000);
}

BLYNK_READ(V2)
{
  // read the input pin:
  int pinState = digitalRead(pinDoorState);
  // This command writes Arduino's digital pin 8's current state to Virtual Pin (2)
  if(pinState == HIGH) Blynk.virtualWrite(V2, "OPEN");
  else Blynk.virtualWrite(V2, "CLOSED");
}

BLYNK_READ(V3)
{
  // read the input on analog pin 0:
  int temperatureSensorValue = analogRead(pinTemperature);
  // Read the This command writes Arduino's uptime in seconds to Virtual Pin (3)
  Blynk.virtualWrite(V3, temperatureSensorValue);//random(0,1000));
}


// This function is called when there is a Widget
// which is writing data to Virtual Pin (0)
BLYNK_WRITE(V4)
{
  // This command reads the value from Virtual Pin (0)
  //Blynk.virtualWrite(V1, millis() / 1000);
  int requiredLedState = param.asInt();
  if(requiredLedState == 0) digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  else digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

// This function is called when there is a Widget
// which is requesting data from Virtual Pin (1)
BLYNK_READ(V5)
{
  // This command writes Arduino's uptime in seconds to Virtual Pin (5)
  Blynk.virtualWrite(V5, millis() / 1000);
}

BLYNK_READ(V6)
{
  // read the input pin:
  int pinState = digitalRead(pinDoorState);
  // This command writes Arduino's digital pin 8's current state to Virtual Pin (2)
  if(pinState == HIGH) Blynk.virtualWrite(V6, "OPEN");
  else Blynk.virtualWrite(V6, "CLOSED");
}

BLYNK_READ(V7)
{
  // read the input on analog pin 0:
  int temperatureSensorValue = analogRead(pinTemperature);
  // Read the This command writes Arduino's uptime in seconds to Virtual Pin (3)
  Blynk.virtualWrite(V7, temperatureSensorValue);//random(0,1000));
}

//
// This function is called when there is a Widget
// which is writing data to Virtual Pin (0)
BLYNK_WRITE(V8)
{
  // This command reads the value from Virtual Pin (0)
  //Blynk.virtualWrite(V1, millis() / 1000);
  int requiredLedState = param.asInt();
  if(requiredLedState == 0) digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  else digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

// This function is called when there is a Widget
// which is requesting data from Virtual Pin (1)
BLYNK_READ(V9)
{
  // This command writes Arduino's uptime in seconds to Virtual Pin (5)
  Blynk.virtualWrite(V9, millis() / 1000);
}

BLYNK_READ(V10)
{
  // read the input pin:
  int pinState = digitalRead(pinDoorState);
  // This command writes Arduino's digital pin 8's current state to Virtual Pin (2)
  if(pinState == HIGH) Blynk.virtualWrite(V10, "OPEN");
  else Blynk.virtualWrite(V10, "CLOSED");
}

BLYNK_READ(V11)
{
  // read the input on analog pin 0:
  int temperatureSensorValue = analogRead(pinTemperature);
  // Read the This command writes Arduino's uptime in seconds to Virtual Pin (3)
  Blynk.virtualWrite(V11, temperatureSensorValue);//random(0,1000));
}

// This function is called when there is a Widget
// which is writing data to Virtual Pin (0)
BLYNK_WRITE(V12)
{
  // This command reads the value from Virtual Pin (0)
  //Blynk.virtualWrite(V1, millis() / 1000);
  int requiredLedState = param.asInt();
  if(requiredLedState == 0) digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  else digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

// This function is called when there is a Widget
// which is requesting data from Virtual Pin (1)
BLYNK_READ(V13)
{
  // This command writes Arduino's uptime in seconds to Virtual Pin (5)
  Blynk.virtualWrite(V13, millis() / 1000);
}

BLYNK_READ(V14)
{
  // read the input pin:
  int pinState = digitalRead(pinDoorState);
  // This command writes Arduino's digital pin 8's current state to Virtual Pin (2)
  if(pinState == HIGH) Blynk.virtualWrite(V14, "OPEN");
  else Blynk.virtualWrite(V14, "CLOSED");
}

BLYNK_READ(V15)
{
  // read the input on analog pin 0:
  int temperatureSensorValue = analogRead(pinTemperature);
  // Read the This command writes Arduino's uptime in seconds to Virtual Pin (3)
  Blynk.virtualWrite(V15, temperatureSensorValue);//random(0,1000));
}


