/* send.ino Example sketch for IRLib2
 *  Illustrates how to send a code.
 */
#include <IRLibDecodeBase.h>
#include <IRLibSendBase.h>    // First include the send base
//Now include only the protocols you wish to actually use.
//The lowest numbered protocol should be first but remainder 
//can be any order.
#include <IRLib_P01_NEC.h>
#include <IRLib_P03_RC5.h>   
#include <IRLibCombo.h>     // After all protocols, include this
// All of the above automatically creates a universal sending
// class called "IRsend" containing only the protocols you want.
// Now declare an instance of that sender.
#include <IRLibRecv.h>
#include <IRLibRecvPCI.h>

IRrecvPCI myReceiver(2); //create receiver and pass pin number
IRdecode myDecoder;   //create decoder
//Create a buffer that we will use for decoding one stream while
//the receiver is using the default buffer "recvGlobal.recvBuffer"
// NOTE this eats away 100 x 16 bits -- i.e. 200 bytes
//uint16_t myBuffer[RECV_BUF_LENGTH];

IRsend mySender;

// VU TV remote codes is RC5 and codes toggle 10xx and 18xx when the button is pressed again
#define BUTTON_INFO 0x100F
#define BUTTON_INFO_REPEAT 0x180F
#define BUTTON_OK 0x101B
#define BUTTON_OK_REPEAT 0x181B
#define BUTTON_2 0x1002 // 0x1802
#define BUTTON_4 0x1004 // 0x1804
#define BUTTON_7 0x1007 // 0x1807

// NEC continous button press will be 0xFFFFFFFF
#define BUTTON_RED_CHNL_MINUS 0xFFA25D  // for Red identifier 
#define BUTTON_GREEN_PREV 0xFF22DD  // for Green identifier 
#define BUTTON_MINUS_BLUE 0xFFE01F  // for Blue identifier 
#define BUTTON_EQ 0xFF906F  // for heartbeat 
#define HEARTBEAT_REQUEST 0xFF906F 
#define HEARTBEAT_RESPONSE_VU 0x101B
#define HEARTBEAT_RESPONSE_VU_REPEAT 0x181B
//#define HEARTBEAT_RESPONSE 0xFFB04F
//#define HEARTBEAT_RESPONSE mySender.send(NEC,0x61a0f00f,0);

//#define HEARTBEAT_RESPONSE NEC,0x61a0f00f,0 //RC5 - Also 0x101B when same button pressed
#define HEARTBEAT_RESPONSE_SEND_PARAMS RC5,0x101B,13 //RC5 - Also 0x181B when same button pressed

#define pinLedRed   11
#define pinLedAnode 10
#define pinLedGreen 9
#define pinLedBlue  8

void setup() {
  Serial.begin(9600);
  delay(2000); while (!Serial); //delay for Leonardo
  //configure pins to identify as an input and enable the internal pull-up resistor
  pinMode(pinLedRed, INPUT_PULLUP);
  pinMode(pinLedGreen, INPUT_PULLUP);
  pinMode(pinLedBlue, INPUT_PULLUP);
  // configure pin as power for led, Let this be high initially supplying power to the LED
  pinMode(pinLedAnode, OUTPUT);
  digitalWrite(pinLedAnode, HIGH);
  Serial.println(F("Every time you press a key is a serial monitor we will send."));
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  Serial.println(F("Ready to receive IR signals"));
}

void loop() {
  if (Serial.read() != -1) {
    //send a code every time a character is received from the 
    // serial port. You could modify this sketch to send when you
    // push a button connected to an digital input pin.
    sendSignal();
  }
  receiveSignal();
}

int numOfSignalsSent = 0;
void sendSignal(){
  Serial.println(F("Sending signal"));
  numOfSignalsSent++;
  //Substitute values and protocols in the following statement
  // for device you have available.
  //mySender.send(SONY,0xa8bca, 20);//Sony DVD power A8BCA, 20 bits
  //mySender.send(NEC,0x61a0f00f,0);//NEC TV power button=0x61a0f00f
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  receiveSignal();
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x101B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  delay(500);
  mySender.send(RC5,0x181B, 13);//VU TV ok A8BCA, 13 bits
  receiveSignal();
  Serial.print(F("Sent signal "));  Serial.println(numOfSignalsSent, DEC);
}

int numOfSignalsReceived = 0;
void receiveSignal() {
  if (myReceiver.getResults()) {
    numOfSignalsReceived++;
    Serial.print(F("numOfSignalsReceived is ")); Serial.println(numOfSignalsReceived, DEC);
    Serial.println(F("decoding signal"));
    if(myDecoder.decode()) {
      //myDecoder.dumpResults(true);  //Now print results. Use false for less detail
      unsigned long decodedValue = myDecoder.value;
      myReceiver.enableIRIn();      //Restart receiver
      Serial.println(F("enabled receiver after decoding"));
      Serial.print(F("Recvd signal "));Serial.println(decodedValue,HEX);
      switch(decodedValue) {
        case HEARTBEAT_REQUEST:
        case BUTTON_INFO:
        case BUTTON_INFO_REPEAT:
          respondHeartBeatRequest();
          break;        
        case HEARTBEAT_RESPONSE_VU:
        case HEARTBEAT_RESPONSE_VU_REPEAT:
          processHeartBeatResponse();
          break;
        default:
          Serial.println(F("unknown signal"));//Serial.println(decodedValue,HEX);
          break;
      }
    } else {
      myReceiver.enableIRIn();      //Restart receiver
      Serial.println(F("enabled receiver failed decode"));
    }
  }
}

int numOfHeartBeatRequests = 0;
int previousState = BUTTON_OK;
void respondHeartBeatRequest(){//int receivedCode){
  numOfHeartBeatRequests++;
  // the receivedCode can be checked to see if this the device to respond
  if(previousState == BUTTON_OK) previousState = BUTTON_OK_REPEAT; else previousState = BUTTON_OK;
  Serial.print("HB requests ");Serial.print(numOfHeartBeatRequests, DEC);Serial.print(" response send ");Serial.println(previousState, HEX);
  mySender.send(RC5, previousState, 0);
  // simply blink the self device's identification LED
  changePinValue(pinLedAnode);
}

int numOfHeartBeatResponses = 0;
void processHeartBeatResponse(){//int receivedCode){
  numOfHeartBeatResponses++;
  // the receivedCode can be checked to see if this a self response or not
  Serial.print(F("HB response received "));Serial.println(numOfHeartBeatResponses, DEC);
}

// this will represent the identity of the device
int currentLedColor = 0;
void identifySelf(){
  //read the pin values into a variable
  //print out the color of value
  int sensorVal = HIGH;
  sensorVal = digitalRead(pinLedRed);
  if(sensorVal == LOW) {currentLedColor = pinLedRed; Serial.print(F("R"));}
  else Serial.print(F("-"));
  sensorVal = digitalRead(pinLedGreen);
  if(sensorVal == LOW) {currentLedColor = pinLedGreen; Serial.print(F("G"));}
  else Serial.print(F("-"));
  sensorVal = digitalRead(pinLedBlue);
  if(sensorVal == LOW) {currentLedColor = pinLedBlue; Serial.print(F("B"));}
  else Serial.print(F("-"));
  Serial.println(F(""));
  // simply keep blinking the inbuilt LED
  //changePinValue(LED_BUILTIN);
}

// this is used to blink the identity led
void changePinValue(int pinNumber){
  digitalWrite(pinNumber, LOW);
  delay(50);
  digitalWrite(pinNumber, HIGH);
}

