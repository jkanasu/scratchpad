// Some times we cannot use the default Serial for debug/logs, hence we use SoftwareSerial
// You could use a spare Hardware Serial on boards that have it (like Mega)
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(10, 11); // RX, TX

#define JAGI_PRINT DebugSerial
#define JAGI_LOG_TIME {JAGI_PRINT.print(F("[")); JAGI_PRINT.print(millis());JAGI_PRINT.print(F("] "));}
#define JAGI_LOG1(pLast) {JAGI_LOG_TIME; JAGI_PRINT.println(pLast);}
#define JAGI_LOG2(p1,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.println(pLast);}
#define JAGI_LOG3(p1,p2,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.print(p2); JAGI_PRINT.println(pLast);}

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
IRsend mySender;

#include <IRLibRecv.h>
#include <IRLibRecvPCI.h>

IRrecvPCI myReceiver(2); //create receiver and pass pin number
IRdecode myDecoder;   //create decoder
//Create a buffer that we will use for decoding one stream while
//the receiver is using the default buffer "recvGlobal.recvBuffer"
// NOTE this eats away 100 x 16 bits -- i.e. 200 bytes
uint16_t myBuffer[RECV_BUF_LENGTH];

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT DebugSerial

#include <BlynkSimpleStream.h>

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
  initializeLocalHardware();
  identifySelf();
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  BLYNK_LOG2(F("Done initializing "), F("jagi"));
}

void loop()
{
  Blynk.run();
  receiveIRSignals();
}

unsigned int DEVICE_IDENTITY=0;
boolean MASTER_MODE=false;

#include <RingBufCPP.h>
typedef struct {
  byte protocolNum;
  unsigned long timestamp;
  unsigned long value;
} NecIrSignal;
#define MAX_NUM_NECSIGNALS_BUF 10
//NECSIGNAL gIRSignalsNEC[MAX_NUM_NECSIGNALS];
// Stack allocate the buffer to hold IR signals structs
RingBufCPP<NecIrSignal, MAX_NUM_NECSIGNALS_BUF> buf;

// NEC is used for our system
// COMMAND & RESPONSE differ in only the MSB value
// COMMAND has MSB = 0
// RESPONSE has MSB = 1
#define REQUEST_HB  0x01
#define RESPONSE_HB 0x81
#define REQUEST_LIGHTSTATUS  0x02
#define RESPONSE_LIGHTSTATUS 0x82
#define REQUEST_FANSPEED  0x03
#define RESPONSE_FANSPEED 0x83

unsigned long rawIRSignalsSent = 0;
unsigned long rawIRSignalsEchoFailed = 0;
unsigned long rawIRSignalsReTransmitted = 0;
unsigned long recentTransmitted = 0xFFFFFFFF;
boolean isWaitingForEcho = false;
boolean isEchoReceived = false;
boolean isReTransmissionRequired = false;
#define ECHO_TIMEOUT_PERIOD 2000 // In milliseconds 125 // For NEC standard, the transmission period is upto 110 ms
#define NUM_IRSIGNAL_MIN_RETRANSMITS 2
#define NUM_IRSIGNAL_MAX_RETRANSMITS 4
void sendIRSignals(unsigned long iRcode){
  BLYNK_LOG_TIME();JAGI_PRINT.print(F("Send raw Ir Signal "));JAGI_PRINT.println(iRcode, HEX);
  recentTransmitted = 0xFFFFFFFF;
  isWaitingForEcho = false;
  isEchoReceived = false;
  isReTransmissionRequired = true;
  //recentTransmitted = iRcode;
  //mySender.send(NEC,iRcode,32);
  //isWaitingForEcho = true;
  //receiveIRSignals();
  byte i = 0;
  unsigned long sendTimeStamp = millis();
  do{
    if(isReTransmissionRequired == true){
      i++;
      recentTransmitted = iRcode;
      unsigned long sendTimeStampBefore = millis();
      mySender.send(NEC,iRcode,32);
      sendTimeStamp = millis();
      long timeTakenToSend = sendTimeStamp - sendTimeStampBefore;
      BLYNK_LOG6(F(" Tx attempt "),i,F(" took "),timeTakenToSend, F(" "), F("seconds"));
      //JAGI_PRINT.print(F(" Tx attempt ")); JAGI_PRINT.print(i);JAGI_PRINT.print(F(" : "));
      //JAGI_PRINT.print(sendTimeStamp); JAGI_PRINT.print(F(" - "));JAGI_PRINT.print(sendTimeStampBefore); JAGI_PRINT.print(F(" = "));JAGI_PRINT.println(timeTakenToSend,DEC);
      rawIRSignalsSent++;
      isWaitingForEcho = true;
      isReTransmissionRequired = false;
    }
    while(true){
      receiveIRSignals();
      if(isEchoReceived){
        //Serial.println(F("Echo Received"));
        break;
      }
      if(hasTimedOut(sendTimeStamp, ECHO_TIMEOUT_PERIOD)){
        BLYNK_LOG1(F(" Echo Timed Out"));
        isReTransmissionRequired = true;
        rawIRSignalsEchoFailed++;
        rawIRSignalsReTransmitted++;
        break;
      }
    }
    if(isEchoReceived ) {
      if(i < NUM_IRSIGNAL_MIN_RETRANSMITS) {
        isReTransmissionRequired = true;
        rawIRSignalsReTransmitted++;
        //Serial.print(F("Force Tx attempt ")); Serial.println(i);
        //delay(200);// else we may simply collide with our selves, actually NO :-(
        continue;
      }
      break;
    }
  }while(i < NUM_IRSIGNAL_MAX_RETRANSMITS);
  if(isEchoReceived) {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("Raw Send Signal complete "));JAGI_PRINT.println(iRcode, HEX);
  }
}

unsigned long rawIRSignalsReceived = 0;
unsigned long rawIRSignalsDecodeFailures = 0;
unsigned long rawIRSignalsDuplicates = 0;
unsigned long rawIRSignalsDropped = 0;
boolean fullDetail=false;
void receiveIRSignals(){
  //Continue looping until you get a complete signal received
  if (myReceiver.getResults()) { // get the Results
    rawIRSignalsReceived++;
    if (myDecoder.decode()){  //Decode it
      //myDecoder.dumpResults(fullDetail);  //Now print results. Use false for less detail
      NecIrSignal necIrSignal;
      necIrSignal.protocolNum = myDecoder.protocolNum;
      necIrSignal.value = myDecoder.value;
      necIrSignal.timestamp = millis();
      byte bufLength = buf.numElements();
      if(isWaitingForEcho && necIrSignal.value == recentTransmitted){
        isEchoReceived = true;
        isWaitingForEcho = false;
        //Serial.print(F("Echo Received ")); Serial.println(necIrSignal.value, HEX);
        goto enableReceiver;
      }
      if(bufLength>0){
        NecIrSignal* prevElement = buf.peek(bufLength-1);
        //if(!prevElement) goto enableReceiver; // Needed only in multithreading or ISR
        if(necIrSignal.value == 0xFFFFFFFF || necIrSignal.value == prevElement->value) {
          //Serial.print(F("Duplicate found ")); Serial.println(necIrSignal.value, HEX);
          rawIRSignalsDuplicates++;
          goto enableReceiver;
        }
      }
      if(!buf.add(necIrSignal)) {    // Add it to the buffer
        BLYNK_LOG_TIME();JAGI_PRINT.print(F("Q failed "));JAGI_PRINT.println(necIrSignal.value, HEX);
        rawIRSignalsDropped++;
      }
    } else { // This is an indication of possible collision or a simple hardware problem, hence re-transmit
      BLYNK_LOG1(F("Decode failed"));
      rawIRSignalsDecodeFailures++;
      if(isWaitingForEcho){
        rawIRSignalsEchoFailed++;// = 0;
        isReTransmissionRequired = true;
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Raw Echo failed for "));JAGI_PRINT.println(recentTransmitted, HEX);
        goto enableReceiver;
      }
    }
enableReceiver:
    myReceiver.enableIRIn();      //Restart receiver
  }
}

// TODO Need to check so that the overflow doesnot cause too much time to timeout
#define TIME_OVERFLOW_VALUE 0xFFFFFFFF
unsigned long findElapsedTimeInMillis(const unsigned long& lastEventTime){
  unsigned long retVal;
  unsigned long currentTime = millis();
  retVal = currentTime - lastEventTime;
  if(retVal < 0 ) { // Overflow is automatically taken care with unsigned numbers, we need not do this.
    retVal = (TIME_OVERFLOW_VALUE - lastEventTime) + currentTime;
  }
  return retVal;
}

boolean hasTimedOut(const unsigned long& lastEventTime, const unsigned long& timeOutPeriod){
  long timeRemaining = timeOutPeriod - findElapsedTimeInMillis(lastEventTime);
  //BLYNK_LOG2((F("timeRemaining "),(timeRemaining));
  if (timeRemaining > 0 ) return false;
  return true;
}

unsigned long formatCodeForSendingRequest(byte destinationAddress, byte command, byte commandParams){
  return formatCodeForSendingRequest(destinationAddress, DEVICE_IDENTITY, command, commandParams);
}
unsigned long formatCodeForSendingRequest(byte destinationAddress, byte sourceAddress, byte command, byte commandParams){
  return formatCodeForSending(true, destinationAddress, sourceAddress, command, commandParams);
}
unsigned long formatCodeForSendingResponse(byte destinationAddress, byte command, byte resultData){
  return formatCodeForSendingResponse(destinationAddress, DEVICE_IDENTITY, command, resultData);
}
unsigned long formatCodeForSendingResponse(byte destinationAddress, byte sourceAddress, byte command, byte resultData){
  return formatCodeForSending(false, destinationAddress, sourceAddress, command, resultData);
}

// typeOfSignal will indicate whether it is a request or it is a response to a request
// The command is expected to be only 7 bits, the MSB is reserved to indicate request/response
// commandParams is the inputs to the command while sendingRequest & it is resultData while sendingResponse
// In this function MSB of command is forcefully modified MSB=0 means request, MSB=1 means response
unsigned long formatCodeForSending(boolean typeOfSignal, byte destinationAddress, byte sourceAddress, byte command, byte commandParams){
  unsigned long retVal = 0;
  //Serial.print(typeOfSignal, HEX); Serial.print("-");Serial.print(destinationAddress, HEX); Serial.print("-");Serial.print(sourceAddress, HEX); Serial.print("-");
  //Serial.print(command, HEX); Serial.print("-");Serial.println(commandParams, HEX);
  retVal = retVal | destinationAddress;
  retVal = retVal << 8;
  //Serial.println(retVal, HEX); 
  retVal = retVal | sourceAddress;
  retVal = retVal << 8;
  //Serial.println(retVal, HEX); 
  if(typeOfSignal) command = command & 0x7F; // always set the MSB to 0 to indicate it is a request
  else command = command | 0x80; // always set the MSB to 1 to indicate it is a response
  retVal = retVal | command;
  retVal = retVal << 8;
  //Serial.println(retVal, HEX); 
  retVal = retVal | commandParams;
  //Serial.println(retVal, HEX);
  return retVal;
}

// Print the buffer's contents then empty it
void print_buf_contents()
{
  NecIrSignal * e = 0;
  JAGI_LOG1("\n______Peek contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  for (int i = 0 ; i<buf.numElements(); i++) {
    e = buf.peek(i);
    if(!e) break;
    JAGI_LOG_TIME; JAGI_PRINT.print(F("t ")); JAGI_PRINT.print(e->timestamp); JAGI_PRINT.print(F(" v ")); JAGI_PRINT.println(e->value, HEX);
  }
  JAGI_LOG1("______Done peeking contents_______");
}

// Print the buffer's contents then empty it
void dump_buf_contents()
{
  NecIrSignal e;
  JAGI_LOG1("\n______Dumping contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  while (buf.pull(&e))
  {
    JAGI_LOG_TIME; JAGI_PRINT.print(F("t ")); JAGI_PRINT.print(e.timestamp); JAGI_PRINT.print(F(" v ")); JAGI_PRINT.println(e.value, HEX);
  }
  JAGI_LOG1("______Done dumping contents_______");
}

void printIrSignalStats(){
  BLYNK_LOG1(F("RawTx\tEchoFail\tReTransmitted"));
  BLYNK_LOG6(rawIRSignalsSent, F("\t"), rawIRSignalsEchoFailed, F("\t"), rawIRSignalsReTransmitted, F(""));
  BLYNK_LOG1(F("RawRx\tDecodeFail\tDuplicate\tDropped"));
  BLYNK_LOG_TIME(); JAGI_PRINT.print(rawIRSignalsReceived,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.print(rawIRSignalsDecodeFailures,DEC);JAGI_PRINT.print(F("\t"));
  JAGI_PRINT.print(rawIRSignalsDuplicates,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.println(rawIRSignalsDropped,DEC);
}

#define pinMasterMode  A5

#define pinLedRed      A0
#define pinLedGreen    A1
#define pinLedBlue     A2
#define pinLedYellow   A3

#define VALUE_TO_CHECK LOW // Set this to HIGH if you are using a common cathode LED

void identifySelf(){
  //to identify the MASTER_MODE or SLAVE_MODE, configure pin as an input and enable the internal pull-up resistor
  pinMode(pinMasterMode, INPUT_PULLUP);
  //to identify the DEVICE configure pins as input and enable the internal pull-up resistor
  pinMode(pinLedRed, INPUT_PULLUP);
  pinMode(pinLedGreen, INPUT_PULLUP);
  pinMode(pinLedBlue, INPUT_PULLUP);
  pinMode(pinLedYellow, INPUT_PULLUP);
  // identify the self device id and the mode of operation
  int sensorVal = HIGH;
  //read the master mode pin, print out the mode
  sensorVal = digitalRead(pinMasterMode);
  if(sensorVal == LOW) {MASTER_MODE = true;}
  else {MASTER_MODE = false;}
  //read the pin values into a variable
  sensorVal = digitalRead(pinLedRed);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0008; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFF7; }
  sensorVal = digitalRead(pinLedGreen);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0004; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFB; }
  sensorVal = digitalRead(pinLedBlue);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0002; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFD; }
  sensorVal = digitalRead(pinLedYellow);
  if (sensorVal == VALUE_TO_CHECK) { DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0001; }
  else { DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFE; }
  printIdentity();
}

void printIdentity(){
  //if(MASTER_MODE) Serial.print(F("MASTER")); else Serial.print(F("SLAVE "));
  //Serial.print(" 0x");Serial.print(DEVICE_IDENTITY,HEX); Serial.print(" Color ");
  char color[] = "----";
  if (DEVICE_IDENTITY & 0x0008) color[0] = 'R';
  if (DEVICE_IDENTITY & 0x0004) color[1] = 'G';
  if (DEVICE_IDENTITY & 0x0002) color[2] = 'B';//Serial.print("B"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0001) color[3] = 'Y';//Serial.println("Y"); else Serial.println("-");
  if(MASTER_MODE) {BLYNK_LOG4("MASTER ", DEVICE_IDENTITY, " Color ", color);}
  else {BLYNK_LOG4("SLAVE  ", DEVICE_IDENTITY, " Color ", color);}
}


// To Select/Enable a pin, connect it to gnd
#define pinDoorState  8   // arbitrarily we chose digital pin D8
#define pinTemperature A4 // we know in our setup analog pin A4 is free

void initializeLocalHardware(){
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  //to identify whether the door is open or not, configure pin as an input and enable the internal pull-up resistor
  //if the pin is HIGH it means it is open, if it is grounded/LOW it means it is closed.
  pinMode(pinDoorState, INPUT_PULLUP);
}
// Below is for Red 0x8 V0,V1,V2,V3
// This function is called when there is a Widget
// which is writing data to Virtual Pin (0)
BLYNK_WRITE(V0)
{
  // This command reads the value from Virtual Pin (0)
  //Blynk.virtualWrite(V1, millis() / 1000);
  int requiredLedState = param.asInt();
  if(requiredLedState == 0) digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
  else digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  BLYNK_LOG1(F("Wrote to inbuilt led"));
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

// Below is for Blue 0x4 V4,V5,V6,V7
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

// Below is for Green 0x2 V8,V9,V10,V11
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

// Below is for Yellow 0x1 V12,V13,V14,V15
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

// Below are all for some debugging
BLYNK_WRITE(V29)
{
  print_buf_contents();
}

BLYNK_WRITE(V30)
{
  identifySelf();
}

BLYNK_WRITE(V31)
{
  printIrSignalStats();
}

void processSerial8(){
  print_buf_contents();
}

void processSerial9(){
  identifySelf();
}

void processSerialE(){
  dump_buf_contents();
}

void processSerialF(){
  JAGI_LOG1(F("Change dump level"));
  fullDetail = (!fullDetail);
}

void processSerialX(){
  JAGI_LOG1(F("Print Stats"));
  printIrSignalStats();
}


