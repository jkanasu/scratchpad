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
uint16_t myBuffer[RECV_BUF_LENGTH];

IRsend mySender;

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

// VU TV remote codes is RC5 and codes toggle 10xx and 18xx when the button is pressed again
#define BUTTON_INFO 0x100F
#define BUTTON_INFO_REPEAT 0x180F
#define BUTTON_RECALL 0x1022
#define BUTTON_RECALL_REPEAT 0x1822
#define BUTTON_OK 0x101B
#define BUTTON_OK_REPEAT 0x181B
#define BUTTON_2 0x1002 // 0x1802
#define BUTTON_4 0x1004 // 0x1804
#define BUTTON_7 0x1007 // 0x1807

// NEC continous button press will be 0xFFFFFFFF
#define BUTTON_RED_CHNL_MINUS 0xFFA25D  // for Red identifier 
#define BUTTON_GREEN_PREV 0xFF22DD  // for Green identifier 
#define BUTTON_BLUE_MINUS 0xFFE01F  // for Blue identifier 
#define BUTTON_EQ 0xFF906F  // for heartbeat 
#define HEARTBEAT_REQUEST 0xFF906F 
#define HEARTBEAT_RESPONSE_VU 0x101B
#define HEARTBEAT_RESPONSE_VU_REPEAT 0x181B
//#define HEARTBEAT_RESPONSE 0xFFB04F
//#define HEARTBEAT_RESPONSE mySender.send(NEC,0x61a0f00f,0);

//#define HEARTBEAT_RESPONSE NEC,0x61a0f00f,0 //RC5 - Also 0x101B when same button pressed
//#define HEARTBEAT_RESPONSE_SEND_PARAMS RC5,0x101B,13 //RC5 - Also 0x181B when same button pressed
//#define SAMSUNG_SEND_PARAMS SAMSUNG36,0xc837,0x0400 // first is 'data'(20bits) and second is 'address'(16bits)

typedef struct{
  byte protocolNum;
  unsigned long value;
  // four bytes received from the 32 bit value from IR
  byte destinationAddress = 0;
  byte sourceAddress = 0;
  byte command = 0;
  byte parameterData = 0;
} JIRCode;

unsigned int DEVICE_IDENTITY=0;
boolean MASTER_MODE=false;

void setup() {
  Serial.begin(9600);
  delay(2000); while (!Serial); //delay for Leonardo
  // identify the self device id and the mode of operation
  identifySelf();
  if(MASTER_MODE)Serial.println(F("Every time you press a key in serial monitor monitoring using HB will be done."));
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  Serial.println(F("Ready to receive IR signals"));
}

void loop() {
  processSerialCommands();
  if(MASTER_MODE) deviceMonitoring();
  else processIRSignals();
}

/* State machine
 *  
 */
#define HB_NOT_SENT 0
#define HB_REQUEST_SENT 10
#define HB_RESPONSE_WAITING 20
#define HB_RESPONSE_TIMEOUT 30
#define HB_RESPONSE_RECEIVED 40
#define HB_FAILED 50
#define HB_SUCCESS 60
typedef struct{
  //byte deviceId; // Should this be here or outside?? :-(
  byte hbStatus = HB_NOT_SENT;
  unsigned long lastEventTime = 0;
} DeviceHBStatus;
#define HB_INTERVAL 10000 // 10 seconds 
#define HB_TIMEOUT_PERIOD 2000 // 2 second
#define MAX_NUMBER_OF_DEVICES 4
DeviceHBStatus hbStatusOfDevices[MAX_NUMBER_OF_DEVICES+1];
byte gCurrentDeviceId = 1; // indexing can all go wrong keep note
unsigned long gHBsignalNumber = 0;
void deviceMonitoring(){
  if(gCurrentDeviceId>MAX_NUMBER_OF_DEVICES) {
    gCurrentDeviceId = 1;
    Serial.println(F("Restarting from beginning"));
  }
  DeviceHBStatus& currentDeviceStatus = hbStatusOfDevices[gCurrentDeviceId];
  if(currentDeviceStatus.hbStatus == HB_NOT_SENT){
    gHBsignalNumber++;
    sendHBRequest(gCurrentDeviceId);
    currentDeviceStatus.hbStatus = HB_RESPONSE_WAITING;
    currentDeviceStatus.lastEventTime = millis();
    //Serial.print(F("HB request sent "));Serial.println(gCurrentDeviceId, DEC);
    return;
  }
  if(currentDeviceStatus.hbStatus == HB_RESPONSE_WAITING){
    // someone else is expected to change this status
    // how long are we waiting?
    unsigned long waitingTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    if(waitingTime > HB_TIMEOUT_PERIOD){
      // donot put any delay here, the main loop() may need that time to do some work
      currentDeviceStatus.hbStatus = HB_FAILED;
      Serial.print(F("HB timedout "));Serial.println(gCurrentDeviceId, DEC);
      return;
    }
    processIRSignals();
    return;
  }
  if(currentDeviceStatus.hbStatus == HB_FAILED){
    Serial.print(F("HB Failed "));Serial.println(gCurrentDeviceId, DEC);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    gCurrentDeviceId++;
    return;
  }
  if(currentDeviceStatus.hbStatus == HB_SUCCESS){
    Serial.print(F("HB succeeded "));Serial.println(gCurrentDeviceId, DEC);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    gCurrentDeviceId++;
    //unsigned long lastHBActivityTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    return;
  }
  Serial.print(F("HB status "));Serial.println(currentDeviceStatus.hbStatus, DEC);
}

void processSerialCommands(){
  if (Serial.available() == 0) return;
  int inComingCommand = Serial.read();
  if (inComingCommand == -1) return;
  //Serial.print(F("command received "));Serial.println(inComingCommand, DEC);
  switch(inComingCommand){
    case 49: // 1 in ascii
      Serial.println(F("Send signal"));
      processSerialSendRequest();
      break;
    case 50: // 2 in ascii
      Serial.println(F("Receive signal"));
      processSerialReceiveRequest();
      break;
    default:
      Serial.print(F("Unknown serial signal ")); Serial.print(inComingCommand,DEC);Serial.print(F("-"));Serial.println(inComingCommand,HEX);
      identifySelf();
      break;
  }
}

void processSerialSendRequest(){
  sendHBRequest(gCurrentDeviceId);
  gCurrentDeviceId++;
  if(gCurrentDeviceId > MAX_NUMBER_OF_DEVICES) gCurrentDeviceId=1;
}

void processSerialReceiveRequest(){
  processIRSignals();
}

void processIRSignals(){
  JIRCode jIrCode;
  boolean isValid = receiveValidJIRSignal(jIrCode);
  if(isValid){
    Serial.print(F("valid Signal received "));Serial.println(jIrCode.value, HEX);
    processJIRSignals(jIrCode);
  }
}

#define HB_REQUEST_CODE_PARAMS 0x00
void sendHBRequest(byte currentDeviceId){
  unsigned long hbRequestCode = formatCodeForSendingRequest(currentDeviceId,REQUEST_HB,gHBsignalNumber);
  mySender.send(NEC,hbRequestCode,32);
  Serial.print(F("HB Request Signal sent "));Serial.println(hbRequestCode, HEX);
  //delay(1000);
}

#define HB_RESPONSE_CODE_PARAMS 0x00
void sendHBResponse(JIRCode& jIrCode){
  unsigned long hbResponseCode = formatCodeForSendingResponse(jIrCode.sourceAddress,RESPONSE_HB,jIrCode.parameterData);
  mySender.send(NEC,hbResponseCode,32);
  Serial.print(F("HB Response Signal sent "));Serial.println(hbResponseCode, HEX);
}

void processHBResponse(JIRCode& jIrCode){
  Serial.print(F("HB Response Signal received "));Serial.println(jIrCode.sourceAddress, DEC);
  if(jIrCode.sourceAddress>MAX_NUMBER_OF_DEVICES) return;
  DeviceHBStatus& currentDeviceStatus = hbStatusOfDevices[jIrCode.sourceAddress];
  if(currentDeviceStatus.hbStatus == HB_RESPONSE_WAITING){
    currentDeviceStatus.hbStatus = HB_SUCCESS;
    currentDeviceStatus.lastEventTime = millis();
  } else {
     Serial.print(F("HB Status is not correct "));Serial.println(currentDeviceStatus.hbStatus, DEC);
  }
}

void processJIRSignals(JIRCode& jIrCode){
  //Serial.println(F("processJIR Signals "));
  jIrCode = convertToJIRCodeFromRawDecodedValue(jIrCode);
  if(jIrCode.protocolNum != NEC || jIrCode.destinationAddress != DEVICE_IDENTITY){
    processJIRSignalsAddressedToOthers(jIrCode);
    return;
  }
  // Below assumes the IR signal is addressed to self
  switch(jIrCode.command){
    case REQUEST_HB:
      Serial.print(F("HB request from ")); Serial.println(jIrCode.sourceAddress);
      sendHBResponse(jIrCode);
      break;        
    case RESPONSE_HB:
      Serial.print(F("HB response from ")); Serial.println(jIrCode.sourceAddress);
      processHBResponse(jIrCode);
      break;
    default:
      Serial.print(F("unknown signal ")); Serial.println(jIrCode.value,HEX);
      //testEchoOfPreviousTransmittedCode(jIrCode.value);
      break;
  }
}

void processJIRSignalsAddressedToOthers(JIRCode& jIrCode){
    //Serial.print(F("processJIRSignalsAddressedToOthers Addressed to ")); Serial.println(jIrCode.destinationAddress);
}

#define FORCED_DELAY_TO_DEBOUNCE_SIGNALS 100 // milliSeconds
unsigned long numOfChecksForIRSignal = 0;
unsigned long numOfRawIRSignalsReceived = 0;
unsigned long numOfRawValidIRSignalsReceived = 0;
unsigned long numOfValidJIRSignalsReceived = 0;
boolean receiveValidJIRSignal(JIRCode& jIRCode){
  boolean retVal = false;
  if (myReceiver.getResults()) {
    numOfRawIRSignalsReceived++;
    //Serial.print(F("numOfRawIRSignalsReceived ")); Serial.println(numOfRawIRSignalsReceived, DEC);
    if(myDecoder.decode()) {
      //myDecoder.dumpResults(false);  //Now print results. Use true for more detail
      numOfRawValidIRSignalsReceived++;
      //Serial.print(F("numOfRawValidIRSignalsReceived ")); Serial.println(numOfRawValidIRSignalsReceived, DEC);
      if(myDecoder.protocolNum == NEC || myDecoder.protocolNum == RC5){
        numOfValidJIRSignalsReceived++;
        jIRCode.protocolNum = myDecoder.protocolNum;
        jIRCode.value = myDecoder.value; // assign the value to the input reference only when a valid JIR signal
        //Serial.print(F("numOfValidJIRSignalsReceived ")); Serial.println(numOfValidJIRSignalsReceived, DEC);
        retVal=true;
      }
    } else Serial.println(F("Decode failed"));
    delay(FORCED_DELAY_TO_DEBOUNCE_SIGNALS);
    // For RC5 the signal is detected twice, donot know why.
    myReceiver.enableIRIn();      //Immediately Restart receiver, the debug print is long afterwards
  }
  return retVal;
}

// this is used to blink the identity led
void changePinValue(int pinNumber){
  digitalWrite(pinNumber, LOW);
  delay(50);
  digitalWrite(pinNumber, HIGH);
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

JIRCode& initializeJIRCode(JIRCode& jIrCode, unsigned int protocolNum, unsigned long value, byte destinationAddress, byte sourceAddress, byte command, byte parameterData){
  jIrCode.protocolNum = NEC;
  jIrCode.value = value;
  jIrCode.destinationAddress = destinationAddress;
  jIrCode.sourceAddress = sourceAddress;
  jIrCode.command = command; 
  jIrCode.parameterData = parameterData;
  //printSerialJIRCode(jIrCode);
  return jIrCode;
}

/* the method just initializes the other part of the JIRCode struct
 *  parameters:
 *  jIrCode the object which has to be initialized
 *          assumes is already set with the protocolNum and value
 */
JIRCode& convertToJIRCodeFromRawDecodedValue(JIRCode& jIrCode){
  unsigned int shiftedDecodedValue = jIrCode.value>>16;
  jIrCode.destinationAddress = highByte(shiftedDecodedValue);
  jIrCode.sourceAddress = lowByte(shiftedDecodedValue);
  jIrCode.command = highByte(jIrCode.value);
  jIrCode.parameterData = lowByte(jIrCode.value);
  //printSerialJIRCode(jIrCode);
  return jIrCode;
}

void printSerialJIRCode(JIRCode& jIrCode){
  Serial.print(jIrCode.protocolNum,DEC);Serial.print(F("-"));Serial.print(jIrCode.value,HEX);Serial.print(F("-"));
  Serial.print(jIrCode.destinationAddress,HEX);Serial.print(F("-"));Serial.print(jIrCode.sourceAddress,HEX);Serial.print(F("-"));
  Serial.print(jIrCode.command,HEX);Serial.print(F("-"));Serial.println(jIrCode.parameterData,HEX);
}

unsigned long findElapsedTimeInMillis(unsigned long& lastEventTime){
  unsigned long retVal;
  unsigned long currentTime = millis();
  retVal = currentTime - lastEventTime;
  if(retVal < 0 ) {
    retVal = (0xFFFFFFFE - lastEventTime) + currentTime;
  }
  return retVal;
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
  if(MASTER_MODE) Serial.print(F("MASTER")); else Serial.print(F("SLAVE"));
  Serial.print(" 0x");Serial.print(DEVICE_IDENTITY,HEX); Serial.print(" Color ");
  if (DEVICE_IDENTITY & 0x0008) Serial.print("R"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0004) Serial.print("G"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0002) Serial.print("B"); else Serial.print("-");
  if (DEVICE_IDENTITY & 0x0001) Serial.println("Y"); else Serial.println("-");  
}

/* Important points to note
 *  
 *  1. Always use the F macro in Serial.print - https://www.baldengineer.com/arduino-f-macro.html
 *  2. IRLib is generating bouncing values when the delay is less between getResults and enableIRIn
 *      for RC5 it generate same value twice and for NEC it is generating a repeat sequence FFFFFFFF
 *  3. Packet collisions are not taken care. This is really a Master-Slave mode operation, only when the Master asks for, the slave responds.
 *     Master - Slave role determination is through the hardware settings in each device.
 *     Never set the hardware to have more than one Master
 *  
 */
