// jagi has changed this script

#include <RingBufCPP.h>
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

typedef struct {
  byte protocolNum;
  unsigned long timestamp;
  unsigned long value;
} NecIrSignal;
#define MAX_NUM_NECSIGNALS_BUF 10
//NECSIGNAL gIRSignalsNEC[MAX_NUM_NECSIGNALS];
// Stack allocate the buffer to hold IR signals structs
RingBufCPP<NecIrSignal, MAX_NUM_NECSIGNALS_BUF> buf;

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

boolean fullDetail=false;
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
#define MAX_NUMBER_OF_DEVICES 8
DeviceHBStatus hbStatusOfDevices[MAX_NUMBER_OF_DEVICES+1];
byte gCurrentDeviceId = 1; // indexing can all go wrong keep note
unsigned long gHBsignalNumber = 0;
void deviceMonitoring(){
  if(gCurrentDeviceId>MAX_NUMBER_OF_DEVICES) {
    gCurrentDeviceId = 1;
    Serial.println(F("Restarting from beginning"));
  }
  monitorDevice(gCurrentDeviceId);
  Serial.print(F("Device Status ")); Serial.print(gCurrentDeviceId); Serial.print(F(" : "));Serial.println(hbStatusOfDevices[gCurrentDeviceId].hbStatus); 
  if(hbStatusOfDevices[gCurrentDeviceId].hbStatus == HB_SUCCESS ||
  hbStatusOfDevices[gCurrentDeviceId].hbStatus == HB_FAILED){
    gCurrentDeviceId++;
  }
}

void monitorDevice(byte currentDeviceId){
  DeviceHBStatus& currentDeviceStatus = hbStatusOfDevices[currentDeviceId];
  if(currentDeviceStatus.hbStatus == HB_NOT_SENT){
    gHBsignalNumber++;
    sendHBRequest(currentDeviceId);
    currentDeviceStatus.hbStatus = HB_RESPONSE_WAITING;
    currentDeviceStatus.lastEventTime = millis();
    Serial.print(F("HB request sent "));Serial.println(currentDeviceId, DEC);
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_RESPONSE_WAITING){
    // someone else is expected to change this status
    // how long are we waiting?
    unsigned long waitingTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    if(waitingTime > HB_TIMEOUT_PERIOD){
      // donot put any delay here, the main loop() may need that time to do some work
      currentDeviceStatus.hbStatus = HB_FAILED;
      Serial.print(F("HB timedout "));Serial.println(currentDeviceId, DEC);
      goto monitorDeviceWorkFinish;//return;
    }
    processIRSignals();
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_FAILED){
    Serial.print(F("HB fail "));Serial.println(currentDeviceId, DEC);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_SUCCESS){
    Serial.print(F("HB success "));Serial.println(currentDeviceId, DEC);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    //unsigned long lastHBActivityTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    goto monitorDeviceWorkFinish;//return;
  }
monitorDeviceWorkFinish:
  Serial.print(F("Monitor HB status "));Serial.print(currentDeviceId, DEC); Serial.print(F(" : "));Serial.println(currentDeviceStatus.hbStatus, DEC);
}

void processSerialCommands(){
  if (Serial.available() == 0) return;
  int inComingCommand = Serial.read();
  if (inComingCommand == -1) return;
  //Serial.print(F("command received "));Serial.println(inComingCommand, DEC);
  switch(inComingCommand){
    case 49: // 1 in ascii
      processSerial1();
      break;
    case 50: // 2 in ascii
      processSerial2();
      break;
    case 51: // 3 in ascii
      break;
    case 56: // 8 in ascii
      processSerial8();
      break;
    case 57: // 9 in ascii
      processSerial9();
      break;
    case 68: // D in ascii
    case 100: // d in ascii
      deviceMonitoring();
      break;
    case 69: // E in ascii
    case 101: // e in ascii
      processSerialE();
      break;
    case 70: // F in ascii
    case 102: // f in ascii
      processSerialF();
      break;
    default:
      Serial.print(F("Unknown serial signal ")); Serial.print(inComingCommand,DEC);Serial.print(F("-"));Serial.println(inComingCommand,HEX);
      processSerialX();
      break;
  }
}

void processSerial1(){
  Serial.println(F("Send signal"));
  sendHBRequest(gCurrentDeviceId);
  gCurrentDeviceId++;
  if(gCurrentDeviceId > MAX_NUMBER_OF_DEVICES) gCurrentDeviceId=1;
}

void processSerial2(){
  Serial.println(F("Receive signal"));
  processIRSignals();
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
  Serial.println(F("Change dump level"));
  fullDetail = (!fullDetail);
}

void processSerialX(){
  Serial.println(F("Print Stats"));
  printIrSignalStats();
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
  //mySender.send(NEC,hbRequestCode,32);
  sendIRSignals(hbRequestCode);
  Serial.print(F("HB Request Signal sent "));Serial.println(hbRequestCode, HEX);
  //delay(1000);
}

#define HB_RESPONSE_CODE_PARAMS 0x00
void sendHBResponse(JIRCode& jIrCode){
  unsigned long hbResponseCode = formatCodeForSendingResponse(jIrCode.sourceAddress,RESPONSE_HB,jIrCode.parameterData);
  //mySender.send(NEC,hbResponseCode,32);
  sendIRSignals(hbResponseCode);
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
  Serial.println(F("processJIR Signals "));
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
  receiveIRSignals();
  if(buf.isEmpty()) return retVal;
  NecIrSignal necIrSignal;
  if(!buf.pull(&necIrSignal)) return retVal;
  jIRCode.protocolNum = necIrSignal.protocolNum;
  jIRCode.value = necIrSignal.value;
  retVal = true;
  return retVal;
}

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
  Serial.print(F("Send raw Ir Signal "));Serial.println(iRcode, HEX);
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
      Serial.print(F(" Tx attempt ")); Serial.print(i);Serial.print(F(" : "));
      Serial.print(sendTimeStamp); Serial.print(F(" - "));Serial.print(sendTimeStampBefore); Serial.print(F(" = "));Serial.println(timeTakenToSend,DEC);
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
        Serial.print(millis()); Serial.println(F(" Echo Timed Out"));
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
    Serial.print(F("Raw Send Signal complete "));Serial.println(iRcode, HEX);
  }
}

unsigned long rawIRSignalsReceived = 0;
unsigned long rawIRSignalsDecodeFailures = 0;
unsigned long rawIRSignalsDuplicates = 0;
unsigned long rawIRSignalsDropped = 0;
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
        Serial.print(F("Q failed ")); Serial.println(necIrSignal.value, HEX);
        rawIRSignalsDropped++;
      }
    } else { // This is an indication of possible collision or a simple hardware problem, hence re-transmit
      Serial.println(F("Decode failed"));
      rawIRSignalsDecodeFailures++;
      if(isWaitingForEcho){
        rawIRSignalsEchoFailed++;// = 0;
        isReTransmissionRequired = true;
        Serial.print(F("Raw Echo failed for ")); Serial.println(recentTransmitted, HEX);
        goto enableReceiver;
      }
    }
enableReceiver:
    myReceiver.enableIRIn();      //Restart receiver
  }
}

// Print the buffer's contents then empty it
void print_buf_contents()
{
  NecIrSignal * e = 0;
  Serial.println("\n______Peek contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  for (int i = 0 ; i<buf.numElements(); i++) {
    e = buf.peek(i);
    if(!e) break;
    Serial.print(F("t ")); Serial.print(e->timestamp);
    Serial.print(F(" v ")); Serial.println(e->value, HEX);
  }
  Serial.println("______Done peeking contents_______");
}

// Print the buffer's contents then empty it
void dump_buf_contents()
{
  NecIrSignal e;
  Serial.println("\n______Dumping contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  while (buf.pull(&e))
  {
    Serial.print(F("t ")); Serial.print(e.timestamp);
    Serial.print(F(" v ")); Serial.println(e.value, HEX);
  }
  Serial.println("______Done dumping contents_______");
}

void printIrSignalStats(){
  Serial.println(F("RawTx\tEchoFail\tReTransmitted"));
  Serial.print(rawIRSignalsSent);Serial.print(F("\t"));Serial.print(rawIRSignalsEchoFailed);Serial.print(F("\t"));Serial.println(rawIRSignalsReTransmitted);
  Serial.println(F("RawRx\tDecodeFail Duplicate Dropped"));
  Serial.print(rawIRSignalsReceived,DEC);Serial.print(F("\t"));Serial.print(rawIRSignalsDecodeFailures,DEC);Serial.print(F("\t"));
  Serial.print(rawIRSignalsDuplicates,DEC);Serial.print(F("\t"));Serial.println(rawIRSignalsDropped,DEC);
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
  //Serial.print(F("timeRemaining "));Serial.println(timeRemaining);
  if (timeRemaining > 0 ) return false;
  return true;
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
