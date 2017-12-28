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
#include <IRLib_P08_Samsung36.h>
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
  unsigned int protocolNum;
  unsigned long value;
  // four bytes received from the 32 bit value from IR
  byte destinationAddress = 0;
  byte sourceAddress = 0;
  byte command = 0;
  byte parameterData = 0;
} JIRCode;

unsigned int DEVICE_IDENTITY=0;
boolean MASTER_MODE=false;
#define pinMasterMode   7

#define pinLedRed      11
#define pinLedAnode    10
#define pinLedGreen     9
#define pinLedBlue      8

void setup() {
  Serial.begin(9600);
  delay(2000); while (!Serial); //delay for Leonardo
  //to identify the MASTER_MODE or SLAVE_MODE, configure pin as an input and enable the internal pull-up resistor
  pinMode(pinMasterMode, INPUT_PULLUP);
  //to identify the DEVICE configure pins as input and enable the internal pull-up resistor
  pinMode(pinLedRed, INPUT_PULLUP);
  pinMode(pinLedGreen, INPUT_PULLUP);
  pinMode(pinLedBlue, INPUT_PULLUP);
  //configure pin as power for led, Let this be high initially supplying power to the LED
  pinMode(pinLedAnode, OUTPUT);
  digitalWrite(pinLedAnode, HIGH);
  // identify the self device id and the mode of operation
  identifySelf();
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
    //sendSignal();
    monitorDevicesWithHB();
    generateHBResponses();
  }
  receiveSignal();
}

unsigned long previousTransmittedCode = 0;
JIRCode jpreviousTransmittedCode;

#define MAX_NUMBER_OF_DEVICES 4
#define HB_REQUEST_CODE_PARAMS 0x00
void monitorDevicesWithHB(){
  unsigned long codeToSend = 0;
  for(int i=1; i<=MAX_NUMBER_OF_DEVICES; i++){
    codeToSend = formatCodeForSendingRequest(i,REQUEST_HB,HB_REQUEST_CODE_PARAMS);
    Serial.print(i, DEC);Serial.print(" HB ");Serial.println(codeToSend, HEX);
    mySender.send(NEC,codeToSend,32);
    previousTransmittedCode = codeToSend;
    jpreviousTransmittedCode = initializeJIRCode(jpreviousTransmittedCode, NEC,codeToSend,i,DEVICE_IDENTITY,REQUEST_HB,HB_REQUEST_CODE_PARAMS);
    // wait for HB response
    if(receiveHBResponse(jpreviousTransmittedCode)){
      Serial.print("DEVICE "); Serial.print(i, DEC); Serial.println(" HB success");
    }else{
      Serial.print("DEVICE "); Serial.print(i, DEC); Serial.println(" HB failed"); 
    }
    myReceiver.enableIRIn(); // Start the receiver
    Serial.println("next HB Ready to receive IR signals");
  }
}

unsigned long numOfHBResponsesReceived = 0;
#define HB_RESPONSE_TIMEOUT 1000 // in milliSeconds
boolean receiveHBResponse(JIRCode& jPreviousTransmittedCode){
  JIRCode jIRCode;
  if(waitAndReceiveSignalAddressedToSelf(jIRCode, HB_RESPONSE_TIMEOUT)){
    if(jIRCode.command == REQUEST_HB && jIRCode.sourceAddress == jPreviousTransmittedCode.destinationAddress){
      numOfHBResponsesReceived++;
      return true;
    }
  }
  return false;
}

unsigned long numOfSignalsReceived = 0;

unsigned long numOfSignalsTimedOutReceivedToSelf = 0;
boolean waitAndReceiveSignalAddressedToSelf(JIRCode& jIRCode, unsigned long millisToWait){
  if(waitAndReceiveSignalAddressedToDestination(jIRCode, DEVICE_IDENTITY, millisToWait)){
    return true;
  }
  numOfSignalsTimedOutReceivedToSelf++;
  return false;
}

unsigned long numOfSignalsTimedOutReceivedToDestination = 0;
#define DELAY_BETWEEN_SIGNAL_POLLING 100 // in milliSeconds
boolean waitAndReceiveSignalAddressedToDestination(JIRCode& jIRCode, byte destinationAddress, unsigned long millisToWait){
  unsigned long timeElapsed = 0;
  while ( timeElapsed < millisToWait){
    if(receiveSignalAddressedToTarget(jIRCode, destinationAddress)) {
      return true;
    }
    delay(DELAY_BETWEEN_SIGNAL_POLLING);
    timeElapsed += DELAY_BETWEEN_SIGNAL_POLLING;
    //Serial.print("Time remaining "); Serial.println(millisToWait-timeElapsed, DEC);
  }
  numOfSignalsTimedOutReceivedToDestination++;
  return false;
}

unsigned long numOfSignalsReceivedToSelf = 0;
boolean receiveSignalAddressedToSelf(JIRCode& jIRCode){
  if(receiveSignalAddressedToTarget(jIRCode, DEVICE_IDENTITY)){
    numOfSignalsReceivedToSelf++;
    return true;
  } else return false;
}

unsigned long numOfSignalsReceivedToDestination = 0;
#define ALL_ADDRESSES 0x7E
boolean receiveSignalAddressedToTarget(JIRCode& jIRCode, byte destinationAddress){
  boolean retVal = false;
  //Serial.print("Checking for raw signal "); Serial.println(numOfSignalsReceived, DEC);
  if (myReceiver.getResults()) {
    //Serial.print("numOfSignalsReceived is "); Serial.println(numOfSignalsReceived, DEC);
    if(myDecoder.decode()) {
      numOfSignalsReceived++;
      //myDecoder.dumpResults(true);  //Now print results. Use false for less detail
      jIRCode = decodeJIRFromIRDecoder(jIRCode, myDecoder);
      myReceiver.enableIRIn(); // Start the receiver
      Serial.println("Ready to receive IR signals");
      if(testEchoOfPreviousTransmittedCode(jIRCode.value)){
        Serial.print("Found echo signal "); Serial.println(jIRCode.value, HEX);
        return false;        
      }
      if(jIRCode.protocolNum == NEC && (destinationAddress == ALL_ADDRESSES || jIRCode.destinationAddress == destinationAddress)){
        numOfSignalsReceivedToDestination++;
        retVal = true;
        Serial.println("Found matching signal");
      } else {
        retVal = false;
        Serial.println("Found non-matching signal");
      }
    } else {
      Serial.println("Decode failed ");
    }
  }
  return retVal;
}

void respondToHB(){
  unsigned long codeToSend = 0;
  codeToSend = formatCodeForSendingResponse(i,REQUEST_HB,0x13);
  mySender.send(NEC,codeToSend,32);
}
void generateHBResponses(){
  unsigned long codeToSend = 0;
  Serial.println(codeToSend, HEX);
  for(int i=1; i<=MAX_NUMBER_OF_DEVICES; i++){
    codeToSend = formatCodeForSendingResponse(i,REQUEST_HB,0x13);
    Serial.print(i, DEC);Serial.print(" HB Response ");Serial.println(codeToSend, HEX);
  }
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

unsigned long currentIRCode = 0;
void receiveSignal() {
  currentIRCode = 0;
  if (myReceiver.getResults()) {
    numOfSignalsReceived++;
    Serial.print(F("numOfSignalsReceived is ")); Serial.println(numOfSignalsReceived, DEC);
    Serial.println(F("decoding signal"));
    if(myDecoder.decode()) {
      //myDecoder.dumpResults(true);  //Now print results. Use false for less detail
      currentIRCode = myDecoder.value;
      myReceiver.enableIRIn();      //Restart receiver
      Serial.println(F("enabled receiver after decoding"));
      Serial.print(F("Recvd signal "));Serial.println(currentIRCode,HEX);
      switch(currentIRCode) {
        case HEARTBEAT_REQUEST:
        case BUTTON_INFO:
        case BUTTON_INFO_REPEAT:
        case BUTTON_RED_CHNL_MINUS:
        case BUTTON_GREEN_PREV:
        case BUTTON_BLUE_MINUS:
          respondHeartBeatRequest();
          break;        
        case HEARTBEAT_RESPONSE_VU:
        case HEARTBEAT_RESPONSE_VU_REPEAT:
          processHeartBeatResponse();
          break;
        default:
          //Serial.println(F("unknown signal"));//Serial.println(decodedValue,HEX);
          testEchoOfPreviousTransmittedCode(currentIRCode);
          break;
      }
    } else {
      myReceiver.enableIRIn();      //Restart receiver
      Serial.println(F("enabled receiver decode failed"));
    }
  }
}


int echoCount = 0;
int nonEchoCount = 0;
boolean testEchoOfPreviousTransmittedCode(unsigned long decodedValue){
  if(decodedValue == previousTransmittedCode) {
    echoCount++;
    Serial.print("Ignore Echo "); Serial.print(echoCount, DEC); Serial.println();
    return true;
  } else {
    nonEchoCount++;
    Serial.print("Ignore unknown signal "); Serial.print(nonEchoCount, DEC); Serial.println(decodedValue, HEX);
  }
  return false;
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
  int sensorVal = HIGH;
  //read the master mode pin, print out the mode
  sensorVal = digitalRead(pinMasterMode);
  if(sensorVal == LOW) {MASTER_MODE = true; Serial.println(F("MASTER"));}
  else {MASTER_MODE = false; Serial.println(F("SLAVE"));}
  //read the pin values into a variable
  //print out the color of value
  sensorVal = digitalRead(pinLedRed);
  if (sensorVal == LOW) {
    DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0004;
    currentLedColor = pinLedRed;
    Serial.print("R");
  } else {
    DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFB;
    Serial.print("-");
  }
  sensorVal = digitalRead(pinLedGreen);
  if (sensorVal == LOW) {
    DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0002;
    currentLedColor = pinLedGreen;
    Serial.print("G");
  } else {
    DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFD;
    Serial.print("-");
  }
  sensorVal = digitalRead(pinLedBlue);
  if (sensorVal == LOW) {
    DEVICE_IDENTITY = DEVICE_IDENTITY | 0x0001;
    currentLedColor = pinLedBlue;
    Serial.print("B");
  } else {
    DEVICE_IDENTITY = DEVICE_IDENTITY & 0xFFFE;
    Serial.print("-");
  }
  Serial.println("");Serial.print(DEVICE_IDENTITY,BIN);Serial.println("");
}

int numOfHeartBeatRequests = 0;
int previousState = BUTTON_OK;
void respondHeartBeatRequest(){
  numOfHeartBeatRequests++;
  // the receivedCode can be checked to see if this the device to respond
  identifySelf();
  extractDataParts(currentIRCode);
  if( 
    (currentIRCode == BUTTON_RED_CHNL_MINUS && currentLedColor == pinLedRed) ||
    (currentIRCode == BUTTON_GREEN_PREV && currentLedColor == pinLedGreen) ||
    (currentIRCode == BUTTON_BLUE_MINUS && currentLedColor == pinLedBlue) ){
      if(previousState == BUTTON_OK) previousState = BUTTON_OK_REPEAT; else previousState = BUTTON_OK;
      Serial.print("HB requests ");Serial.print(numOfHeartBeatRequests, DEC);Serial.print(" response send ");Serial.println(previousState, HEX);
      mySender.send(RC5, previousState, 0);
      // simply blink the self device's identification LED
      changePinValue(pinLedAnode);
    } else {
      Serial.print("HB request Not Self ");Serial.println(numOfHeartBeatRequests, DEC);
    }
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
  Serial.println(retVal, HEX);
  return retVal;
}

// four bytes received from the 32 bit value from IR
byte gDestinationAddress = 0;
byte gSourceAddress = 0;
byte gCommand = 0;
byte gParameterData = 0;
void extractDataParts(unsigned long decodedValue){
  unsigned int shiftedValue = decodedValue>>16;
  gDestinationAddress = highByte(shiftedValue);
  gSourceAddress = lowByte(shiftedValue);
  gCommand = highByte(decodedValue);
  gParameterData = lowByte(decodedValue);
  Serial.print(gDestinationAddress,HEX);Serial.print("-");
  Serial.print(gSourceAddress,HEX);Serial.print("-");
  Serial.print(gCommand,HEX);Serial.print("-");
  Serial.println(gParameterData,HEX);
}

JIRCode& initializeJIRCode(JIRCode& jIrCode, unsigned int protocolNum, unsigned long value, byte destinationAddress, byte sourceAddress, byte command, byte parameterData){
  jIrCode.protocolNum = NEC;
  jIrCode.value = value;
  jIrCode.destinationAddress = destinationAddress;
  jIrCode.sourceAddress = sourceAddress;
  jIrCode.command = command; 
  jIrCode.parameterData = parameterData;
  printSerialJIRCode(jIrCode);
  return jIrCode;
}

// assumes the decoder.decode() is already executed
JIRCode& decodeJIRFromIRDecoder(JIRCode& jIrCode, IRdecode& decoder){
  jIrCode.protocolNum = decoder.protocolNum;
  jIrCode.value = decoder.value;
  unsigned int shiftedDecodedValue = jIrCode.value>>16;
  jIrCode.destinationAddress = highByte(shiftedDecodedValue);
  jIrCode.sourceAddress = lowByte(shiftedDecodedValue);
  jIrCode.command = highByte(jIrCode.value);
  jIrCode.parameterData = lowByte(jIrCode.value);
  printSerialJIRCode(jIrCode);
  return jIrCode;
}

void printSerialJIRCode(JIRCode& jIrCode){
  Serial.print(jIrCode.protocolNum,DEC);Serial.print("-");Serial.print(jIrCode.value,HEX);Serial.print("-");
  Serial.print(jIrCode.destinationAddress,HEX);Serial.print("-");Serial.print(jIrCode.sourceAddress,HEX);Serial.print("-");
  Serial.print(jIrCode.command,HEX);Serial.print("-");Serial.println(jIrCode.parameterData,HEX);
}

