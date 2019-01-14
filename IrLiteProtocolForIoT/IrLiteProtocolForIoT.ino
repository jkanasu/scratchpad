// Some times we cannot use the default Serial for debug/logs, hence we use SoftwareSerial
// You could use a spare Hardware Serial on boards that have it (like Mega)
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(10, 11); // RX, TX

#define JAGI_PRINT DebugSerial // You can change this to use a Software Serial
#define JAGI_LOG_TIME {JAGI_PRINT.print(F("[")); JAGI_PRINT.print(millis());JAGI_PRINT.print(F("] "));}
#define JAGI_LOG1(pLast) {JAGI_LOG_TIME; JAGI_PRINT.println(pLast);}
#define JAGI_LOG2(p1,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.println(pLast);}
#define JAGI_LOG3(p1,p2,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.print(p2); JAGI_PRINT.println(pLast);}
#define JAGI_LOG4(p1,p2,p3,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.print(p2); JAGI_PRINT.print(p3); JAGI_PRINT.println(pLast);}
#define JAGI_LOG6(p1,p2,p3,p4,p5,pLast) {JAGI_LOG_TIME; JAGI_PRINT.print(p1); JAGI_PRINT.print(p2); JAGI_PRINT.print(p3); JAGI_PRINT.print(p4); JAGI_PRINT.print(p5); JAGI_PRINT.println(pLast);}

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

unsigned int DEVICE_IDENTITY=0;
boolean MASTER_MODE=false;

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT DebugSerial

#include <BlynkSimpleStream.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "63555079ad804657bd66629a162fa6ba";

void setup() {
  // put your setup code here, to run once:
  // Debug console
  DebugSerial.begin(57600); // NOTE baud rate 9600

  Serial.begin(57600);
  delay(2000); while (!Serial); //delay for Leonardo
  while (!DebugSerial);
  JAGI_LOG1(F("IrLiteStateMachine.ino"));
  // identify the self device id and the mode of operation
  identifySelf();
  JAGI_LOG1(F("IrLiteStateMachine.ino : Every time you press a key in serial monitor; monitoring using HB will be done."));
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  initializeDeviceInformation();
  initializeIrLiteFSM();
  Blynk.begin(Serial, auth);
  JAGI_LOG1(F("Ready to receive IR signals"));
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  runIrLiteFSM();
  receiveIRSignals();
}

struct necIrSignal{
  byte protocolNum;
  unsigned long timestamp;
  unsigned long value;
};
typedef struct necIrSignal NecIrSignal;
#define MAX_NUM_NECSIGNALS_BUF 10
//NECSIGNAL gIRSignalsNEC[MAX_NUM_NECSIGNALS];
// Stack allocate the buffer to hold IR signals structs
#include <RingBufCPP.h>
RingBufCPP<NecIrSignal, MAX_NUM_NECSIGNALS_BUF> buf;
RingBufCPP<NecIrSignal, MAX_NUM_NECSIGNALS_BUF> bufNotRelated;

struct extendedNecIrSignal{
  byte destination;
  byte source;
  byte command;
  byte params;
};
typedef struct extendedNecIrSignal ExtendedNecIrSignal;

#define INFORMATION_STATE_INVALID 0
#define INFORMATION_STATE_VALID 1
#define INFORMATION_VALIDITY_TIMEOUT 10000
struct deviceInformation{
  byte id;
  byte informationType;
  byte informationParameters;
  unsigned long sendTimestamp = 0x0;
  unsigned long updateTimestamp = 0x0;
  byte informationState = INFORMATION_STATE_INVALID;
  byte informationValue = 0x0;
};
typedef struct deviceInformation DeviceInformation;

#define INFORMATION_UPTIME 0x01
#define INFORMATION_PINSTATUS 0x02
#define INFORMATION_ANALOGREAD 0x03
#define INFORMATION_CHANGEPINSTATE 0x04

#define PARAMETERS_UPTIME 0x00
#define PARAMETERS_PINSTATUS 0x00
#define PARAMETERS_ANALOGREAD A4
#define PARAMETERS_CHANGEPINSTATE LED_BUILTIN

#define MAX_NUM_OF_DEVICES 4
#define NUM_OF_COMMANDS_PER_DEVICE 4
DeviceInformation deviceInformation[MAX_NUM_OF_DEVICES * NUM_OF_COMMANDS_PER_DEVICE];

boolean initializeDeviceInformation(){
  for(int i=0; i<MAX_NUM_OF_DEVICES; i++){
    deviceInformation[i*4+0].id = i+1;
    deviceInformation[i*4+0].informationType = INFORMATION_UPTIME;
    deviceInformation[i*4+0].informationParameters = PARAMETERS_UPTIME;

    deviceInformation[i*4+1].id = i+1;
    deviceInformation[i*4+1].informationType = INFORMATION_PINSTATUS;
    deviceInformation[i*4+1].informationParameters = PARAMETERS_PINSTATUS;

    deviceInformation[i*4+2].id = i+1;
    deviceInformation[i*4+2].informationType = INFORMATION_ANALOGREAD;
    deviceInformation[i*4+2].informationParameters = PARAMETERS_ANALOGREAD;

    deviceInformation[i*4+3].id = i+1;
    deviceInformation[i*4+3].informationType = INFORMATION_CHANGEPINSTATE;
    deviceInformation[i*4+3].informationParameters = PARAMETERS_CHANGEPINSTATE;
  }
}

#define STATE_UNDEFINED 0
#define EVENT_UNDEFINED 0

#define STATE_LOOP_NEXT_ITEM 25
#define EVENT_SELECTED_NEXT_ITEM 26
#define EVENT_WAITING_FOR_DELAY 27

#define STATE_READY_TO_SEND_COMMAND 50
#define EVENT_COMMAND_SENT 51

#define STATE_WAIT_FOR_ECHO 75
#define EVENT_WAITING_FOR_ECHO 76
#define EVENT_RECEIVED_ECHO 77
#define EVENT_TIMEDOUT_ECHO 78

#define STATE_WAIT_FOR_RESPONSE 100
#define EVENT_WAITING_FOR_RESPONSE 101
#define EVENT_RECEIVED_RESPONSE 102
#define EVENT_TIMEDOUT_RESPONSE 103

#define STATE_READY_TO_PROCESS_RESPONSE 125
#define EVENT_PROCESSING_RESPONSE_COMPLETE 127

#define STATE_READY_TO_RETRY_COMMAND 150
#define EVENT_RETRIES_AVAILABLE 151
#define EVENT_RETRIES_EXHAUSTED 152

#define STATE_COMMAND_SUCCEEDED 175
#define EVENT_PROCESSING_COMMAND_SUCCESS_COMPLETE 176
#define STATE_COMMAND_FAILED 200
#define EVENT_PROCESSING_COMMAND_FAILURE_COMPLETE 201

byte currentItem = 0;
byte currentState = STATE_UNDEFINED;
// NEC is used for our system
// COMMAND & RESPONSE differ in only the MSB value
boolean initializeIrLiteFSM(){
  currentState = STATE_LOOP_NEXT_ITEM;
  currentItem = -1;
}

boolean runIrLiteFSM(){
  byte currentEvent = 0;
  switch (currentState){
    case STATE_UNDEFINED:
      JAGI_LOG3("STATE_UNDEFINED", "-->", "STATE_UNDEFINED");
      break;
    case STATE_LOOP_NEXT_ITEM:
      currentEvent = selectNextItem();
      switch(currentEvent){
        case EVENT_WAITING_FOR_DELAY:
          //JAGI_LOG3("STATE_LOOP_NEXT_ITEM", "-->", "STATE_LOOP_NEXT_ITEM");
          break;
        case EVENT_SELECTED_NEXT_ITEM:
          currentState = STATE_READY_TO_SEND_COMMAND;
          JAGI_LOG3("STATE_LOOP_NEXT_ITEM", "-->", "STATE_READY_TO_SEND_COMMAND");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_READY_TO_SEND_COMMAND:
      currentEvent = sendCommand();
      switch(currentEvent){
        case EVENT_COMMAND_SENT:
          currentState = STATE_WAIT_FOR_ECHO;
          JAGI_LOG3("STATE_READY_TO_SEND_COMMAND", "-->", "STATE_WAIT_FOR_ECHO");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_WAIT_FOR_ECHO:
      currentEvent = waitForEcho();
      switch(currentEvent){
        case EVENT_WAITING_FOR_ECHO:
          //JAGI_LOG3("STATE_WAIT_FOR_ECHO", "-->", "STATE_WAIT_FOR_ECHO");
          break;
        case EVENT_RECEIVED_ECHO:
        case EVENT_TIMEDOUT_ECHO:
          currentState = STATE_WAIT_FOR_RESPONSE;
          JAGI_LOG3("STATE_WAIT_FOR_ECHO", "-->", "STATE_WAIT_FOR_RESPONSE");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_WAIT_FOR_RESPONSE:
      currentEvent = waitForResponse();
      switch(currentEvent){
        case EVENT_WAITING_FOR_RESPONSE:
          //JAGI_LOG3("STATE_WAIT_FOR_RESPONSE", "-->", "STATE_WAIT_FOR_RESPONSE");
          break;
        case EVENT_RECEIVED_RESPONSE:
          currentState = STATE_READY_TO_PROCESS_RESPONSE;
          JAGI_LOG3("STATE_WAIT_FOR_RESPONSE", "-->", "STATE_READY_TO_PROCESS_RESPONSE");
          break;
        case EVENT_TIMEDOUT_RESPONSE:
          currentState = STATE_READY_TO_RETRY_COMMAND;
          JAGI_LOG3("STATE_WAIT_FOR_RESPONSE", "-->", "STATE_READY_TO_RETRY_COMMAND");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_READY_TO_PROCESS_RESPONSE:
      currentEvent = processResponse();
      switch(currentEvent){
        case EVENT_PROCESSING_RESPONSE_COMPLETE:
          currentState = STATE_COMMAND_SUCCEEDED;
          JAGI_LOG3("STATE_READY_TO_PROCESS_RESPONSE", "-->", "STATE_COMMAND_SUCCEEDED");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_READY_TO_RETRY_COMMAND:
      currentEvent = retryCommand();
      switch(currentEvent){
        case EVENT_RETRIES_EXHAUSTED:
          currentState = STATE_COMMAND_FAILED;
          JAGI_LOG3("STATE_READY_TO_RETRY_COMMAND", "-->", "STATE_COMMAND_FAILED");
          break;
        case EVENT_RETRIES_AVAILABLE:
          currentState = STATE_READY_TO_SEND_COMMAND;
          JAGI_LOG3("STATE_READY_TO_RETRY_COMMAND", "-->", "STATE_READY_TO_SEND_COMMAND");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_COMMAND_SUCCEEDED:
      currentEvent = postProcessCommandSuccess();
      switch(currentEvent){
        case EVENT_PROCESSING_COMMAND_SUCCESS_COMPLETE:
          currentState = STATE_LOOP_NEXT_ITEM;
          JAGI_LOG3("STATE_COMMAND_SUCCEEDED", "-->", "STATE_LOOP_NEXT_ITEM");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    case STATE_COMMAND_FAILED:
      currentEvent = postProcessCommandFailure();
      switch(currentEvent){
        case EVENT_PROCESSING_COMMAND_FAILURE_COMPLETE:
          currentState = STATE_LOOP_NEXT_ITEM;
          JAGI_LOG3("STATE_COMMAND_FAILED", "-->", "STATE_LOOP_NEXT_ITEM");
          break;
        case EVENT_UNDEFINED:
          break;
        default:
          break;
      }
      break;
    default:
      JAGI_LOG2("STATE_UNKNOWN ", currentState);
      break;
  }
}

#define MAX_NUM_OF_ITEMS MAX_NUM_OF_DEVICES*NUM_OF_COMMANDS_PER_DEVICE
#define DELAY_BEFORE_SELECT_NEXT_ITEM 10000
#define DELAY_BEFORE_SELECT_NEXT_LOOP 60000
byte selectNextItem(){
  if(currentItem == 0xFF) {
    currentItem = 0;
    return EVENT_SELECTED_NEXT_ITEM;
  }
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  //JAGI_LOG4(F("Current Item "), currentItem, F(" - "), currentDeviceInformation.sendTimestamp);
  if(currentItem+1 >= MAX_NUM_OF_ITEMS){ // end of loop give more delay
    if (!hasTimedOut(currentDeviceInformation.sendTimestamp,DELAY_BEFORE_SELECT_NEXT_LOOP)) return EVENT_WAITING_FOR_DELAY;
    JAGI_LOG1(F("END of loop"));
  } else {
    if (!hasTimedOut(currentDeviceInformation.sendTimestamp,DELAY_BEFORE_SELECT_NEXT_ITEM)) return EVENT_WAITING_FOR_DELAY;
    //JAGI_LOG1(F("Just next item"));
  }
  currentItem++;
  if (currentItem >= MAX_NUM_OF_ITEMS) currentItem=0;
  //delay(DELAY_BEFORE_SELECT_NEXT_ITEM); /// NEVER DO THIS
  //JAGI_LOG4(F("Current Item "), currentItem, F(" - "), currentDeviceInformation.sendTimestamp);
  //print_buf_contents(buf);
  //print_buf_contents(bufNotRelated);
  //printIrSignalStats();
  return EVENT_SELECTED_NEXT_ITEM;
}

byte sendCommand(){
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  unsigned long irCode = formatCodeForSendingRequest(currentDeviceInformation.id, currentDeviceInformation.informationType, currentDeviceInformation.informationParameters);
  //JAGI_LOG_TIME;JAGI_PRINT.print(F("irCode ")); JAGI_PRINT.println(irCode,HEX);
  sendIRSignals(irCode);
  //mySender.send(NEC,irCode,32);
  currentDeviceInformation.sendTimestamp = millis();
  return EVENT_COMMAND_SENT;
}

#define ECHO_TIMEOUT_PERIOD 110
byte waitForEcho(){
  //EVENT_WAITING_FOR_ECHO
  //EVENT_TIMEDOUT_ECHO;
  //EVENT_RECEIVED_ECHO;
  if (receiveEcho()) return EVENT_RECEIVED_ECHO;
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  if (hasTimedOut(currentDeviceInformation.sendTimestamp,ECHO_TIMEOUT_PERIOD)) return EVENT_TIMEDOUT_ECHO;
  return EVENT_WAITING_FOR_ECHO;
}

#define RESPONSE_TIMEOUT_PERIOD 2000
byte waitForResponse(){
  //EVENT_WAITING_FOR_RESPONSE
  //EVENT_TIMEDOUT_RESPONSE;
  //EVENT_RECEIVED_RESPONSE;
  if (receiveResponse()) return EVENT_RECEIVED_RESPONSE;
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  if (hasTimedOut(currentDeviceInformation.sendTimestamp,RESPONSE_TIMEOUT_PERIOD)) return EVENT_TIMEDOUT_RESPONSE;
  return EVENT_WAITING_FOR_RESPONSE;
}

byte processResponse(){
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  JAGI_LOG_TIME;JAGI_PRINT.print(F("Response valid "));JAGI_PRINT.print(currentDeviceInformation.informationState); 
  JAGI_PRINT.print(F(" params "));JAGI_PRINT.println(currentDeviceInformation.informationValue,HEX);
  return EVENT_PROCESSING_RESPONSE_COMPLETE;
}

byte postProcessCommandSuccess(){
  return EVENT_PROCESSING_COMMAND_SUCCESS_COMPLETE;
}

#define MAX_NUM_OF_RETRIES 0
int previousItemForRetries = -1;
int currentItemReTryCount = 0;
byte retryCommand(){
  if(MAX_NUM_OF_RETRIES <= 0) return EVENT_RETRIES_EXHAUSTED;
  if(previousItemForRetries == currentItem) {
    currentItemReTryCount++;
    if(MAX_NUM_OF_RETRIES <= currentItemReTryCount) return EVENT_RETRIES_EXHAUSTED;
    else {
      JAGI_LOG2("Retry num : ", currentItemReTryCount+1);
      return EVENT_RETRIES_AVAILABLE;
    }
  }
  previousItemForRetries = currentItem;
  currentItemReTryCount = 0;
  JAGI_LOG2("Retry num : ", currentItemReTryCount+1);
  return EVENT_RETRIES_AVAILABLE;
}

byte postProcessCommandFailure(){
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  // This means we need to mark the information of device as invalid
  currentDeviceInformation.informationState = INFORMATION_STATE_INVALID;
  currentDeviceInformation.updateTimestamp = millis();
  //JAGI_LOG3(currentItem, " rxresp ", true);
  return EVENT_PROCESSING_COMMAND_FAILURE_COMPLETE;
}

boolean receiveEcho(){
  NecIrSignal headElement;
  boolean signalAvailable = buf.pull(&headElement);
  if(!signalAvailable) return false;
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  unsigned long irCode = formatCodeForSendingRequest(currentDeviceInformation.id, currentDeviceInformation.informationType, currentDeviceInformation.informationParameters);
  //printIrCode(irCode);
  //JAGI_LOG4(F("Receive Echo expected "),irCode, F(" received "), headElement.value);
  //printIrCode(headElement.value);
  if(headElement.value == irCode) return true;
  //if((random(10,10000) % 2) == 0) return true;
  // it was not some unknown signal
  // add it back to the queue
  buf.add(headElement);
  return false;
}

int previousItem = -1;
boolean currentItemResponseStaus = false;
boolean receiveResponse(){
  NecIrSignal headElement;
  boolean signalAvailable = buf.pull(&headElement);
  if(!signalAvailable) return false;
  DeviceInformation& currentDeviceInformation = deviceInformation[currentItem];
  unsigned long irCode = formatCodeForSendingRequest(currentDeviceInformation.id, currentDeviceInformation.informationType, currentDeviceInformation.informationParameters);
  //printIrCode(irCode);
  //JAGI_LOG4(F("Receive Response expected "),irCode, F(" received "), headElement.value);
  //printIrCode(headElement.value);
  struct extendedNecIrSignal decodedSignal;
  if(!getResponseParams(decodedSignal, irCode, headElement.value)) {
    // it was not some unknown signal
    // add it unknown back to the queue
    bufNotRelated.add(headElement);
    //JAGI_LOG2( F(" rxresp "), false);
    return false;
  }
  // This means we received a valid response
  currentDeviceInformation.informationValue = decodedSignal.params;
  currentDeviceInformation.informationState = INFORMATION_STATE_VALID;
  currentDeviceInformation.updateTimestamp = millis();
  //JAGI_LOG3(currentItem, " rxresp ", true);
  return true;
}

unsigned long rawIRSignalsSent = 0;
unsigned long rawIRSignalsEchoFailed = 0;
unsigned long rawIRSignalsReTransmitted = 0;
unsigned long recentTransmitted = 0xFFFFFFFF;
boolean isWaitingForEcho = false;
boolean isEchoReceived = false;
boolean isReTransmissionRequired = false;
void sendIRSignals(unsigned long iRcode){
  JAGI_LOG_TIME;JAGI_PRINT.print(F("Tx raw Ir Signal 0x  "));JAGI_PRINT.println(iRcode, HEX);
  //recentTransmitted = 0xFFFFFFFF;
  //mySender.send(NEC,iRcode,32);
  //recentTransmitted = iRcode;
  unsigned long sendTimeStampBefore = millis();
  mySender.send(NEC,iRcode,32);
  unsigned long sendTimeStamp = millis();
  long timeTakenToSend = sendTimeStamp - sendTimeStampBefore;
  recentTransmitted = iRcode;
  //JAGI_LOG_TIME;JAGI_PRINT.print(F(" Tx attempt ")); JAGI_PRINT.print(i);
  //JAGI_PRINT.print(F(" : "));JAGI_PRINT.print(sendTimeStamp); JAGI_PRINT.print(F(" - "));JAGI_PRINT.print(sendTimeStampBefore); JAGI_PRINT.print(F(" = "));JAGI_PRINT.println(timeTakenToSend);
  rawIRSignalsSent++;
}

unsigned long rawIRSignalsReceived = 0;
unsigned long rawIRSignalsDecodeFailures = 0;
unsigned long rawIRSignalsDuplicates = 0;
unsigned long rawIRSignalsDropped = 0;
boolean fullDetail = false;
void receiveIRSignals(){
  //Continue looping until you get a complete signal received
  if (myReceiver.getResults()) { // get the Results
    rawIRSignalsReceived++;
    if (myDecoder.decode()){  //Decode it
      //myDecoder.dumpResults(fullDetail);  //Now print results. Use false for less detail
      JAGI_LOG_TIME;JAGI_PRINT.print(F("Rx raw Ir Signal 0x  ")); JAGI_PRINT.println(myDecoder.value, HEX);
      NecIrSignal necIrSignal;
      necIrSignal.protocolNum = myDecoder.protocolNum;
      necIrSignal.value = myDecoder.value;
      necIrSignal.timestamp = millis();
      byte bufLength = buf.numElements();
      if(isWaitingForEcho && necIrSignal.value == recentTransmitted){
        isEchoReceived = true;
        isWaitingForEcho = false;
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Echo Received 0x  ")); JAGI_PRINT.println(necIrSignal.value, HEX);
        goto enableReceiver;
      }
      if(bufLength>0){
        NecIrSignal* prevElement = buf.peek(bufLength-1);
        //if(!prevElement) goto enableReceiver; // Needed only in multithreading or ISR
        if(necIrSignal.value == 0xFFFFFFFF || necIrSignal.value == prevElement->value) {
          JAGI_LOG_TIME;JAGI_PRINT.print(F("Duplicate found 0x  ")); JAGI_PRINT.println(necIrSignal.value, HEX);
          rawIRSignalsDuplicates++;
          goto enableReceiver;
        }
      }
      if(!buf.add(necIrSignal)) {    // Add it to the buffer
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Q failed 0x  ")); JAGI_PRINT.println(necIrSignal.value, HEX);
        rawIRSignalsDropped++;
      }
    } else { // This is an indication of possible collision or a simple hardware problem, hence re-transmit
      JAGI_LOG1(F("Decode failed"));
      rawIRSignalsDecodeFailures++;
      if(isWaitingForEcho){
        rawIRSignalsEchoFailed++;// = 0;
        isReTransmissionRequired = true;
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Raw Echo failed for 0x  ")); JAGI_PRINT.println(recentTransmitted, HEX);
        goto enableReceiver;
      }
    }
enableReceiver:
    myReceiver.enableIRIn();      //Restart receiver
  }
}

// Print the buffer's contents then empty it
void print_buf_contents(RingBufCPP<struct necIrSignal, MAX_NUM_NECSIGNALS_BUF>& bufLocal){
  NecIrSignal * e = 0;
  JAGI_LOG1("______Peek contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  for (int i = 0 ; i<bufLocal.numElements(); i++) {
    e = bufLocal.peek(i);
    if(!e) break;
    JAGI_LOG_TIME; JAGI_PRINT.print(F("t ")); JAGI_PRINT.print(e->timestamp);
    JAGI_PRINT.print(F(" v 0x  ")); JAGI_PRINT.println(e->value, HEX);
  }
  JAGI_LOG1("______Done peeking contents_______");
}

void printIrSignalStats(){
  JAGI_LOG1(F("RawTx\tEchoFail\tReTransmitted"));
  JAGI_LOG6(rawIRSignalsSent, F("\t"), rawIRSignalsEchoFailed, F("\t"), rawIRSignalsReTransmitted, F(""));
  JAGI_LOG1(F("RawRx\tDecodeFail\tDuplicate\tDropped"));
  JAGI_LOG_TIME; JAGI_PRINT.print(rawIRSignalsReceived,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.print(rawIRSignalsDecodeFailures,DEC);JAGI_PRINT.print(F("\t"));
  JAGI_PRINT.print(rawIRSignalsDuplicates,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.println(rawIRSignalsDropped,DEC);
}

// TODO Need to check so that the overflow doesnot cause too much time to timeout
#define TIME_OVERFLOW_VALUE 0xFFFFFFFF
unsigned long findElapsedTimeInMillis(const unsigned long& lastEventTime){
  unsigned long retVal;
  unsigned long currentTime = millis();
  retVal = currentTime - lastEventTime;
  //JAGI_LOG6(currentTime , F(" - "), lastEventTime, F(" = "), retVal, F(" ms"));
  if(retVal < 0 ) { // Overflow is automatically taken care with unsigned numbers, we need not do this.
    retVal = (TIME_OVERFLOW_VALUE - lastEventTime) + currentTime;
  }
  return retVal;
}
boolean hasTimedOut(const unsigned long& lastEventTime, const unsigned long& timeOutPeriod){
  long timeRemaining = timeOutPeriod - findElapsedTimeInMillis(lastEventTime);
  //JAGI_LOG4(F("timeRemaining "), lastEventTime, F(":"), timeRemaining);
  if (timeRemaining > 0 ) return false;
  return true;
}

/**
 * The return value is true if response signal is the correct response for the 
 */
boolean getResponseParams(struct extendedNecIrSignal& decodedSignal, unsigned long irCodeTx, unsigned long irCodeRx){
  boolean retVal = false;
  struct extendedNecIrSignal decodedSignalTx;
  decodeIrCodeToExtendedIrSignal(decodedSignalTx, irCodeTx);
  struct extendedNecIrSignal decodedSignalRx;
  decodeIrCodeToExtendedIrSignal(decodedSignalRx, irCodeRx);
  if(!isIrSignalAddressedToSelf(decodedSignalTx, decodedSignalRx)) return false;
  if(!isResponseValid(decodedSignalTx, decodedSignalRx)) return false;

  JAGI_LOG_TIME;JAGI_PRINT.print(F("Response "));JAGI_PRINT.println(decodedSignalRx.params,HEX);
  decodedSignal.params = decodedSignalRx.params;//0xFF&(irCodeRx>>0);
  decodedSignal.command = decodedSignalRx.command;//0xFF&(irCodeRx>>8);
  decodedSignal.source = decodedSignalRx.source;//0xFF&(irCode>Rx>16);
  decodedSignal.destination = decodedSignalRx.destination;//0xFF&(irCodeRx>>24);
  return true;
}

boolean isResponseValid(struct extendedNecIrSignal& decodedSignalTx, struct extendedNecIrSignal& decodedSignalRx){
  boolean retVal = false;
  if(decodedSignalTx.command == (decodedSignalRx.command & 0x7F)) retVal = true;
  //JAGI_LOG_TIME;JAGI_PRINT.print(F("Signal "));JAGI_PRINT.print(decodedSignalTx.command,HEX);JAGI_PRINT.print(F(" - "));JAGI_PRINT.println(decodedSignalRx.command,HEX);
  //JAGI_LOG2(F("Signal is containing valid response ?? "), retVal);
  return retVal;
}

boolean isIrSignalAddressedToSelf(struct extendedNecIrSignal& decodedSignalTx, struct extendedNecIrSignal& decodedSignalRx){
  boolean retVal = false;
  if(decodedSignalTx.destination == decodedSignalRx.source && decodedSignalTx.source == decodedSignalRx.destination) retVal = true;
  //JAGI_LOG2(F("Signal is addressed to self ?? "), retVal);
  return retVal;
}

void decodeIrCodeToExtendedIrSignal(struct extendedNecIrSignal& decodedSignal, unsigned long irCode){
  decodedSignal.destination = 0xFF & (irCode>>24);
  decodedSignal.source      = 0xFF & (irCode>>16);
  decodedSignal.command     = 0xFF & (irCode>>8);
  decodedSignal.params      = 0xFF & (irCode>>0);
}

void printExtendedIrSignal(struct extendedNecIrSignal& decodedSignal){
  JAGI_LOG_TIME;
  JAGI_PRINT.print(decodedSignal.destination,HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.print(decodedSignal.source,HEX);JAGI_PRINT.print(F("-"));
  JAGI_PRINT.print(decodedSignal.command,HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.println(decodedSignal.params,HEX);
}

void printIrCode(unsigned long irCode){
  JAGI_LOG_TIME;
  JAGI_PRINT.print(0xFF & (irCode>>24),HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.print(0xFF & (irCode>>16),HEX);JAGI_PRINT.print(F("-"));
  JAGI_PRINT.print(0xFF & (irCode>>8),HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.println(0xFF & (irCode>>0),HEX);
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
  //JAGI_PRINT.print(typeOfSignal, HEX); JAGI_PRINT.print("-");JAGI_PRINT.print(destinationAddress, HEX); JAGI_PRINT.print("-");JAGI_PRINT.print(sourceAddress, HEX); JAGI_PRINT.print("-");
  //JAGI_PRINT.print(command, HEX); JAGI_PRINT.print("-");JAGI_PRINT.println(commandParams, HEX);
  retVal = retVal | destinationAddress;
  retVal = retVal << 8;
  //JAGI_PRINT.println(retVal, HEX); 
  retVal = retVal | sourceAddress;
  retVal = retVal << 8;
  //JAGI_PRINT.println(retVal, HEX); 
  if(typeOfSignal) command = command & 0x7F; // always set the MSB to 0 to indicate it is a request
  else command = command | 0x80; // always set the MSB to 1 to indicate it is a response
  retVal = retVal | command;
  retVal = retVal << 8;
  //JAGI_PRINT.println(retVal, HEX); 
  retVal = retVal | commandParams;
  //JAGI_PRINT.println(retVal, HEX);
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
  char color[] = "----";
  if (DEVICE_IDENTITY & 0x0008) color[0] = 'R';
  if (DEVICE_IDENTITY & 0x0004) color[1] = 'G';
  if (DEVICE_IDENTITY & 0x0002) color[2] = 'B';
  if (DEVICE_IDENTITY & 0x0001) color[3] = 'Y';
  if(MASTER_MODE) {JAGI_LOG4("MASTER ", DEVICE_IDENTITY, " Color ", color);}
  else {JAGI_LOG4("SLAVE  ", DEVICE_IDENTITY, " Color ", color);}
}

BLYNK_READ(V0)
{
  JAGI_LOG2(F("Blynk HB to 01 V"),V0);
  DeviceInformation& currentDeviceInformation = deviceInformation[V0];
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed HB to 01 V"),V0);
    //unsigned long uptimeInMinutes = millis() / 1000;
    //Blynk.virtualWrite(V0, uptimeInMinutes, " local secs");
    Blynk.virtualWrite(V0, "NotReachable");
    return;
  }
  int uptime = currentDeviceInformation.informationValue;
  JAGI_LOG_TIME;JAGI_PRINT.print(F("V0 uptime "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(uptime);
  Blynk.virtualWrite(V0, uptime, " mins");
}
BLYNK_READ(V4)
{
  JAGI_LOG2(F("Blynk HB to 02 V"),V4);
  DeviceInformation& currentDeviceInformation = deviceInformation[V4];
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed HB to 02 V"),V4);
    //unsigned long uptimeInMinutes = millis() / 1000;
    //Blynk.virtualWrite(V4, uptimeInMinutes);
    Blynk.virtualWrite(V4, "NotReachable");
    return;
  }
  int uptime = currentDeviceInformation.informationValue;
  JAGI_LOG_TIME;JAGI_PRINT.print(F("V4 uptime "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(uptime);
  Blynk.virtualWrite(V4, uptime, " mins");
}
BLYNK_READ(V8)
{
  JAGI_LOG2(F("Blynk HB to 03 V"),V8);
  DeviceInformation& currentDeviceInformation = deviceInformation[V8];
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed HB to 03 V"),V8);
    //unsigned long uptimeInMinutes = millis() / 1000;
    //Blynk.virtualWrite(V8, uptimeInMinutes);
    Blynk.virtualWrite(V8, "NotReachable");
    return;
  }
  int uptime = currentDeviceInformation.informationValue;
  JAGI_LOG_TIME;JAGI_PRINT.print(F("V8 uptime "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(uptime);
  Blynk.virtualWrite(V8, uptime, " mins");
}
BLYNK_READ(V12)
{
  JAGI_LOG2(F("Blynk HB to 04 V"),V12);
  DeviceInformation& currentDeviceInformation = deviceInformation[V12];
  int uptime = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG3(currentDeviceInformation.updateTimestamp,F("Blynk Failed HB to 04 V"),V12);
    //unsigned long uptimeInMinutes = millis() / 1000;
    //Blynk.virtualWrite(V12, uptimeInMinutes);
    //JAGI_LOG_TIME;JAGI_PRINT.print(F("V12 invalid uptime "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(uptimeInMinutes, HEX);
    Blynk.virtualWrite(V12, "NotReachable");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V12 uptime "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(uptime, HEX);
    Blynk.virtualWrite(V12, uptime, " mins");
  }
}

BLYNK_READ(V1)
{
  JAGI_LOG2(F("Blynk PINB to 01 V"),V1);
  DeviceInformation& currentDeviceInformation = deviceInformation[V1];
  int pins = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    Blynk.virtualWrite(V1, PINB, " local pins");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V1 PINB "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(pins, HEX);
    Blynk.virtualWrite(V1, pins);
  }
}

BLYNK_READ(V5)
{
  JAGI_LOG2(F("Blynk PINB to 02 V"),V5);
  DeviceInformation& currentDeviceInformation = deviceInformation[V5];
  int pins = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed PINB to 02 V"),V5);
    //Blynk.virtualWrite(V5, PINB, " local pins");
    Blynk.virtualWrite(V5, "--");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V5 PINB "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(pins, HEX);
    Blynk.virtualWrite(V5, pins);
  }
}

BLYNK_READ(V9)
{
  JAGI_LOG2(F("Blynk PINB to 03 V"),V9);
  DeviceInformation& currentDeviceInformation = deviceInformation[V9];
  int pins = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    Blynk.virtualWrite(V9, PINB, " local pins");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V9 PINB "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(pins, HEX);
    Blynk.virtualWrite(V9, pins);
  }
}

BLYNK_READ(V13)
{
  JAGI_LOG2(F("Blynk PINB to 04 V"),V13);
  DeviceInformation& currentDeviceInformation = deviceInformation[V13];
  int pins = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed PINB to 04 V"),V13);
    //Blynk.virtualWrite(V13, PINB, " local pins");
    Blynk.virtualWrite(V13, "--");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V13 PINB "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(pins, HEX);
    Blynk.virtualWrite(V13, pins);
  }
}

BLYNK_READ(V6)
{
  JAGI_LOG2(F("Blynk FANSPEED to 02 V"),V6);
  DeviceInformation& currentDeviceInformation = deviceInformation[V6];
  int fanspeed = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed FANSPEED to 02 V"),V6);
    //Blynk.virtualWrite(V2, FANSPEED, " local speed");
    Blynk.virtualWrite(V6, "-0-");
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V6 FANSPEED "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(fanspeed);
    Blynk.virtualWrite(V6, fanspeed);
  }
}

BLYNK_READ(V14)
{
  JAGI_LOG2(F("Blynk FANSPEED to 04 V"),V14);
  DeviceInformation& currentDeviceInformation = deviceInformation[V14];
  int fanspeed = currentDeviceInformation.informationValue;
  if(currentDeviceInformation.informationState == INFORMATION_STATE_INVALID){
    if (hasTimedOut(currentDeviceInformation.updateTimestamp,INFORMATION_VALIDITY_TIMEOUT)) JAGI_LOG2(F("Blynk Failed FANSPEED to 04 V"),V14);
    //Blynk.virtualWrite(V2, FANSPEED, " local speed");
    Blynk.virtualWrite(V14, 0);
    return;
  } else {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("V14 FANSPEED "));JAGI_PRINT.print(currentDeviceInformation.informationValue,HEX);JAGI_PRINT.print(F(" "));JAGI_PRINT.println(fanspeed);
    Blynk.virtualWrite(V14, fanspeed);
  }
}


// Below are all for some debugging
BLYNK_WRITE(V28){
  // TODO dev monitoring
}

BLYNK_WRITE(V29)
{
  JAGI_LOG2(F("Blynk Send signal V"), V29);
  //sendHBRequest(gCurrentDeviceId);
  //gCurrentDeviceId++;
  //if(gCurrentDeviceId > MAX_NUMBER_OF_DEVICES) gCurrentDeviceId=1;
}

BLYNK_WRITE(V30)
{
  JAGI_LOG2(F("Blynk Id Self V"), V30);
  identifySelf();
  print_buf_contents(buf);
  JAGI_LOG_TIME; JAGI_PRINT.print(F("pin 8 to 13  ")); JAGI_PRINT.println(PINB, BIN);
  Blynk.virtualWrite(V30, PINB);
}

BLYNK_WRITE(V31)
{
  JAGI_LOG2(F("Blynk Sig Stats V"), V31);
  printIrSignalStats();
}
