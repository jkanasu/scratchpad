// Some times we cannot use the default Serial for debug/logs, hence we use SoftwareSerial
// You could use a spare Hardware Serial on boards that have it (like Mega)
//#include <SoftwareSerial.h>
//SoftwareSerial DebugSerial(10, 11); // RX, TX

#define JAGI_PRINT Serial // You can change this to use a Software Serial
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
  // Debug console
  //DebugSerial.begin(57600); // NOTE baud rate

  Serial.begin(57600);
  delay(2000); while (!Serial); //delay for Leonardo
  // identify the self device id and the mode of operation
  identifySelf();
  JAGI_LOG1(F("IrDuplexCommunication.ino : Every time you press a key in serial monitor; monitoring using HB will be done."));
  //Enable auto resume and pass it the address of your extra buffer
  //myReceiver.enableAutoResume(myBuffer);
  myReceiver.enableIRIn(); // Start the receiver
  JAGI_LOG1(F("Ready to receive IR signals"));
}

boolean fullDetail=false;
void loop() {
  processSerialCommands();
  if(MASTER_MODE) deviceMonitoring();
  else processIRSignals();
  //delay(1000);
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
unsigned long previousMonitoringCycleTime = millis();
#define MONITORING_INTERVAL 60000 // 2 second
void deviceMonitoring(){
  JAGI_LOG4(F("deviceMonitoring "), gCurrentDeviceId, F(" : "), hbStatusOfDevices[gCurrentDeviceId].hbStatus);
  if(gCurrentDeviceId>MAX_NUMBER_OF_DEVICES) {
    if(!hasTimedOut(previousMonitoringCycleTime, MONITORING_INTERVAL )) return;
    gCurrentDeviceId = 1;
    previousMonitoringCycleTime = millis();
    JAGI_LOG1(F("Restarting from beginning"));
  }
  monitorDevice(gCurrentDeviceId);
  JAGI_LOG4(F("Device Status "), gCurrentDeviceId, F(" : "), hbStatusOfDevices[gCurrentDeviceId].hbStatus);
  //JAGI_PRINT.print(F("Device Status ")); JAGI_PRINT.print(gCurrentDeviceId); JAGI_PRINT.print(F(" : "));JAGI_PRINT.println(hbStatusOfDevices[gCurrentDeviceId].hbStatus); 
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
    JAGI_LOG2(F("HB request sent "), currentDeviceId);
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_RESPONSE_WAITING){
    // someone else is expected to change this status
    // how long are we waiting?
    unsigned long waitingTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    if(waitingTime > HB_TIMEOUT_PERIOD){
      // donot put any delay here, the main loop() may need that time to do some work
      currentDeviceStatus.hbStatus = HB_FAILED;
      JAGI_LOG2(F("HB timedout "), currentDeviceId);
      goto monitorDeviceWorkFinish;//return;
    }
    processIRSignals();
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_FAILED){
    JAGI_LOG2(F("HB fail "), currentDeviceId);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    goto monitorDeviceWorkFinish;//return;
  }
  if(currentDeviceStatus.hbStatus == HB_SUCCESS){
    JAGI_LOG2(F("HB success "), currentDeviceId);
    currentDeviceStatus.hbStatus = HB_NOT_SENT;
    //unsigned long lastHBActivityTime = findElapsedTimeInMillis(currentDeviceStatus.lastEventTime);//millis() - currentDeviceStatus.lastEventTime;
    goto monitorDeviceWorkFinish;//return;
  }
monitorDeviceWorkFinish:
  currentDeviceStatus.hbStatus;
  JAGI_LOG4(F("Monitor HB status "), currentDeviceId, F(" : "), currentDeviceStatus.hbStatus);
}

void processSerialCommands(){
  if (JAGI_PRINT.available() == 0) return;
  int inComingCommand = JAGI_PRINT.read();
  if (inComingCommand == -1) return;
  JAGI_LOG2(F("command received "), inComingCommand);
  switch(inComingCommand){
    case 49: // 1 in ascii
      processSerial1();
      break;
    case 50: // 2 in ascii
      processSerial2();
      break;
    case 52: // 4 in ascii
      processSerial4();
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
      JAGI_LOG_TIME;JAGI_PRINT.print(F("Unknown serial signal ")); JAGI_PRINT.print(inComingCommand,DEC);JAGI_PRINT.print(F("-"));JAGI_PRINT.println(inComingCommand,HEX);
      processSerialX();
      break;
  }
}

void processSerial1(){
  JAGI_LOG1(F("Send signal"));
  sendHBRequest(gCurrentDeviceId);
  gCurrentDeviceId++;
  if(gCurrentDeviceId > MAX_NUMBER_OF_DEVICES) gCurrentDeviceId=1;
}

void processSerial2(){
  JAGI_LOG1(F("Receive signal"));
  processIRSignals();
}

// The analog step is as per arduino, i.e. 5 / 1024 = 0.0049
#define ANALOG_VOLTAGE_STEP 0.0049
void processSerial4(){
  JAGI_LOG1(F("Print analog pin A4 value"));
  // read the input on analog pin
  int sensorValue = analogRead(A4);
  JAGI_LOG2(F("Sensor Value Read "), sensorValue);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // Change this range from (0 - 1023) to (0 - 50), this is done because we can only transmit a byte i.e. max 2^8 = 256
  int voltage = (int)(10.0 * sensorValue * ANALOG_VOLTAGE_STEP);
  // print out the value you read:
  JAGI_LOG2(F("Voltage Read "), voltage);
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

void processIRSignals(){
  JIRCode jIrCode;
  boolean isValid = receiveValidJIRSignal(jIrCode);
  if(isValid){
    JAGI_LOG_TIME;JAGI_PRINT.print(F("valid Signal received 0x"));JAGI_PRINT.println(jIrCode.value, HEX);
    processJIRSignals(jIrCode);
  }
}

#define HB_REQUEST_CODE_PARAMS 0x00
void sendHBRequest(byte currentDeviceId){
  unsigned long hbRequestCode = formatCodeForSendingRequest(currentDeviceId,REQUEST_HB,gHBsignalNumber);
  //mySender.send(NEC,hbRequestCode,32);
  sendIRSignals(hbRequestCode);
  JAGI_LOG_TIME;JAGI_PRINT.print(F("HB Request Signal sent 0x"));JAGI_PRINT.println(hbRequestCode, HEX);
  //delay(1000);
}

#define HB_RESPONSE_CODE_PARAMS 0x00
void sendHBResponse(JIRCode& jIrCode){
  byte uptimeInMinutes = millis() / 60000; // obviously will roll over in 256 minutes which is like 4 hours 16 minutes
  unsigned long hbResponseCode = formatCodeForSendingResponse(jIrCode.sourceAddress,RESPONSE_HB,uptimeInMinutes);
  //mySender.send(NEC,hbResponseCode,32);
  delay(100);// just like that delay 100 ms
  sendIRSignals(hbResponseCode);
  JAGI_LOG_TIME;JAGI_PRINT.print(F("HB Response Signal sent 0x"));JAGI_PRINT.println(hbResponseCode, HEX);
}

#define FANSPEED_PIN A4
void sendFSResponse(JIRCode& jIrCode){
  // read the input on analog pin
  int sensorValue = analogRead(FANSPEED_PIN);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  // Change this range from (0 - 1023) to (0 - 50), this is done because we can only transmit a byte i.e. max 2^8 = 256
  int voltage = (int)(10.0 * sensorValue * ANALOG_VOLTAGE_STEP);
  // print out the value you read:
  JAGI_LOG2(F("Voltage Read "), voltage);
  unsigned long fsResponseCode = formatCodeForSendingResponse(jIrCode.sourceAddress,RESPONSE_FANSPEED,voltage);
  //mySender.send(NEC,hbResponseCode,32);
  delay(100);// just like that delay 100 ms
  sendIRSignals(fsResponseCode);
  JAGI_LOG_TIME;JAGI_PRINT.print(F("LS Response Signal sent 0x"));JAGI_PRINT.println(fsResponseCode, HEX);
}

void sendLSResponse(JIRCode& jIrCode){
  unsigned long lsResponseCode = formatCodeForSendingResponse(jIrCode.sourceAddress,RESPONSE_LIGHTSTATUS,PINB);
  //mySender.send(NEC,hbResponseCode,32);
  delay(100);// just like that delay 100 ms
  sendIRSignals(lsResponseCode);
  JAGI_LOG_TIME;JAGI_PRINT.print(F("LS Response Signal sent 0x"));JAGI_PRINT.println(lsResponseCode, HEX);
}

void processHBResponse(JIRCode& jIrCode){
  JAGI_LOG2(F("HB Response Signal received "), jIrCode.sourceAddress);
  if(jIrCode.sourceAddress>MAX_NUMBER_OF_DEVICES) return;
  DeviceHBStatus& currentDeviceStatus = hbStatusOfDevices[jIrCode.sourceAddress];
  if(currentDeviceStatus.hbStatus == HB_RESPONSE_WAITING){
    currentDeviceStatus.hbStatus = HB_SUCCESS;
    currentDeviceStatus.lastEventTime = millis();
  } else {
     JAGI_LOG2(F("HB Status is not correct "), currentDeviceStatus.hbStatus);
  }
}

void processJIRSignals(JIRCode& jIrCode){
  JAGI_LOG1(F("processJIR Signals "));
  jIrCode = convertToJIRCodeFromRawDecodedValue(jIrCode);
  if(jIrCode.protocolNum != NEC || jIrCode.destinationAddress != DEVICE_IDENTITY){
    processJIRSignalsAddressedToOthers(jIrCode);
    return;
  }
  // Below assumes the IR signal is addressed to self
  switch(jIrCode.command){
    case REQUEST_HB:
      JAGI_LOG2(F("HB request from "), jIrCode.sourceAddress);
      sendHBResponse(jIrCode);
      break;        
    case REQUEST_LIGHTSTATUS:
      JAGI_LOG2(F("LIGHTSTATUS request from "), jIrCode.sourceAddress);
      sendLSResponse(jIrCode);
      break;        
    case REQUEST_FANSPEED:
      JAGI_LOG2(F("FANSPEED request from "), jIrCode.sourceAddress);
      sendFSResponse(jIrCode);
      break;        
    case RESPONSE_HB:
      JAGI_LOG2(F("HB response from "), jIrCode.sourceAddress);
      processHBResponse(jIrCode);
      break;
    default:
      JAGI_LOG_TIME;JAGI_PRINT.print(F("unknown signal 0x")); JAGI_PRINT.println(jIrCode.value,HEX);
      //testEchoOfPreviousTransmittedCode(jIrCode.value);
      break;
  }
}

void processJIRSignalsAddressedToOthers(JIRCode& jIrCode){
    JAGI_LOG2(F("processJIRSignalsAddressedToOthers Addressed to "), jIrCode.destinationAddress);
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
#define ECHO_TIMEOUT_PERIOD 1000 // 1 second // For NEC standard, the transmission period is upto 110 ms
#define NUM_IRSIGNAL_MIN_RETRANSMITS 1
#define NUM_IRSIGNAL_MAX_RETRANSMITS 1
void sendIRSignals(unsigned long iRcode){
  JAGI_LOG_TIME;JAGI_PRINT.print(F("Send raw Ir Signal 0x"));JAGI_PRINT.println(iRcode, HEX);
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
      JAGI_LOG2(F("Tx attempt "), i);
      //JAGI_LOG_TIME;JAGI_PRINT.print(F(" Tx attempt ")); JAGI_PRINT.print(i);
      //JAGI_PRINT.print(F(" : "));JAGI_PRINT.print(sendTimeStamp); JAGI_PRINT.print(F(" - "));JAGI_PRINT.print(sendTimeStampBefore); JAGI_PRINT.print(F(" = "));JAGI_PRINT.println(timeTakenToSend);
      rawIRSignalsSent++;
      isWaitingForEcho = true;
      isReTransmissionRequired = false;
    }
    while(true){
      receiveIRSignals();
      if(isEchoReceived){
        //JAGI_PRINT.println(F("Echo Received"));
        break;
      }
      if(hasTimedOut(sendTimeStamp, ECHO_TIMEOUT_PERIOD)){
        JAGI_LOG1(F("Echo Timed Out"));
        isReTransmissionRequired = true;
        rawIRSignalsEchoFailed++;
        rawIRSignalsReTransmitted++;
        isWaitingForEcho = false;
        break;
      }
    }
    if(isEchoReceived ) {
      if(i < NUM_IRSIGNAL_MIN_RETRANSMITS) {
        isReTransmissionRequired = true;
        rawIRSignalsReTransmitted++;
        JAGI_LOG2(F("Force Tx attempt "), i);
        //delay(200);// else we may simply collide with our selves, actually NO :-(
        continue;
      }
      break;
    }
  }while(i < NUM_IRSIGNAL_MAX_RETRANSMITS);
  if(isEchoReceived) {
    JAGI_LOG_TIME;JAGI_PRINT.print(F("Raw Send Signal complete 0x"));JAGI_PRINT.println(iRcode, HEX);
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
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Echo Received 0x")); JAGI_PRINT.println(necIrSignal.value, HEX);
        goto enableReceiver;
      }
      if(bufLength>0){
        NecIrSignal* prevElement = buf.peek(bufLength-1);
        //if(!prevElement) goto enableReceiver; // Needed only in multithreading or ISR
        if(necIrSignal.value == 0xFFFFFFFF || necIrSignal.value == prevElement->value) {
          JAGI_LOG_TIME;JAGI_PRINT.print(F("Duplicate found 0x")); JAGI_PRINT.println(necIrSignal.value, HEX);
          rawIRSignalsDuplicates++;
          goto enableReceiver;
        }
      }
      if(!buf.add(necIrSignal)) {    // Add it to the buffer
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Q failed 0x")); JAGI_PRINT.println(necIrSignal.value, HEX);
        rawIRSignalsDropped++;
      }
    } else { // This is an indication of possible collision or a simple hardware problem, hence re-transmit
      JAGI_LOG1(F("Decode failed"));
      rawIRSignalsDecodeFailures++;
      if(isWaitingForEcho){
        rawIRSignalsEchoFailed++;// = 0;
        isReTransmissionRequired = true;
        JAGI_LOG_TIME;JAGI_PRINT.print(F("Raw Echo failed for 0x")); JAGI_PRINT.println(recentTransmitted, HEX);
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
  JAGI_LOG1("______Peek contents of ring buffer_______");
  // Keep looping until pull() returns NULL
  for (int i = 0 ; i<buf.numElements(); i++) {
    e = buf.peek(i);
    if(!e) break;
    JAGI_LOG_TIME; JAGI_PRINT.print(F("t ")); JAGI_PRINT.print(e->timestamp);
    JAGI_PRINT.print(F(" v 0x")); JAGI_PRINT.println(e->value, HEX);
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
    JAGI_LOG_TIME;JAGI_PRINT.print(F("t ")); JAGI_PRINT.print(e.timestamp);
    JAGI_PRINT.print(F(" v 0x")); JAGI_PRINT.println(e.value, HEX);
  }
  JAGI_LOG1("______Done dumping contents_______");
}

void printIrSignalStats(){
  JAGI_LOG1(F("RawTx\tEchoFail\tReTransmitted"));
  JAGI_LOG6(rawIRSignalsSent, F("\t"), rawIRSignalsEchoFailed, F("\t"), rawIRSignalsReTransmitted, F(""));
  JAGI_LOG1(F("RawRx\tDecodeFail\tDuplicate\tDropped"));
  JAGI_LOG_TIME; JAGI_PRINT.print(rawIRSignalsReceived,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.print(rawIRSignalsDecodeFailures,DEC);JAGI_PRINT.print(F("\t"));
  JAGI_PRINT.print(rawIRSignalsDuplicates,DEC);JAGI_PRINT.print(F("\t"));JAGI_PRINT.println(rawIRSignalsDropped,DEC);
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
  JAGI_LOG_TIME;JAGI_PRINT.print(jIrCode.protocolNum,DEC);JAGI_PRINT.print(F("-"));JAGI_PRINT.print(jIrCode.value,HEX);JAGI_PRINT.print(F("-"));
  JAGI_PRINT.print(jIrCode.destinationAddress,HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.print(jIrCode.sourceAddress,HEX);JAGI_PRINT.print(F("-"));
  JAGI_PRINT.print(jIrCode.command,HEX);JAGI_PRINT.print(F("-"));JAGI_PRINT.println(jIrCode.parameterData,HEX);
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
