 /*************************************
   Samantha Woods
   Skripsie Hand Prototype
   Hand Actuation Code
 *************************************/

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>


/***Servos***/
Servo PinkyFinger;
Servo RingFinger;
Servo MiddleFinger;
Servo IndexFinger;
Servo ThumbFlex;
Servo ThumbAA;
Servo ThumbOp;

int Servo0Pin = 15;
int Servo1Pin = 2;
int Servo2Pin = 19;
int Servo3Pin = 16;
int Servo4Pin = 17;
int Servo5Pin = 5;
int Servo6Pin = 18;

/***Pressure Sensors***/
int ThumbSense = 39;
int IndexSense = 34;
int MiddleSense = 35;
int RingSense = 32;
int PinkySense = 33;

/***Servo Position Array Indexing***
 ***********************************
   0 = Pinky Finger
   1 = Ring Finger
   2 = Middle Finger
   3 = Index Finger
   4 = Thumb Bending
   5 = Thumb Abduction
   6 = Thumb Opposition
 ***********************************
*/
//uint8_t ServoMaxPosition[] = {80, 50, 80, 50, 46, 22, 70};                         //Maximum angle the servo can run to, read from NX simulation
uint8_t ServoMaxPosition[] = {76, 55, 76, 55, 37, 70, 80};                         //Maximum angle the servo can run to, read from NX simulation
uint8_t ServoMinPosition[] = {0, 0, 0, 0, 0, 0, 0};      //Minumum angle the servo can run to, read from NX simulation
uint8_t ServoCurrentPosition[] = {0, 0, 0, 0, 0, 0, 0};  //Position the servo is set to currently
uint8_t ServoSetPosition[] = {0, 0, 0, 0, 0, 0, 0};      //Position the servo is running to, set from glove readings, values sent wirelessly

/***Pressure Sensor Array Indexing***
 ************************************
   0 = Pinky Finger
   1 = Ring Finger
   2 = Middle Finger
   3 = Index Finger
   4 = Thumb
 ************************************
   Value range = 0 - 9
*/
uint8_t HandPressure[] = {0, 0, 0, 0, 0};             //Interval value of the FSR analog reading from the hand (ADC)
uint8_t GlovePressure[] = {1, 1, 1, 1, 1};            //Interval value of the FSR analog reading from the glove (ADC), values sent wirelessly


/***MAC Address Arrays***/
uint8_t BOARD3Address[] = {0x58, 0xBF, 0x25, 0x37, 0x2F, 0x30};

/*Timers*/
hw_timer_t *Timer5 = NULL;      //5 millisecond tick
hw_timer_t *Timer8 = NULL;      //8 millisecond tick

/*Miscellaneous Other Variables*/
uint8_t HolderArray[12];         //Placeholder array used to hold the incoming data
uint8_t SendArray[12];           //Placeholder array used to hold the outgoing data





/***FUNCTIONS***
 ***************
 ***************
*/

/***ESP_NOW Callback on data sent, used to confirm whether data sent successfully***/
void OnDataTransmission(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nPacket Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


/***ESP-NOW Callback on data received, used to handle the incoming data***/
void OnDataReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  int i;

  Serial.write("received");
  memcpy(&HolderArray, incomingData, sizeof(HolderArray));                                //Copy incoming data to the holder array
  for (i = 0; i < 7; i++) {
    if (HolderArray[i] >= ServoMaxPosition[i]) {
      ServoSetPosition[i] = ServoMaxPosition[i];
    }
    else if (HolderArray[i] <= ServoMinPosition[i]) {
      ServoSetPosition[i] = ServoMinPosition[i];
    }
    else {
      ServoSetPosition[i] = HolderArray[i];
    }
  }                                                                                      //Normalise Incoming Position Values to within servo's active range

  for (i = 7; i < 12; i++) {
    GlovePressure[i - 7] = 1;
  }                                                                                      //Record received glove pressure readings for comparison to hand pressure readings
}

/***Function for ESP-NOW data transmission***/
void SendData() {
  int i;
  for (i = 0; i < 7; i++) {
    SendArray[i] = ServoCurrentPosition[i];
  }
  for (i = 7; i < 12; i++) {
    SendArray[i] = HandPressure[i - 7];
  }
  esp_err_t result = esp_now_send(BOARD3Address, (uint8_t *) &SendArray, sizeof(SendArray));
}



/***Timer Interrupt Functions***
 ******************************/

/*Timer5 Interrupt*/
void IRAM_ATTR OnTimer5() {


  if ((ServoSetPosition[0] > ServoCurrentPosition[0]) /*&& (HandPressure[0] != GlovePressure[0])*/) {
    ServoCurrentPosition[0]++;
  }
  else if (ServoSetPosition[0] < ServoCurrentPosition[0]) {
    ServoCurrentPosition[0]--;
  }

  if ((ServoSetPosition[2] > ServoCurrentPosition[2]) /*&& (HandPressure[2] != GlovePressure[2])*/) {
    ServoCurrentPosition[2]++;
  }
  else if (ServoSetPosition[2] < ServoCurrentPosition[2]) {
    ServoCurrentPosition[2]--;
  }




  PinkyFinger.write(ServoMaxPosition[0] - ServoCurrentPosition[0]);
  RingFinger.write(ServoCurrentPosition[1]);
  MiddleFinger.write(ServoMaxPosition[2] - ServoCurrentPosition[2]);
  IndexFinger.write(ServoCurrentPosition[3]);
  ThumbFlex.write(ServoCurrentPosition[4]);
  //ThumbAA.write(ServoMaxPosition[5] - ServoCurrentPosition[5]);
  ThumbOp.write(ServoCurrentPosition[6]);
  ThumbAA.write(ServoCurrentPosition[5]);


}

/*Timer8 Interrupt*/
void IRAM_ATTR OnTimer8() {

  if ((ServoSetPosition[1] > ServoCurrentPosition[1]) /*&& (HandPressure[1] != GlovePressure[1])*/) {
    ServoCurrentPosition[1]++;
  }
  else if (ServoSetPosition[1] < ServoCurrentPosition[1]) {
    ServoCurrentPosition[1]--;
  }

  if ((ServoSetPosition[3] > ServoCurrentPosition[3]) /*&& (HandPressure[3] != GlovePressure[3])*/) {
    ServoCurrentPosition[3]++;
  }
  else if (ServoSetPosition[3] < ServoCurrentPosition[3]) {
    ServoCurrentPosition[3]--;
  }
  
  if ((ServoSetPosition[4] > ServoCurrentPosition[4]) /*&& (HandPressure[3] != GlovePressure[3])*/) {
    ServoCurrentPosition[4]++;
  }
  else if (ServoSetPosition[4] < ServoCurrentPosition[4]) {
    ServoCurrentPosition[4]--;
  }

  if ((ServoSetPosition[5] > ServoCurrentPosition[5]) /*&& (HandPressure[3] != GlovePressure[3])*/) {
    ServoCurrentPosition[5]++;
  }
  else if (ServoSetPosition[5] < ServoCurrentPosition[5]) {
    ServoCurrentPosition[5]--;
  }

    if ((ServoSetPosition[6] > ServoCurrentPosition[6]) /*&& (HandPressure[3] != GlovePressure[3])*/) {
    ServoCurrentPosition[6]++;
  }
  else if (ServoSetPosition[6] < ServoCurrentPosition[6]) {
    ServoCurrentPosition[6]--;
  }

}


/***END FUNCTIONS***
 *******************/




void setup() {
  /*Begin SetUp for ESP-Now
    ~~~~~~~~~~~~~~~~~~~~~~~
  */
  //Initialise Serial Monitor, Set device as Wifi station, Initialise ESP-NOW
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Register callback function to get transmission status
  esp_now_register_send_cb(OnDataTransmission);
  //Register callback function to be called when data is recieved
  esp_now_register_recv_cb(OnDataReceive);

  //Register BOARD3 as Peer
  esp_now_peer_info_t BOARD3Info;
  memset(&BOARD3Info, 0, sizeof(BOARD3Info));
  memcpy(BOARD3Info.peer_addr, BOARD3Address, 6);
  BOARD3Info.channel = 0;
  BOARD3Info.encrypt = false;

  //Add BOARD3 as Peer
  if (esp_now_add_peer(&BOARD3Info) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  /*End SetUp for ESP-Now
    ~~~~~~~~~~~~~~~~~~~~~~~
  */

  /*servo set up*/
  PinkyFinger.attach(Servo0Pin);
  RingFinger.attach(Servo1Pin);
  MiddleFinger.attach(Servo2Pin);
  IndexFinger.attach(Servo3Pin);
  ThumbFlex.attach(Servo4Pin);
  ThumbAA.attach(Servo5Pin);
  ThumbOp.attach(Servo6Pin);
  /*end servo set up*/

  /*timer set up*/
  Timer5 = timerBegin(0, 80, true);                     //attach timer5 to 1st timer of 4, 1 tick = 1us
  Timer8 = timerBegin(1, 80, true);                     //attach timer8 to 1st timer of 4, 1 tick = 1us

  timerAttachInterrupt(Timer5, &OnTimer5, true);       //attach interrupt function to timer5
  timerAttachInterrupt(Timer8, &OnTimer8, true);       //attach interrupt function to timer8

  timerAlarmWrite(Timer5, 5000, true);                 //Set timer5 alarm to call interrupt every 5ms
  timerAlarmWrite(Timer8, 8000, true);                 //Set timer8 alarm to call interrupt every 8ms

  timerAlarmEnable(Timer5);                             //enable timer5
  timerAlarmEnable(Timer8);                             //enable timer8
  /*end timer set up*/

}

void loop() {
  // put your main code here, to run repeatedly:

}


/*GUIDE FOR USING ESP_NOW*/

/***Board Library***
   Find MAC Address:
   Flash get_mac_address program to board, read from serial terminal.
 ***************************
 ***Board Descriptions***
  ~~~~~~~~~~~~~~~~~~~~~~~
  Board For The Hand (BOARD1):
  DoIt ESP32 Devkit V1
  bolt in hole left of the usb port
  MAC = 58:BF:25:37:3A:28
  Note: This board overheats on USB. Do not plug in for long
 ***
  Board For The Glove Running ADC (BOARD2):
  DoIt ESP32 Devkit V1
  bolt in hole right of usb port
  Note: This board overheats on USB. Not as bad as BOARD1
  MAC = 58:BF:25:36:F3:00
 ***
  Board For The Glove Running ESP-NOW (BOARD3):
  DoIt ESP32 Devkit V1
  No Bolt
  MAC = 58:BF:25:37:2F:30
*/

/* What to put in void Setup():
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   1. Initialise Serial Monitor, Set device as Wifi station, Initialise ESP-NOW
   2. Register send and recieve callback functions
   3. Register your peer using the code format:
         esp_now_peer_info_t NameHereInfo;
         memcpy(NameHereInfo.peer_addr, NameHereAddress, 6);
         NameHereInfo.channel = **Set eccording to how many peers you have. Default is 0**;
         NameHereInfo.encrypt = false;
   4.Add your peer using the code:
         if (esp_now_add_peer(&peerInfo) != ESP_OK){
         Serial.println("Failed to add peer");
         return;
         }
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
