/***Glove ESP-NOW Board***/
#include <WiFi.h>
#include <esp_now.h>


#define RXD2 16
#define TXD2 17



String success;
int SystemControl = 0;
uint8_t Incoming[12];
uint8_t SendArray[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//uint8_t ServoMaxPosition[] = {80, 50, 80, 50, 46, 22, 70};                         //Maximum angle the servo can run to, read from NX simulation
uint8_t ServoMaxPosition[] = {76, 55, 76, 55, 37, 70, 80};                         //Maximum angle the servo can run to, read from NX simulation
uint8_t PercentageFlex[7] = {0, 0, 0, 0, 0, 0, 0};
hw_timer_t *Timer50 = NULL;     //50 millisecond tick

uint8_t HandBoardAddress[] = {0x58, 0xBF, 0x25, 0x37, 0x3A, 0x28};

void OnDataTransmission(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(millis());
  Serial.print("\tLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //Serial.println(millis());
}

void OnDataReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Incoming, incomingData, sizeof(Incoming));
}

void SendData() {
  esp_now_send(HandBoardAddress, (uint8_t *) &SendArray, sizeof(SendArray));

}

void IRAM_ATTR OnTimer50() {
  SendData();
}

void GetVals(){
  
  while (SystemControl != 4) {
    if (Serial2.available() > 0) {
      SystemControl = Serial2.read();
      Serial.println(SystemControl);
    }
    if (SystemControl == 1) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (SystemControl == 2) {
      digitalWrite(LED_BUILTIN, LOW);
    }
    if (SystemControl == 3) {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
pinMode(LED_BUILTIN, OUTPUT);
GetVals();

  //Register callback function to get transmission status
  esp_now_register_send_cb(OnDataTransmission);
  //Register callback function to be called when data is recieved
  esp_now_register_recv_cb(OnDataReceive);

  //Register Board Two as Peer
  esp_now_peer_info_t HandBoardInfo;
  memset(&HandBoardInfo, 0, sizeof(HandBoardInfo));
  memcpy(HandBoardInfo.peer_addr, HandBoardAddress, 6);
  HandBoardInfo.channel = 0;
  HandBoardInfo.encrypt = false;

  //Add Board Two as Peer
  if (esp_now_add_peer(&HandBoardInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Timer50 = timerBegin(2, 80, true);                    //attach timer50 to 1st timer of 4, 1 tick = 1us
  timerAttachInterrupt(Timer50, &OnTimer50, true);     //attach interrupt function to timer50
  timerAlarmWrite(Timer50, 100000, true);               //Set timer50 alarm to call interrupt every 50ms
  timerAlarmEnable(Timer50);                            //enable timer50


}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0) {
    Serial2.readBytes(PercentageFlex, 7);
    //Serial.println(PercentageFlex[2]);
    for(int i = 0; i<7; i++){
      SendArray[i] = ((PercentageFlex[i]*ServoMaxPosition[i])/100);
      //Serial.println(SendArray[0]);
    }

    
  }

}
