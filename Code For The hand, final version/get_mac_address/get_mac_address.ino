#include <WiFi.h>
#include <esp_now.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  delay(3000);
  Serial.println("ADDRESS" + WiFi.macAddress());
}

void loop() {
  // put your main code here, to run repeatedly:

}
