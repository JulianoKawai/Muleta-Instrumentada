#include <esp_now.h>
#include <WiFi.h>
 
// Define a data structure
typedef struct struct_message {
  float lc1;
  float lc2;
  float lc3;
  float omgx;
  float omgy;
  float omgz;
}struct_message;
 
// Create a structured object
struct_message myData;
 
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("Load Cell 1: ");
  Serial.println(myData.lc1);
  Serial.print("Load Cell 2: ");
  Serial.println(myData.lc2);
  Serial.print("Load Cell 3: ");
  Serial.println(myData.lc3);
  Serial.print("Omega X: ");
  Serial.println(myData.omgx);
  Serial.print("Omega Y: ");
  Serial.println(myData.omgy);
  Serial.print("Omega Z: ");
  Serial.println(myData.omgz);
  Serial.println();
}
 
void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
 
}
