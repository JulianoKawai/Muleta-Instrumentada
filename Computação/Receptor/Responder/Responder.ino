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
  sendToPC(&myData.lc1, &myData.lc2, &myData.lc3, &myData.omgx, &myData.omgy, &myData.omgz);
  //Serial.println(myData.lc1);
  //Serial.println(myData.lc2);
  //Serial.println(myData.lc3);
  //Serial.println(myData.omgx);
  //Serial.println(myData.omgy);
  //Serial.println(myData.omgz);
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

void sendToPC(float* data1, float* data2, float* data3,float* data4, float* data5, float* data6)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte buf[24] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                 byteData6[0], byteData6[1], byteData6[2], byteData6[3]};
  Serial.write(buf, 24);
}