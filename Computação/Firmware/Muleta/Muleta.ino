#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h> 
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// Variables for test data
int int_value;
float float_value;
bool bool_value = true;
 
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xA9, 0x80, 0xDC};

// Define a data structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;
 
// Create a structured object
struct_message myData;
 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {34, 0, false};
Button button2 = {35, 0, false};

//HX711
const int HX711_dout_1 = 15; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 4; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 16; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 17; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 18; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 19; //mcu > HX711 no 3 sck pin
unsigned long t = 0;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3

byte display_map[10][7] = {
  {0,0,0,0,0,0,1},  //0
  {1,0,0,1,1,1,1},  //1
  {0,0,1,0,0,1,0},  //2
  {0,0,0,0,1,1,0},  //3
  {1,0,0,1,1,0,0},  //4
  {0,1,0,0,1,0,0},  //5
  {0,1,0,0,0,0,0},  //6
  {0,0,0,1,1,1,1},  //7
  {0,0,0,0,0,0,0},  //8
  {0,0,0,0,1,0,0}   //9
};

byte display_pins[] = {27,26,25,33,32,14,13};

int count = 0;

void display_num(int num)
{
  for (int i = 0;i < 7;i++) 
  {
     digitalWrite(display_pins[i], display_map[num][i]); 
  }
}

void TaskBlink( void *pvParameters );

void TaskIMU(void *pvParameters);

void TaskSend(void *pvParameters);

void display_num(int num);

void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  IMU_setup();
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(33, OUTPUT);  
  pinMode(32, OUTPUT); 
  
  float calibrationValue_1 = 0; // calibration value load cell 1
  float calibrationValue_2 = 0; // calibration value load cell 2
  float calibrationValue_3 = 0; // calibration value load cell 3

  LoadCell_1.begin();
  LoadCell_2.begin();  
  LoadCell_3.begin();

  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }
  
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
  Serial.println("Wheatstone bridges startup is complete");
  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBlink
    ,  "TaskBlink"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSend
    ,  "Send"
    ,  1024  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskIMU
    ,  "IMU"
    ,  2048  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0; //increase value to slow down serial print activity
  
    // check for new data/start next conversion:
    if (LoadCell_1.update()) newDataReady = true;
    LoadCell_2.update();
    LoadCell_3.update();
    //get smoothed value from data set
    if ((newDataReady)) {
      if (millis() > t + serialPrintInterval) {
        float a = LoadCell_1.getData();
        float b = LoadCell_2.getData();
        float c = LoadCell_3.getData();
//        Serial.print("Load_cell 1 output val: ");
//        Serial.print(a);
//        Serial.print("    Load_cell 2 output val: ");
//        Serial.println(b);
//        Serial.print("    Load_cell 3 output val: ");
//        Serial.println(c);
        newDataReady = 0;
        t = millis();
      }
    }
  
    // receive command from serial terminal, send 't' to initiate tare operation:
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') {
        LoadCell_1.tareNoDelay();
        LoadCell_2.tareNoDelay();
        LoadCell_3.tareNoDelay();
      }
    }
  
    //check if last tare operation is complete
    if (LoadCell_1.getTareStatus() == true) {
      Serial.println("Tare load cell 1 complete");
    }
    if (LoadCell_2.getTareStatus() == true) {
      Serial.println("Tare load cell 2 complete");
    }
    if (LoadCell_3.getTareStatus() == true) {
      Serial.println("Tare load cell 3 complete");
    }
    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskSend(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
    // Generate a random integer
    int_value = random(1,20);
   
    // Use integer to make a new float
    float_value = 1.3 * int_value;
   
    // Invert the boolean value
    bool_value = !bool_value;
    
    // Format structured data
    strcpy(myData.a, "Welcome to the Workshop!");
    myData.b = int_value;
    myData.c = float_value;
    myData.d = bool_value;
    
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
    if (result == ESP_OK) {
      //Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskIMU(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    IMU_loop();
    vTaskDelay(20);
  }
}
