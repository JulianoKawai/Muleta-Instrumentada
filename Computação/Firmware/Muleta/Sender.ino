// Variables for test data
int int_value;
float float_value;
bool bool_value = true;

// MAC Address of responder - edit as requiredcy
uint8_t broadcastAddress[] = { 0x24, 0xD7, 0xEB, 0x11, 0xC7, 0x88 };

// Define a data structure
//typedef struct struct_message {
//  char a[32];
//  int b;
//  float c;
//  bool d;
//} struct_message;

typedef struct struct_message {
  float lc1;
  float lc2;
  float lc3;
  float omgx;
  float omgy;
  float omgz;
} struct_message;

// Create a structured object
struct struct_message infos = { .lc1 = 0.0, .lc2 = 0.0, .lc3 = 0.0, .omgx = 0.0, .omgy = 0.0, .omgz = 0.0 };

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void ESP_NOW_Setup() {
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void Send_Message() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&infos, sizeof(infos));

  if (result == ESP_OK) {
    //Serial.println("Sending confirmed");
  } else {
    Serial.println("Sending error");
  }
}