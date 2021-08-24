//Saddle
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>

const int pinPressure = 35; // Pin for pressure sensor
const int readingsAvarage = 16; //Avarage readings

//Calibration/Mapping variables for the pressure sensor
int pressureMax = 0;
int pressureMin = 4095;
int pressureValue = 0;

// MAC address of the receiver device
uint8_t MacAddress[] = {0xB4,0xE6,0x2D,0x85,0xB5,0x49};

//SSID of the router
constexpr char SSID[] = "Hotspot";

// Readings from mpu, lmu to be sent through esp-now
bool leftFlash;
bool rightFlash;
bool breaks;
bool lights;
bool seat;
bool alarmp;

// Incoming variables from esp-now
bool incleftFlash;
bool incrightFlash;
bool incbreaks;
bool inclights;
bool incalarm;

//Flash light count
int flashCount;

//Counts if the head turns up for alarm
int AlarmCount;

//Alarm light count
int AlarmLight;

// Variable to store if sending data was successful
String success;

//Structure for incoming message 
typedef struct incoming_struct_packet {
  bool left;
  bool right;
  bool lights;
  bool breaksli;
  bool alarmli;
} incoming_struct_packet;

//Structure for message 
typedef struct struct_packet {
  bool seatli;
} struct_packet;

// Create a message struct to hold lmu-mpu readings
struct_packet message;

// Create amessage struct to hold incoming values
incoming_struct_packet incomingData;

// Callback when data is sent with esp-now
void dataSent(const uint8_t * mac_addr, esp_now_send_status_t status) {
  Serial.println("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

// Callback when data is received with esp-now
void DataReceive(const uint8_t * mac,
  const uint8_t * newData, int len) {
  memcpy( & incomingData, newData, sizeof(incomingData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incleftFlash = incomingData.left;
  incrightFlash = incomingData.right;
  inclights = incomingData.lights;
  incbreaks = incomingData.breaksli;
  incalarm = incomingData.alarmli;
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

void setup() {
  // Initilize Serial Monitor
  Serial.begin(115200);

  //Left-LEDS
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  //Right-LEDS
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);

  //Middle-LED
  pinMode(25, OUTPUT);

  //Light-LEDS
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  //Pressure sensor pin
  pinMode(pinPressure, INPUT);

  // Calibrate the first 4 seconds (Pressure Sensor)
  while (millis() < 4000) {

    pressureValue = analogRead(pinPressure);

    //Save the maximum pressure sensor value to pressureMax
    if (pressureValue > pressureMax) {
      pressureMax = pressureValue;
      //Save the minimum pressure sensor value to pressureMin
    } else if (pressureValue < pressureMin) {
      pressureMin = pressureValue;
    }
  }

  // Set the device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

int32_t channel = getWiFiChannel(SSID);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //When ESP-NOW is initilize successfully,i register to Send CB and get the status of trasnmitted packet
  esp_now_register_send_cb(dataSent);

  // Register the peer to esp-now
  esp_now_peer_info_t peerDetails;
  memcpy(peerDetails.peer_addr, MacAddress, 6);
  peerDetails.channel = 0;
  peerDetails.encrypt = false;

  // Add the peer to esp-now
  if (esp_now_add_peer( & peerDetails) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback func. when data is received
  esp_now_register_recv_cb(DataReceive);
}

void loop() {

  checkFlash(); //Checks if the helmet turns right or left for flash
  checkLight(); //Checks if is day or night to turn on/off the leds 
  checkBreak(); //Checks if the bicycle stops
  checkAlarm(); //Checks if the helmet turns up for alarm
  activeFlash(); //Active/Deactivate the flash lights
  activeAlarm(); //Active/Deactivate the alarm lights
  checkSeat(); //Checks if the cyclist is sitting in the bicycle saddle

  // Set values in struct variables to send with esp-now
  message.seatli = seat;

  // Send the message via esp-now
  esp_err_t result = esp_now_send(MacAddress, (uint8_t * ) & message, sizeof(message));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  delay(500);
}

void checkLight() { //Checks if is day or night to turn on/off the leds from the message(from the helmet) 
  //If is day, turn on the white light leds 
  if (inclights == false) {
    digitalWrite(33, LOW);
    digitalWrite(32, LOW);
  } else { //If is night, turn off the white light leds 
    digitalWrite(33, HIGH);
    digitalWrite(32, HIGH);
  }
}

void checkBreak() { //Checks if the bicycle stops   
  // if AcX < 1000 then the bicycle stopped
  if (incbreaks == true) { //If breaks is true, then the bicycle stopped else is moving
    digitalWrite(25, HIGH); // Turn on break light
  } else {
    digitalWrite(25, LOW); // Turn off break light
  }
}

void checkFlash() { //Checks if the helmet turns right or left for flash
  if (incrightFlash == true) { //If incleftFlash is true, turn on the right flash leds 
    flashCount = 1; //The right flash is on
  } else if (incleftFlash == true) { //If incleftFlash is true, turn on the left flash leds 
    flashCount = 2; //The left flash is on
  } else {
    flashCount = 0; //Turn off all the flash lights
  }
}

void checkAlarm() { //Checks if the helmet turns up for alarm
  if (incalarm == true) { //If incalarm is true, turn on the alarm leds 
    AlarmLight = 1; //The alarm flash is on forever until the helmet move up 2 times again
    flashCount = 0; //Turn off the flash lights function
  } else { //If incleftFlash is true, turn on the left flash leds 
    AlarmLight = 0; //The alarm flash is off forever until the helmet move up 2 times again
  }
}

void checkSeat() { //Checks if the cyclist is sitting in the bicycle saddle
  pressureValue = analogRead(pinPressure); //Read the pressure sensor output
  if (pressureValue != 0) { //if pressureValue != 0 means that no-one sit on the saddle
    seat = false;
  } else { // //if pressureValue != 0 means that someone sit on the saddle
    seat = true;
  }
}

void activeFlash() { //Active/Deactivate the flash lights
  //if flashCount == 1  means that bicycle turns on the right flash otherwise 
  //if flashCount == 2 means that bicycle turns on the left flash
  //if flashCount == 0  bicycle turns off all the flash lights
  if (flashCount == 1) {
    digitalWrite(27, HIGH);
    digitalWrite(26, HIGH);
    delay(1000);
    digitalWrite(27, LOW);
    digitalWrite(26, LOW);
  } else if (flashCount == 2) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
  } else {
    digitalWrite(27, LOW);
    digitalWrite(26, LOW);
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    rightFlash = false;
    leftFlash = false;
  }
}

void activeAlarm() { //Active/Deactivate the alarm lights
  //if AlarmLight > 0 means that bicycle turns on the alarm lights
  //if AlarmLight == 0  bicycle turns off all the alarm lights
  if (AlarmLight == 1) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(27, HIGH);
    digitalWrite(26, HIGH);
    // digitalWrite(25, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(27, LOW);
    digitalWrite(26, LOW);
    // digitalWrite(25, LOW);
  } else {
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(27, LOW);
    digitalWrite(26, LOW);
    // digitalWrite(25, LOW);
    alarmp = false;
  }
}
