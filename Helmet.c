//HEAD
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "modelt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>

//Variables for MQTT
float rightMqtt, leftMqtt, alarmMqtt, brightnessMqtt, temperatureMqtt, rightPoints, leftPoints, seatMqtt, breakMqtt;
char rightS[5], leftS[5], alarmS[5], brightnessS[5], temperatureS[5], rightPString[5], leftPString[5], seatString[5], breakString[5];

// SSID and password for the server
const char * ssid = "Hotspot";
const char * password = "2810543072";

// IP address of the server
const char * mqtt_server = "192.168.1.8";
int mqtt_port = 1883;

// MAC address of the receiver device
uint8_t MacAddress[] = {0xB4,0xE6,0x2D,0x87,0x69,0xD1};

bool state = false;

WiFiClient espClient;
PubSubClient client(espClient);

//Constants Definition for MPU
#define NUM_SAMPLES 30
#define NUM_AXES 3
#define TRUNCATE 20
#define ACCEL_THRESHOLD 10

//variables definition for MPU
float baseline[NUM_AXES];
float features[NUM_SAMPLES * NUM_AXES];
Adafruit_MPU6050 mpu;
Eloquent::ML::Port::SVM clf;

//If temp%2!=0 means that the route is begin
//If StartRoute=1 means that the route is begin
int StartRoute, temp;

//LDR read
int LDR_Reading;

//LDR voltage
float LDR_Voltage;

//LDR pin
const int LDR_PIN = 39;

//Counts if the head turns left for left flash 
int leftCount;

//Counts if the head turns right for right flash 
int rightCount;

//Flash light count
int flashCount;

//Count the speed for break light
int speedCount;

//Counts if the head turns up for alarm
int AlarmLight;

//Alarm light count
int AlarmCount;

// Readings from mpu, lmu to be sent through esp-now
bool leftFlash;
bool rightFlash;
bool breaks;
bool lights;
bool seat;
bool alarmp;

// Incoming variables from esp-now
bool incseat;

// Variable to store if sending data was successful
String success;

//Counts how many loops we have 
int loopCount;

//Structure for message 
typedef struct struct_packet {
  bool left;
  bool right;
  bool lights;
  bool breaksli;
  bool alarmli;
}
struct_packet;

//Structure for incoming message 
typedef struct incoming_struct_packet {
  bool seatli;
}
incoming_struct_packet;

// Create a message struct to hold lmu-mpu readings
struct_packet message;

// Create amessage struct to hold incoming values
incoming_struct_packet incomingData;

//MPU 6050 setup 
void mpu_setup() {
  Wire.begin();
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

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
  incseat = incomingData.seatli;
}

void setup() {
  // Initilize Serial Monitor
  Serial.begin(115200);
  analogReadResolution(10); //default is 12. Can be set between 9-12.

  //MPU
  mpu_setup(); //MPU 6050 setup 
  mpu_calibrate(); //Mpu callibrate function

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

  //Counts to zero
  loopCount = 0;
  leftCount = 0;
  rightCount = 0;
  speedCount = 0;
  AlarmCount = 0;
  StartRoute = 0;
  temp = 0;

  // Set the device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

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

  //Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  //MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {

  //Every 5 loops, all the counts set to 0
  if (loopCount == 5) {
    cleanCounts();
  }

  // Loop increase
  loopCount++;

  //MPU readings
  float ax, ay, az, tmp;
  mpu_read( & ax, & ay, & az, & tmp); //Mpu reading function

  //Get data to send through MQTT
  rightMqtt = ax;
  leftMqtt = ay;
  alarmMqtt = az;
  temperatureMqtt = tmp;
  if (incseat == true) {
    breakMqtt = 1;
  } else {
    breakMqtt = 0;
  }

  ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
  ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
  az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);

  recordMpu(); //Fill the feature vector with readings from MPU

  checkBreak(); //Checks if the bicycle stops
  checkFlash(); //Checks if the helmet turns right or left for flash
  checkAlarm(); //Checks if the helmet turns up for alarm
  checkLight(); //Checks if is day or night to turn on/off the leds 
  activeFlash(); //Active/Deactivate the flash lights
  activeAlarm(); //Active/Deactivate the alarm lights

  // Set values in struct variables to send with esp-now
  message.left = leftFlash;
  message.right = rightFlash;
  message.lights = lights;
  message.breaksli = breaks;
  message.alarmli = alarmp;

  // Send the message via esp-now
  esp_err_t result = esp_now_send(MacAddress, (uint8_t * ) & message, sizeof(message));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  //WIFI and MQTT 
  if (!client.connected()) {
    reconnect();
  }
  //Convert the floats at strings for client.publish
  dtostrf(rightMqtt, 0, 0, rightS);
  dtostrf(leftMqtt, 0, 0, leftS);
  dtostrf(alarmMqtt, 0, 0, alarmS);
  dtostrf(brightnessMqtt, 0, 0, brightnessS);
  dtostrf(temperatureMqtt, 0, 0, temperatureS);
  dtostrf(rightPoints, 0, 0, rightPString);
  dtostrf(leftPoints, 0, 0, leftPString);
  dtostrf(seatMqtt, 0, 0, seatString);
  dtostrf(breakMqtt, 0, 0, breakString);

  //Check if the route is begin and publish the data
  checkRoute();
  if(StartRoute==1){
  // This loop maintain the connection with the server.
  client.loop();
  //Publish values to server
  client.publish("right/flash", rightS);
  client.publish("left/flash", leftS);
  client.publish("alarm/data", alarmS);
  client.publish("brightness/data", brightnessS);
  client.publish("temperature/data", temperatureS);
  client.publish("leftPoints/data", rightPString);
  client.publish("rightPoints/data", leftPString);
  client.publish("seat/data", seatString);
  client.publish("break/data", breakString);
  }
  delay(900);
}

//Callback function for the WIFI
void callback(char * topic, byte * payload, unsigned int length) {}

//If temp%2!=0 means that the route is begin
//If StartRoute=1 means that the route is begin
void checkRoute() {
  if (clf.predictLabel(features) == "down") {
    temp++;
    if ((temp % 2) != 0) {
      StartRoute = 1;
    } else {
      StartRoute = 0;
    }
  }
}

//Function until esp connect with the server
void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32 client")) {
      Serial.println("Connected");
      client.subscribe("esp32/out");
      client.subscribe("esp32/pwm");
    } else {
      Serial.print(client.state());
      Serial.println("Failed - Try again in 5 seconds");
      delay(5000);
    }
  }
}

//Mpu callibrate function
void mpu_calibrate() {
  Serial.println("The MPU6050 is callibrating");
  float ax, ay, az, tmp;
  for (int i = 0; i < 10; i++) {
    mpu_read( & ax, & ay, & az, & tmp);
    delay(100);
  }
  baseline[0] = ax;
  baseline[1] = ay;
  baseline[2] = az;
  Serial.println("Is ready");
}

//Mpu reading function
void mpu_read(float * ax, float * ay, float * az, float * tmp) {
  int16_t _ax, _ay, _az;
  sensors_event_t a, g, temp;
  mpu.getEvent( & a, & g, & temp);
  * ax = a.acceleration.x;
  * ay = a.acceleration.y;
  * az = a.acceleration.z;
  * tmp = temp.temperature;
}

//Detect if motion is happening
bool motionDetected(float ax, float ay, float az) {
  return (abs(ax) + abs(ay) + abs(az)) > ACCEL_THRESHOLD;
}

//Fill the feature vector with readings from MPU
void recordMpu() {
  float ax, ay, az, tmp;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    mpu_read( & ax, & ay, & az, & tmp);
    ax = constrain(ax - baseline[0], -TRUNCATE, TRUNCATE);
    ay = constrain(ay - baseline[1], -TRUNCATE, TRUNCATE);
    az = constrain(az - baseline[2], -TRUNCATE, TRUNCATE);
    features[i * NUM_AXES + 0] = ax;
    features[i * NUM_AXES + 1] = ay;
    features[i * NUM_AXES + 2] = az;
  }
}

void checkLight() { //Checks if is day or night to turn on/off the leds 
  //LDR readings
  LDR_Reading = analogRead(LDR_PIN);
  LDR_Voltage = ((float) LDR_Reading * 3.3 / 1023);
  brightnessMqtt = LDR_Reading;
  //If is day, turn on the white light leds 
  if (LDR_Reading > 9) {
    digitalWrite(33, LOW);
    digitalWrite(32, LOW);
    lights = false; // Variable to prepare the message for receiver to turn off the lights
  } else { //If is night, turn off the white light leds 
    digitalWrite(33, HIGH);
    digitalWrite(32, HIGH);
    lights = true; // Variable to prepare the message for receiver to turn on the lights
  }
}

void checkBreak() { //Checks if the bicycle stops 
  // Machine Learing
  //If clf.predictLabel(features) == "break" then the bicycle stopped 
  if (clf.predictLabel(features) == "break") {
    speedCount++;
    if (speedCount == 2) { //If speedCount == 2 means that  bicycle stopped over 2 loops
      //MQTT data
      seatMqtt = 1;
      digitalWrite(25, HIGH); // Turn on break light
      breaks = true; // Variable to prepare the message for receiver to turn on the break light
    }
    //If clf.predictLabel(features) == "ongoing" then the bicycle is moving 
  } else if (clf.predictLabel(features) == "ongoing") {
    speedCount--;
    if (speedCount == -2) { //If speedCount == -2 means that  bicycle is active more than 2 loops
      //MQTT data
      seatMqtt = 0;
      digitalWrite(25, LOW); // Turn off break light
      breaks = false; // Variable to prepare the message for receiver to turn on the break light
    }
  }
}

void checkFlash() { //Checks if the helmet turns right or left for flash
  // Machine Learing
  // if clf.predictLabel(features) == "right"then the helmet turn right 
  //or if clf.predictLabel(features) == "left" then the helmet turns left
  if (clf.predictLabel(features) == "right") {
    //MQTT data
    rightPoints++;
    rightCount++;
    if (rightCount == 2) { //If rightCount == 2 means that bicycle will turn right
      //If the right flash is already on and the alarm is off
      if ((rightFlash == false) && (alarmp == false)) {
        flashCount = 1; //The right flash is on forever until the helmet move left 2 times again
        rightFlash = true; // Variable to prepare the message for receiver to turn on the alarm
      } else {
        flashCount = 0; //The right flash turns of
        rightFlash = false; // Variable to prepare the message for receiver to turn on the right flash
      }
    }
	// Machine Learing
    //if clf.predictLabel(features) == "left" then the helmet turns left
  } else if (clf.predictLabel(features) == "left") {
    //MQTT data
    leftPoints++;
    leftCount++;
    if (leftCount == 2) { //If leftCount == 2 means that  bicycle will turn left
      //If the left flash is already on and the alarm is off
      if ((leftFlash == false) && (alarmp == false)) {
        flashCount = 2; //The left flash is on forever until the helmet move left 2 times again
        leftFlash = true; // Variable to prepare the message for receiver to turn on the alarm
      } else {
        flashCount = 0; //The left flash turns of
        leftFlash = false; // Variable to prepare the message for receiver to turn on the left flash
      }
    }
  }
}

void checkAlarm() { //Checks if the helmet turns up for alarm
  // Machine Learing
  //if clf.predictLabel(features) == "up" then the helmet turns up
  if (clf.predictLabel(features) == "up") {
    AlarmCount++;
    if (AlarmCount == 2) { //If rightCount == 2 means that bicycle will turn on the alarm
      flashCount = 0; //Turn off flash function
      if (alarmp == false) { //If the alarm is already on
        AlarmLight = 1; //The alarm flash is on forever until the helmet move up 2 times again
        alarmp = true; // Variable to prepare the message for receiver to turn on the alarm
      } else {
        AlarmLight = 0; //The alarm flash is off forever until the helmet move up 2 times again
        alarmp = false; // Variable to prepare the message for receiver to turn on the alarm
      }
    }
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

void cleanCounts() { //Every 5 loops, all the counts is 0
  loopCount = 0;
  leftCount = 0;
  rightCount = 0;
  speedCount = 0;
  AlarmCount = 0;
}