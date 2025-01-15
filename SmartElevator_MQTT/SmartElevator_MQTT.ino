#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Stepper.h>
#include <Servo.h>

const char* ssid = "WiFi-LabIoT";
const char* password = "s1jzsjkw5b";

const char* mqtt_server = "192.168.1.115";
const int mqtt_port = 1883;

const char* mqtt_request_topic = "elevator/request";
const char* mqtt_status_topic = "elevator/status";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

int current_floor = 0;
int requested_floor = 0;
int destination_floor = 0;

bool is_moving = false;

Stepper stepper(2048, 8, 10, 9, 11);
Servo doorServo;

const int SERVO_PIN = 13;
const int LED_PINS[] = {2, 3, 4, 5}; // LEDs for floors G, 1, 2

void setup() {
  Serial.begin(9600);

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  reconnect();

  stepper.setSpeed(10);
  doorServo.attach(SERVO_PIN);
  doorServo.write(0); // Door closed

  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  updateFloorIndicator();
}

void loop() {
  client.loop();

  if (current_floor != requested_floor && !is_moving) {
    moveElevator(current_floor, requested_floor);
  } else if (requested_floor == current_floor && destination_floor != current_floor && !is_moving) {
    moveElevator(current_floor, destination_floor);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received on topic ");
  Serial.println(topic);
  Serial.println(message);

  if (String(topic) == mqtt_request_topic) {
    int splitIndex = message.indexOf(',');
    if (splitIndex != -1) {
      requested_floor = message.substring(0, splitIndex).toInt();
      destination_floor = message.substring(splitIndex + 1).toInt();
    }
  }
}

void moveElevator(int start, int end) {
  is_moving = true;
  Serial.print("Moving from floor ");
  Serial.print(start);
  Serial.print(" to floor ");
  Serial.println(end);

  digitalWrite(LED_PINS[start], LOW);

  if (start < end) {
    for (int i = start; i < end; i++) {
      stepper.step(2048); // Move one floor up
    }
  } else {
    for (int i = start; i > end; i--) {
      stepper.step(-2048); // Move one floor down
    }
  }

  current_floor = end;
  updateFloorIndicator();
  openDoor();
  is_moving = false;
}

void openDoor() {
  doorServo.write(90); // Open door
  delay(3000);
  doorServo.write(0); // Close door
  delay(1000);
}

void updateFloorIndicator() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], i == current_floor ? HIGH : LOW);
  }
  client.publish(mqtt_status_topic, String(current_floor).c_str());
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi.");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ElevatorController")) {
      Serial.println("connected");
      client.subscribe(mqtt_request_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}





// #include <WiFiNINA.h>
// #include <PubSubClient.h>
// #include<Stepper.h>
// #include<Servo.h>

// const char* ssid = "WiFi-LabIoT";
// const char* password = "s1jzsjkw5b";

// const char* mqtt_server = "192.168.1.115";
// const int mqtt_port = 1883;

// const char *mqtt_floor_tracker_topic = "floor_tracker";
// const char *mqtt_current_floor_topic = "current_floor";
// const char *mqtt_destination_floor_topic = "destination_floor";

// WiFiClient rev2Client;            // WiFi client
// PubSubClient client(rev2Client);  // MQTT client

// // Global variables
// int request_from = 0;
// int request_to = 0;
// int current_floor = 0;
// int distance = 5000;
// bool going_up, going_down;

// unsigned long previousMillis = 0; // Stores the last time the LED was toggled
// const long interval = 500;

// double spr = 2048;
// Stepper stepper (spr,8,10,9,11);
// Servo servo;

// int SERVO_PIN = 13;
// int LED_PIN0 = 2;
// int LED_PIN1 = 3;
// int LED_PIN2 = 4;
// int LED_PIN3 = 5;

// void setup() {
//   // Start serial port at 9600 bps:
//   Serial.begin(9600);
//   // setup WiFi
//   setupWifi();
//   // MQTT connection:
//   Serial.println();
//   client.setServer(mqtt_server,mqtt_port);
//   client.setCallback(callback);
  
//   if (!client.connected()) {
//     reconnect();
//   }

//   servo.attach(SERVO_PIN);
//   servo.write(0);

//   stepper.setSpeed(10);
//   pinMode(LED_PIN0, OUTPUT);
//   pinMode(LED_PIN1, OUTPUT);
//   pinMode(LED_PIN2, OUTPUT);
//   pinMode(LED_PIN3, OUTPUT);

//   digitalWrite(LED_PIN0, HIGH);
//   digitalWrite(LED_PIN1, LOW);
//   digitalWrite(LED_PIN2, LOW);
//   digitalWrite(LED_PIN3, LOW);
// }

// void loop() {
//   client.loop();
    
//   if((request_from != request_to) && (going_up==going_down)){
//     if(request_from != current_floor){
//       moveElevator(current_floor, request_from);
//     } 
//     openDoor();
//     moveElevator(request_from, request_to);
//     openDoor();
//   }
// }

// void openDoor(){
//   if(current_floor==0){
//       digitalWrite(LED_PIN0, HIGH);
//   } else if(current_floor==1){
//       digitalWrite(LED_PIN1, HIGH);
//   } else if(current_floor==2){
//       digitalWrite(LED_PIN2, HIGH);
//   }
//   delay(2000);
//   servo.write(90);
//   delay(5000);
//   // check if there is something on the door before proceeding
//   servo.write(0);
//   delay(2000);
// }

// void moveElevator(int A, int B){
//   if(A>B){
//     going_up = false;
//     going_down = true;
//   } else{
//     going_up = true;
//     going_down = false;
//   }

//   switch(A){
//     case 0: digitalWrite(LED_PIN0, LOW);
//             if(B==1){
//               //stepper.step(distance);
//               Serial.println("Moving from G-floor to first-floor");
//             } else {
//                Serial.println("Moving from G-floor to 2nd-floor");
//               //stepper.step(distance);
//                delay(1000);
//               Serial.println("Now on 1st  floor");
//               delay(1000);
//               //client.publish(mqtt_floor_tracker_topic, 1);
//               //stepper.step(distance);
//               Serial.println("Arrived on 2nd-floor");
//             }
//             break;
//     case 1: digitalWrite(LED_PIN1, LOW);
//             if(B==2){
//               //stepper.step(distance);
//               Serial.println("Moving from 1st-floor to 2nd-floor");
//             } else {
//               //stepper.step(-distance);
//               Serial.println("Moving from 1st-floor to G-floor");
//             }
//             break;
//     case 2: digitalWrite(LED_PIN2, LOW);
//             if(B==1){
//               //stepper.step(-distance);
//               Serial.println("Moving from 2nd-floor to first-floor");
//             } else {
//               Serial.println("Moving from 2nd-floor to G-floor");
//               //stepper.step(distance);
//               //client.publish(mqtt_floor_tracker_topic, 1);
//               delay(1000);
//               Serial.println("Now on 1st  floor");
//               delay(1000);
//               //stepper.step(distance);
//               Serial.println("Arrive on G-floor");
//             }
//             break;
//     default: break;
//   }

//   current_floor = B;
//   going_up = false;
//   going_down = false;
//   client.publish(mqtt_floor_tracker_topic, current_floor); // Publish the elevator current floor
// }


// void indicate(int floor) {
//   unsigned long currentMillis = millis();

//   // Check if it's time to toggle the LED
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis; // Save the current time

//     // Toggle the LED
//     int ledState;
//     if(floor==0){
//       ledState = digitalRead(LED_PIN0);
//       digitalWrite(LED_PIN0, !ledState);
//     } else if(floor==1){
//       ledState = digitalRead(LED_PIN1);
//       digitalWrite(LED_PIN1, !ledState);
//     } else {
//       ledState = digitalRead(LED_PIN2);
//       digitalWrite(LED_PIN2, !ledState);
//     }
//   }
// }

// // Function to process received messages
// void callback(char* topic, byte* message, unsigned int length) {
//   Serial.print("Message arrived on topic: ");
//   Serial.println(topic);
//   Serial.print(" - Message : ");
//   String messageTemp;
  
//   for (int i = 0; i < length; i++) {
//     Serial.print((char)message[i]);
//     messageTemp += (char)message[i];
//   }
//   Serial.println(messageTemp);

//   if(strcmp(topic, mqtt_current_floor_topic)==0){
//     request_from = toInt(messageTemt);
//   }
//   else if(strcmp(topic, mqtt_destination_floor_topic)==0){
//     request_to = toInt(messageTemt);
//   }
// }


// void setupWifi(){
//   WiFi.begin(ssid, password);
//   delay(2000);

//   // Start WiFi connection:
//   Serial.print("\nConnecting to WiFi...");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println();
//   Serial.println("Connected to the WiFi network.");
//   Serial.print("ARDUINO IP_Address: ");
//   Serial.println(WiFi.localIP());
// }

// void reconnect() {
//   // Loop until we're reconnected
//   Serial.print("Attempting MQTT connection...");
//   while (!client.connected()) {
//     Serial.print("...");
//     // Attempt to connect
//     if (client.connect("REV2")) {
//       Serial.println();
//       Serial.println("Connected to the mosquitto Broker.");
//       // Subscribe
//       client.subscribe(mqtt_current_floor_topic);
//       Serial.print("Subscribed to the topic: ");
//       Serial.println(mqtt_current_floor_topic);
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       delay(2000);
//     }
//   }
// }
  
