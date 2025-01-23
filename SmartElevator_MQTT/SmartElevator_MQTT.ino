#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Stepper.h>
#include <Servo.h>

const char* ssid = "WiFi-LabIoT";
const char* password = "s1jzsjkw5b";

// const char* ssid = "Galaxy A516A59";
// const char* password = "20111996";

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

int analogValue = 0, step = 2750;

Stepper stepper(2048, 8, 10, 9, 11);
Servo doorServo;

const int SERVO_PIN = 13;
const int LED_PINS[] = {2, 3, 4, 5}; // LEDs for floors G, 1, 2
const int LASER = 12;

void setup() {
  Serial.begin(9600);

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  reconnect();

  stepper.setSpeed(10);
  doorServo.attach(SERVO_PIN);
  doorServo.write(110); // Door closed

  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  digitalWrite(LED_PINS[0], HIGH);

  //pinMode(LASER, OUTPUT);
  //digitalWrite(LASER, HIGH);
  updateFloorIndicator();

  stepper.step(-step);
  
}

void loop() {
  client.loop();

  //Serial.println(analogRead(A0));

  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Wifi Disconnected!");
    setupWiFi();
    reconnect();
  }

  if (current_floor != requested_floor && !is_moving) {
    moveElevator(current_floor, requested_floor);
  } else if (requested_floor == current_floor && destination_floor != current_floor && !is_moving) {
    openDoor();
    moveElevator(current_floor, destination_floor);
    openDoor();
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
      client.publish(mqtt_status_topic, (String(i) + " ↑").c_str());
      stepper.step(step); // Move one floor up
    }
  } else {
    for (int i = start; i > end; i--) {
      client.publish(mqtt_status_topic, (String(i) + "  ↓").c_str());
      stepper.step(-step); // Move one floor down
    }
  }

  current_floor = end;
  updateFloorIndicator();
  is_moving = false;
  if(end==destination_floor){
    requested_floor=end;
  }
}

void openDoor() {
  Serial.println("Opening the door ");
  doorServo.write(80); // Open door
  delay(5000);
  
  while(analogValue < 800){
    Serial.println(analogValue);
    analogValue = analogRead(A0);
  }
  analogValue = 0;
  Serial.println("Closing the door ");
  doorServo.write(110); // Close door
  delay(2000);
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
      delay(2000);
    }
  }
}





// #include <WiFiNINA.h>
// #include <PubSubClient.h>

// const char* ssid = "WiFi-LabIoT";
// const char* password = "s1jzsjkw5b";
// const char* mqtt_server = "192.168.1.116";
// const int mqtt_port = 1883;

// const char* mqtt_request_topic = "elevator/request";
// const char* mqtt_status_topic = "elevator/status";

// WiFiClient wifiClient;
// PubSubClient client(wifiClient);

// const int MAX_REQUESTS = 10; // Maximum number of requests
// int pickupQueue[MAX_REQUESTS];
// int destinationQueue[MAX_REQUESTS];
// int pickupHead = 0, pickupTail = 0;
// int destinationHead = 0, destinationTail = 0;

// int current_floor = 0;
// bool going_up = true; // True if moving up, false if moving down

// void setup() {
//   Serial.begin(9600);
//   setupWiFi();
//   client.setServer(mqtt_server, mqtt_port);
//   client.setCallback(mqttCallback);
//   reconnect();
// }

// void loop() {
//   client.loop();
//   processRequests();
// }

// void mqttCallback(char* topic, byte* payload, unsigned int length) {
//   String message;
//   for (unsigned int i = 0; i < length; i++) {
//     message += (char)payload[i];
//   }

//   if (String(topic) == mqtt_request_topic) {
//     int splitIndex = message.indexOf(',');
//     if (splitIndex != -1) {
//       int request_floor = message.substring(0, splitIndex).toInt();
//       int destination_floor = message.substring(splitIndex + 1).toInt();

//       enqueue(pickupQueue, pickupHead, pickupTail, request_floor);
//       enqueue(destinationQueue, destinationHead, destinationTail, destination_floor);
//     }
//   }
// }

// void processRequests() {
//   if (going_up) {
//     while (!isQueueEmpty(pickupQueue, pickupHead, pickupTail)) {
//       int nextPickup = dequeue(pickupQueue, pickupHead, pickupTail);
//       moveToFloor(nextPickup);
//     }
//     while (!isQueueEmpty(destinationQueue, destinationHead, destinationTail)) {
//       int nextDestination = dequeue(destinationQueue, destinationHead, destinationTail);
//       moveToFloor(nextDestination);
//     }
//     going_up = false; // Switch direction after finishing upward requests
//   } else {
//     while (!isQueueEmpty(pickupQueue, pickupHead, pickupTail)) {
//       int nextPickup = dequeue(pickupQueue, pickupHead, pickupTail);
//       moveToFloor(nextPickup);
//     }
//     while (!isQueueEmpty(destinationQueue, destinationHead, destinationTail)) {
//       int nextDestination = dequeue(destinationQueue, destinationHead, destinationTail);
//       moveToFloor(nextDestination);
//     }
//     going_up = true; // Switch direction after finishing downward requests
//   }
// }

// void moveToFloor(int targetFloor) {
//   Serial.print("Moving to floor ");
//   Serial.println(targetFloor);

//   if (targetFloor > current_floor) {
//     for (int i = current_floor; i <= targetFloor; i++) {
//       delay(1000);
//       current_floor = i;
//       updateFloorStatus();
//     }
//   } else if (targetFloor < current_floor) {
//     for (int i = current_floor; i >= targetFloor; i--) {
//       delay(1000);
//       current_floor = i;
//       updateFloorStatus();
//     }
//   }

//   openDoor();
// }

// void openDoor() {
//   Serial.println("Opening door...");
//   delay(3000);
//   Serial.println("Closing door...");
// }

// void updateFloorStatus() {
//   String status = String(current_floor) + (going_up ? " ↑" : " ↓");
//   client.publish(mqtt_status_topic, status.c_str());
// }

// void enqueue(int* queue, int& head, int& tail, int value) {
//   if ((tail + 1) % MAX_REQUESTS == head) {
//     Serial.println("Queue full, cannot add more requests");
//     return;
//   }
//   queue[tail] = value;
//   tail = (tail + 1) % MAX_REQUESTS;
// }

// int dequeue(int* queue, int& head, int& tail) {
//   if (head == tail) {
//     Serial.println("Queue empty, no requests to process");
//     return -1;
//   }
//   int value = queue[head];
//   head = (head + 1) % MAX_REQUESTS;
//   return value;
// }

// bool isQueueEmpty(int* queue, int head, int tail) {
//   return head == tail;
// }

// void setupWiFi() {
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("Connected to WiFi.");
// }

// void reconnect() {
//   while (!client.connected()) {
//     Serial.print("Connecting to MQTT...");
//     if (client.connect("ElevatorController")) {
//       Serial.println("connected");
//       client.subscribe(mqtt_request_topic);
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       delay(2000);
//     }
//   }
// }

  
