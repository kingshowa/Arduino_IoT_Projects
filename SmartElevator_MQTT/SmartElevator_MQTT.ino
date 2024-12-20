#include <WiFiNINA.h>
#include <PubSubClient.h>
#include<Stepper.h>
#include<Servo.h>

const char* ssid = "WiFi-LabIoT";
const char* password = "s1jzsjkw5b";

const char* mqtt_server = "192.168.1.125";
const int mqtt_port = 1883;       // default 1883

const char *mqtt_floor_tracker_topic = "floor_tracker";
const char *mqtt_current_floor_topic = "current_floor";
const char *mqtt_destination_floor_topic = "destination_floor";

WiFiClient rev2Client;            // WiFi client
PubSubClient client(rev2Client);  // MQTT client

// Global variables
int request_from = 0;
int request_to = 0;
int current_floor = 0;
int distance = 5000;
bool going_up, going_down;

unsigned long previousMillis = 0; // Stores the last time the LED was toggled
const long interval = 500;

double spr = 2048;
Stepper stepper (spr,8,10,9,11);
Servo servo;

int SERVO_PIN = 13;
int LED_PIN0 = 2;
int LED_PIN1 = 3;
int LED_PIN2 = 4;

void setup() {
  // Start serial port at 9600 bps:
  Serial.begin(9600);
  // setup WiFi
  setupWifi();
  // MQTT connection:
  Serial.println();
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);
  
  if (!client.connected()) {
    reconnect();
  }

  servo.attach(SERVO_PIN);
  servo.write(0);

  stepper.setSpeed(10);
  pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

  digitalWrite(LED_PIN0, HIGH);
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, LOW);
}

void loop() {
  client.loop();
    
  if((request_from != request_to) && (going_up==going_down)){
    if(request_from != current_floor){
      moveElevator(current_floor, request_from);
    } 
    openDoor();
    moveElevator(request_from, request_to);
    openDoor();
  }
}

void openDoor(){
  if(current_floor==0){
      digitalWrite(LED_PIN0, HIGH);
  } else if(current_floor==1){
      digitalWrite(LED_PIN1, HIGH);
  } else if(current_floor==2){
      digitalWrite(LED_PIN2, HIGH);
  }
  delay(2000);
  servo.write(90);
  delay(5000);
  // check if there is something on the door before proceeding
  servo.write(0);
  delay(2000);
}

void moveElevator(int A, int B){
  if(A>B){
    going_up = false;
    going_down = true;
  } else{
    going_up = true;
    going_down = false;
  }

  switch(A){
    case 0: digitalWrite(LED_PIN0, LOW);
            if(B==1){
              //stepper.step(distance);
              Serial.println("Moving from G-floor to first-floor");
            } else {
               Serial.println("Moving from G-floor to 2nd-floor");
              //stepper.step(distance);
               delay(1000);
              Serial.println("Now on 1st  floor");
              delay(1000);
              //client.publish(mqtt_floor_tracker_topic, 1);
              //stepper.step(distance);
              Serial.println("Arrived on 2nd-floor");
            }
            break;
    case 1: digitalWrite(LED_PIN1, LOW);
            if(B==2){
              //stepper.step(distance);
              Serial.println("Moving from 1st-floor to 2nd-floor");
            } else {
              //stepper.step(-distance);
              Serial.println("Moving from 1st-floor to G-floor");
            }
            break;
    case 2: digitalWrite(LED_PIN2, LOW);
            if(B==1){
              //stepper.step(-distance);
              Serial.println("Moving from 2nd-floor to first-floor");
            } else {
              Serial.println("Moving from 2nd-floor to G-floor");
              //stepper.step(distance);
              //client.publish(mqtt_floor_tracker_topic, 1);
              delay(1000);
              Serial.println("Now on 1st  floor");
              delay(1000);
              //stepper.step(distance);
              Serial.println("Arrive on G-floor");
            }
            break;
    default: break;
  }

  current_floor = B;
  going_up = false;
  going_down = false;
  client.publish(mqtt_floor_tracker_topic, current_floor); // Publish the elevator current floor
}


void indicate(int floor) {
  unsigned long currentMillis = millis();

  // Check if it's time to toggle the LED
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the current time

    // Toggle the LED
    int ledState;
    if(floor==0){
      ledState = digitalRead(LED_PIN0);
      digitalWrite(LED_PIN0, !ledState);
    } else if(floor==1){
      ledState = digitalRead(LED_PIN1);
      digitalWrite(LED_PIN1, !ledState);
    } else {
      ledState = digitalRead(LED_PIN2);
      digitalWrite(LED_PIN2, !ledState);
    }
  }
}

// Function to process received messages
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print(" - Message : ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // if(strcmp(topic, mqtt_current_floor_topic)==0){
  //   request_from = toInt(messageTemt);
  // }
  // else if(strcmp(topic, mqtt_destination_floor_topic)==0){
  //   request_to = toInt(messageTemt);
  // }
}


void setupWifi(){
  WiFi.begin(ssid, password);
  delay(2000);

  // Start WiFi connection:
  Serial.print("\nConnecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to the WiFi network.");
  Serial.print("ARDUINO IP_Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  while (!client.connected()) {
    Serial.print("...");
    // Attempt to connect
    if (client.connect("REV2")) {
      Serial.println();
      Serial.println("Connected to the mosquitto Broker.");
      // Subscribe
      //client.subscribe(topic);
      Serial.print("Subscribed to the topic: ");
      //Serial.println(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(2000);
    }
  }
}
  
