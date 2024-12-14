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

double spr = 2048;
Stepper stepper (spr,8,10,9,11);
Servo servo;

int SERVO_PIN = 13;

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
}

void loop() {
  client.loop();
  
  boolean rc = client.publish(mqtt_floor_tracker_topic, current_floor); // Publish the elevator current floor

  if(request_from != request_to){
    if(request_from != current_floor){
      moveElevator(current_floor, request_from);
    } 
    openDoor();
    moveElevator(request_from, request_to);
    openDoor();
  }
}

void openDoor(){

}

void moveElevator(int A, int B){
  if(A>B){
    going_up = false;
    going_down = true;
  } else{
    going_up = true;
    going_down = false;
  }

  swicth(A){
    case 0: if(B==1){
              stepper.step(distance);
            } else {
              stepper.step(distance*2);
            }
            break;
    case 1: if(B==2){
              stepper.step(distance);
            } else {
              stepper.step(-distance);
            }
            break;
    case 2: if(B==1){
              stepper.step(-distance);
            } else {
              stepper.step(-distance*2);
            }
            break;
    default: break;
  }
  current_floor = B;
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

  if(strcmp(topic, mqtt_current_floor_topic)==0){
    request_from = toInt(messageTemt);
  }
  else if(strcmp(topic, mqtt_destination_floor_topic)==0){
    request_to = toInt(messageTemt);
  }
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
      client.subscribe(topic);
      Serial.print("Subscribed to the topic: ");
      Serial.println(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(2000);
    }
  }
}
  
