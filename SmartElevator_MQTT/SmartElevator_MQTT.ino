#include <WiFiNINA.h>
#include <PubSubClient.h>

const char* ssid = "WiFi-LabIoT";
const char* password = "s1jzsjkw5b";

const char* mqtt_server = "192.168.1.125";
const int mqtt_port = 1883;       // default 1883

const char *mqtt_floor_tracker_topic = "floor_tracker";
const char *mqtt_current_floor_topic = "current_floor";
const char *mqtt_destination_floor_topic = "destination_floor";

WiFiClient rev2Client;            // WiFi client
PubSubClient client(rev2Client);  // MQTT client

void setup() {
  // Start serial port at 9600 bps:
  Serial.begin(9600);

  // setup WiFi
  setupWifi();
  
  // MQTT connection:
  Serial.println();
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);
  long lastMsg = 0;
  char msg[50];
  int value = 0;
  if (!client.connected()) {
    reconnect();
  }
}

void loop() {
  client.loop();

  if (Serial.available()) {    // Verify the presence of a character
    String str = "";
    str = Serial.readString();   // Read a string
    //float prova = 3; //To send a integer
    //char char_array2[6]; //To send a integer
    //dtostrf(prova, 2, 0, char_array2); //To send a integer
    int str_len = str.length() + 1;
    char char_array[str_len];
    str.toCharArray(char_array, str_len); // Convert the string in an array_char
    boolean rc = client.publish(topic,char_array); // Publish the received string
    //boolean rc = client.publish(topic,"3"); // Publish the received string
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

// Function to process received messages
void callback(char* topic, byte* message, unsigned int length) {
  //Serial.print("Message arrived on topic: ");
  //Serial.println(topic);
  Serial.print(" - Message : ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}
  
