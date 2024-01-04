// Temp / Hummidity Sensor
#include <DHT.h>
#include <Arduino.h> // not sure if we need this lol

// Time Start
#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Time END

#define DHTTYPE DHT11 // DHT 11 sensor type
#define DHTPIN 23     // Pin where the DHT11 is connected to (change accordingly)
DHT dht(DHTPIN, DHTTYPE);

// State Information Start
int state = 0;
#define S0 0 // No WIFI
#define S1 1 // Connected to WIFI
#define S2 2 // Has an ID and is sending data
#define S3 3 // Has an ID and is sending data

// Tunable Paramaters
int RESET = 0; // pull high if you want to use
int READ = 0; // pull high if you want to use
int SEND = 1; // pull high if you want to use

int deviceID = 0;
int inProcess = 0;
volatile float periord = 100; // default

bool periordChanged = false;

#define SEC 1000
#define MIN 60000
#define HOUR 3600000

// Buttons Pins
#define NOWIFI 5
#define YESWIFI 18
#define SENDING_DATA 19
#define BUTTON_PIN 22 // Example pin for the button (change accordingly)
// Button Information End

// IP and Mac Start
char formattedIP[16]; // IPv4 addresses can have up to 15 characters (including dots) plus the null terminator
char formattedMac[18]; // Array to store the formatted MAC address (including colons and null terminator)
uint8_t mac[6];
// IP and Mac End

// Button Information Start
SemaphoreHandle_t xSemaphore;
void buttonTask(void *pvParameters);
void buttonISR();
// Button Information End

// MQTT Information Start
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
const char *ssid = "MySpectrumWiFi5B-2G";
const char *password = "urbanfamous507";


// MQTT Broker
const int mqtt_port = 8883;
const char *mqtt_broker = "u88196e4.ala.us-east-1.emqxsl.com";
const char *mqtt_topic = "emqx/esp32";
const char *mqtt_username = "nialab";
const char *mqtt_password = "pi4life";
char *labName = "nialab";

// Load DigiCert Global Root G2, which is used by EMQX Public Broker: broker.emqx.io
const char* ca_cert= \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEqjCCA5KgAwIBAgIQAnmsRYvBskWr+YBTzSybsTANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n" \
"QTAeFw0xNzExMjcxMjQ2MTBaFw0yNzExMjcxMjQ2MTBaMG4xCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xLTArBgNVBAMTJEVuY3J5cHRpb24gRXZlcnl3aGVyZSBEViBUTFMgQ0EgLSBH\n" \
"MTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALPeP6wkab41dyQh6mKc\n" \
"oHqt3jRIxW5MDvf9QyiOR7VfFwK656es0UFiIb74N9pRntzF1UgYzDGu3ppZVMdo\n" \
"lbxhm6dWS9OK/lFehKNT0OYI9aqk6F+U7cA6jxSC+iDBPXwdF4rs3KRyp3aQn6pj\n" \
"pp1yr7IB6Y4zv72Ee/PlZ/6rK6InC6WpK0nPVOYR7n9iDuPe1E4IxUMBH/T33+3h\n" \
"yuH3dvfgiWUOUkjdpMbyxX+XNle5uEIiyBsi4IvbcTCh8ruifCIi5mDXkZrnMT8n\n" \
"wfYCV6v6kDdXkbgGRLKsR4pucbJtbKqIkUGxuZI2t7pfewKRc5nWecvDBZf3+p1M\n" \
"pA8CAwEAAaOCAU8wggFLMB0GA1UdDgQWBBRVdE+yck/1YLpQ0dfmUVyaAYca1zAf\n" \
"BgNVHSMEGDAWgBQD3lA1VtFMu2bwo+IbG8OXsj3RVTAOBgNVHQ8BAf8EBAMCAYYw\n" \
"HQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBIGA1UdEwEB/wQIMAYBAf8C\n" \
"AQAwNAYIKwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdp\n" \
"Y2VydC5jb20wQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQu\n" \
"Y29tL0RpZ2lDZXJ0R2xvYmFsUm9vdENBLmNybDBMBgNVHSAERTBDMDcGCWCGSAGG\n" \
"/WwBAjAqMCgGCCsGAQUFBwIBFhxodHRwczovL3d3dy5kaWdpY2VydC5jb20vQ1BT\n" \
"MAgGBmeBDAECATANBgkqhkiG9w0BAQsFAAOCAQEAK3Gp6/aGq7aBZsxf/oQ+TD/B\n" \
"SwW3AU4ETK+GQf2kFzYZkby5SFrHdPomunx2HBzViUchGoofGgg7gHW0W3MlQAXW\n" \
"M0r5LUvStcr82QDWYNPaUy4taCQmyaJ+VB+6wxHstSigOlSNF2a6vg4rgexixeiV\n" \
"4YSB03Yqp2t3TeZHM9ESfkus74nQyW7pRGezj+TC44xCagCQQOzzNmzEAP2SnCrJ\n" \
"sNE2DpRVMnL8J6xBRdjmOsC3N6cQuKuRXbzByVBjCqAA8t1L0I+9wXJerLPyErjy\n" \
"rMKWaBFLmfK/AHNF4ZihwPGOc7w6UHczBZXH5RFzJNnww+WnKuTPI0HfnVH8lg==\n" \
"-----END CERTIFICATE-----\n";

WiFiClientSecure espClient;
PubSubClient client(espClient);
// MQTT Information End


////////////////////////////////////////
// WIFI and MQTT Connection Functions //
////////////////////////////////////////

// Not called as of dec 18th
void subscribeToTopic(char* topic) {
  client.subscribe(topic);
  Serial.print("Subscribed to topic: ");
  Serial.println(topic);
}

void initMacIP(){
  // IP and Mac Initilization Start
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; ++i) {
      if (i < 5) {
          sprintf(formattedMac + i * 3, "%02X:", mac[i]);
      } else {
          sprintf(formattedMac + i * 3, "%02X", mac[i]);
      }
  }

  // Get and print the local IP address
  IPAddress localIP = WiFi.localIP();
  sprintf(formattedIP, "%d.%d.%d.%d", localIP[0], localIP[1], localIP[2], localIP[3]);
}

void setupWifiandMQTT() {
    // Set software serial baud to 115200;
    Serial.begin(115200);
    // connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");

    initMacIP();

    // set root ca cert
    espClient.setCACert(ca_cert);
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);

    while (!client.connected()) {
        String client_id = "toChangeSoon";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Labsensors MQTT Broker Connected");
        } else {
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.print(client.state());
            Serial.println("Retrying in 5 seconds.");
            delay(5000);
        }
    }
    char combinedString[20];

    Serial.println("Subscribing to topics");

    strcpy(combinedString, labName);
    strcat(combinedString, "/INIT/IN");         
    subscribeToTopic(combinedString);

    strcpy(combinedString, labName);
    strcat(combinedString, "/CONFIG");    
    subscribeToTopic(combinedString);

    strcpy(combinedString, labName);
    strcat(combinedString, "/STATUS/OUT");    
    subscribeToTopic(combinedString);

    Serial.println("Done subscribing to topics");


    Serial.println(WiFi.localIP());
    state = S1;
}

// Updated Dec 18th
void reconnect() {
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT broker...");
    String client_id = "esp8266-client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Reconnected to MQTT broker.");
    } else {
        Serial.print("Failed to reconnect to MQTT broker, rc=");
        Serial.print(client.state());
        Serial.println("Retrying in 5 seconds.");
        delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  payload[length] = '\0';  // Ensure null termination
  parseCallback(topic,payload);
}



//////////////////////
// Helper Functions //
//////////////////////
void getCurrentEpochTimeString(char* result) {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  // Convert unsigned long to String
  String epochTimeString = String(epochTime);

  // Copy the contents of the String to the character array
  epochTimeString.toCharArray(result, epochTimeString.length() + 1);
}

int CalcPeriord(int frequency,char* units) {
  float localPeriord = -1;
  if(strcmp(units, "Second") == 0){
    localPeriord = SEC / frequency;
  }else if(strcmp(units, "Minute") == 0){
    localPeriord = MIN / frequency;
  }else if(strcmp(units, "Hour") == 0){
    localPeriord = HOUR / frequency;
  }
  return localPeriord;
}

void createDataString(char returnString[], int localDeviceID,char epochTimeString[], float temperature, float humidity){
  // Convert deviceID to string using casting
  char deviceIDStr[10];
  itoa(localDeviceID, deviceIDStr, 10);

  strcpy(returnString, deviceIDStr);
  strcat(returnString, " ");

  strcat(returnString, epochTimeString);
  strcat(returnString, " ");
    
  // Convert temperature to string using dtostrf
  char tempStr[10];
  dtostrf(temperature, 5, 2, tempStr); // 5 is minimum width, 2 is precision
  strcat(returnString, tempStr);
  strcat(returnString, " ");

  // Convert humidity to string using dtostrf
  char humStr[10];
  dtostrf(humidity, 5, 2, humStr); // 5 is minimum width, 2 is precision

  strcat(returnString, humStr);

  Serial.print("MQTT String to be sent: ");
  Serial.println(returnString);
}

///////////////////////////
// MQTT Sender Functions //
///////////////////////////
void sendMQTT(char* topic, char* message) {
  Serial.println("Sending an MQTT message");
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (client.connected()) {
    client.publish(topic, message);
  }
}

void setup() {
  // Serial Start
  Serial.begin(115200);
  Serial.println("This has started");
  // Serial End

  // Sensor Setup
  dht.begin();

  // Wifi Setup Start
  Serial.println("Starting Wifi Setup");
  setupWifiandMQTT();
  // Wifi Setup End

  // Free RTOS Tasks Start
  xTaskCreate(SampleSensor, "SampleSensor", 10000, NULL, 1, NULL);         
  xTaskCreate(StateMachine, "StateMachine",10000, NULL, 2, NULL); 

  // Free RTOS Tasks END

  // Button Interupt Code Start
  xSemaphore = xSemaphoreCreateBinary();
  if (xSemaphore == NULL) {
    Serial.println("Semaphore creation failed!");
    while (1);
  }
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  xTaskCreate(buttonTask, "Button Task", 10000, NULL, 1, NULL);
  // Button Interupt Code END

  // Get Time Start
  timeClient.begin();
  timeClient.setTimeOffset(0);
  // Get Time Stop

  // LED Indicators 
  pinMode(NOWIFI, OUTPUT);
  pinMode(YESWIFI, OUTPUT);
  pinMode(SENDING_DATA, OUTPUT);

  digitalWrite(NOWIFI, LOW);
  digitalWrite(YESWIFI, LOW);
  digitalWrite(SENDING_DATA, LOW);
}

void loop() {
    if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(100);
}


// This is a helper function for parseCallback. This will parse the topic edit the incoming lab and topic action
void parseTopic(char * incomingLab, char * topicAction, char* topic){

    char *separator = strchr(topic, '/');
    int separatorIndex = separator - topic;

    strncpy(incomingLab, topic, separatorIndex);
    strncpy(topicAction, separator, 15);
}

void parseCallback(char* topic, byte* payload){
  Serial.println("This is a work in progress");
  char incomingLab[25] = "";
  char topicAction[25] = "";

  parseTopic(incomingLab, topicAction, topic);

  Serial.print("Incoming Lab: ");
  Serial.println(incomingLab);
  Serial.print("Incoming topic Action: ");
  Serial.println(topicAction);

  if(strcmp(incomingLab, labName) == 1){
    Serial.println("ERROR, this is the incorrect Lab");
  }
  else{
    Serial.println("This is the correct lab, we shall proceed");

    if(strcmp(topicAction, "/INIT/IN") == 0){
      Serial.println("Setup Detected");
      char* recievedMacAddress;
      char* recievedDeviceID;
      char* recievedFrequency;

      int frequency = 0;
      char* units;

      recievedMacAddress = strtok((char*)payload, " ");
      Serial.print("I think the macAddress is: ");
      Serial.println(recievedMacAddress);

      if (strcmp(recievedMacAddress, formattedMac) == 0){
        Serial.println("Formatting New Parameters");

        recievedDeviceID = strtok(NULL, " ");
        deviceID = atoi(recievedDeviceID);

        recievedFrequency = strtok(NULL, " ");
        frequency = atoi(recievedFrequency);
      
        units = strtok(NULL, " ");

        periord = CalcPeriord(frequency, units);

        Serial.print("Finished Formatting Parameters, the periord is: ");
        Serial.println(periord);

        // Change State Parameters
        state = S3;
      }
      else{Serial.println("Incorret Device");}
    }

    if(strcmp(topicAction, "/CONFIG") == 0){
      int localDeviceID = atoi(strtok((char*)payload, " "));

      // If this is the correct device, configure the data
      if(localDeviceID == deviceID){
        char* frequency = strtok(NULL, " ");
        char* units = strtok(NULL, " ");

        periordChanged = true;

        Serial.print("Periord has been changed from: ");
        Serial.print(periord);

        periord = CalcPeriord(atof(frequency), (units));

        vTaskDelay(30 / portTICK_PERIOD_MS); // Delay for 30ms
        
        Serial.print(" -> ");
        Serial.println(periord);
      }else{Serial.println("Wrong Device");}
    }
    
    if(strcmp(topicAction, "/STATUS/OUT") == 0){
      char stringDeviceID[10];  // Adjust the size based on the expected length of the integer
      itoa(deviceID, stringDeviceID, 10);

      randomSeed(analogRead(0));
      float randomFloat = map(random(10000), 0, 10000, 0.0, 3000.0);
      vTaskDelay(randomFloat / portTICK_PERIOD_MS); // Delay for random

      char returnString[30];
      strcpy(returnString, stringDeviceID);

      Serial.print("Returning Status: ");
      Serial.println(returnString);

      char returnTopic[20] = "";

      strcpy(returnTopic, labName);
      strcat(returnTopic, "/STATUS/IN");

      sendMQTT(returnTopic, returnString);
    }
  }




  // check to see if the lab is the correct lab
  // check to see the type of topic

  // if it is init in

  // if it is config

  // if it is status out
}


// Task 1 function
void StateMachine(void *pvParameters) {
  for (;;) {
    switch (state)
    {
    // NO WIFI
    case S0:
      digitalWrite(NOWIFI, HIGH);
      digitalWrite(YESWIFI, LOW);
      digitalWrite(SENDING_DATA, LOW);
      break;
    
    // Wifi Connected and Waiting
    case S1:
      digitalWrite(NOWIFI, LOW);
      digitalWrite(YESWIFI, HIGH);
      digitalWrite(SENDING_DATA, LOW);
      break;

    // Sending ID to request database
    case S2:
      digitalWrite(NOWIFI, LOW);
      digitalWrite(YESWIFI, HIGH);
      digitalWrite(SENDING_DATA, LOW);
      break;

    // Device ID intercepted, Sending data Now
    case S3:
      digitalWrite(NOWIFI, LOW);
      digitalWrite(YESWIFI, LOW);
      digitalWrite(SENDING_DATA, HIGH);
      break;
    
    default:
      break;
    }
    Serial.print("The State is:");
    Serial.println(state);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1000ms
  }
}

// Task 2 function
void SampleSensor(void *pvParameters) {
  for (;;) {
    if(state == S3){
      float humidity = dht.readHumidity();
      float temperature = dht.readTemperature();

      if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
      }
      
      char epochTimeString[20];
      getCurrentEpochTimeString(epochTimeString);
      
      char message[50];
      createDataString(message, deviceID, epochTimeString ,temperature, humidity);

      
      char returnTopic[20] = "";

      strcpy(returnTopic, labName);
      strcat(returnTopic, "/DATA");

      sendMQTT(returnTopic, message);
      //Serial.println(message);

      for (int i = 0; i < periord; i++)
      {
        if(!periordChanged){
          vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else{
          Serial.println("The periord has changed");
          periordChanged = false;
          break;
        }
        
      }      
    }else{
      vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 2000ms
    }
  }
}


void buttonTask(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      Serial.println("Button pressed!");

      // If you want to reset the device
      if(RESET){
        Serial.println("Reseting Device ID");
        deviceID = 0;
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 1000ms
      }

      // If you want to read the ID
      if(READ){
        Serial.println("Reading Device ID");
        Serial.print("The read device ID is: ");
        Serial.println(deviceID);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 1000ms
      }

      // Make sure you are in setup mode, and in send mode, and are not already in the process of sending a message
      if((state == S1) && SEND && (deviceID == 0)){ // add inprocess back
        // Create String
        char result[45];
        char returnWithLab[20];

        char* topic = "/INIT/OUT";

        strcpy(returnWithLab, labName);
        strcat(returnWithLab, topic);

        strcpy(result, formattedIP);
        strcat(result, " ");
        strcat(result, formattedMac);

        Serial.println(returnWithLab);
        Serial.println(result);
        sendMQTT(returnWithLab, result);
        inProcess = 1;
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 1000ms
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 2000ms
  }
}

void buttonISR() {
  // Notify the task that the button was pressed
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

  // If there was a task that was waiting for the semaphore,
  // request a context switch to the waiting task
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}
