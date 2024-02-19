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

#define SEC 1000
#define MIN 60000
#define HOUR 3600000

// LEDS
#define NOWIFI 5
#define YESWIFI 2 // This is for onboard led
#define SENDING_DATA 2

// Air Start
#include <MQUnifiedsensor.h>
//Definitions
#define placa "ESP32"
#define Voltage_Resolution 5
#define pin 32
// #define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, "MQ-135");
// Air End

// Tunable Paramaters
int RESET = 0; // pull high if you want to use
int READ = 0; // pull high if you want to use
int SEND = 1; // pull high if you want to use

int deviceID = 0;
int inProcess = 0;
volatile float periord = 100; // default

const int numDataPointsToCollect = 9;
int dataToCollect[numDataPointsToCollect];


bool periordChanged = false;

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
const char* ssid = "BU Guest (unencrypted)";


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
//               Setup                //
////////////////////////////////////////
void setup() {
  // Serial Start
  Serial.begin(115200);
  Serial.println("Starting now");
  delay(1000);
  WiFi.mode(WIFI_STA); //Optional
  // Serial End

  // Sensor Setup TESTING WITHOUT AIR SENSOR
  // dht.begin();

  airInit();
  delay(100);

  // Wifi Setup Start
  Serial.println("Starting Wifi Setup");
  setupWifiandMQTT();
  // Wifi Setup End

  // Free RTOS Tasks Start
  xTaskCreate(SampleSensor, "SampleSensor", 10000, NULL, 1, NULL);         
  xTaskCreate(StateMachine, "StateMachine",10000, NULL, 2, NULL); 

  // Free RTOS Tasks END

  delay(1000);
  // Get Time Start
  timeClient.begin();
  timeClient.setTimeOffset(0);

  char epochTimeString[20];
  getCurrentEpochTimeString(epochTimeString);
  Serial.print("The current time is: ");
  Serial.println(epochTimeString);


  // Get Time Stop

  // LED Indicators 
  pinMode(NOWIFI, OUTPUT);
  pinMode(YESWIFI, OUTPUT);
  pinMode(SENDING_DATA, OUTPUT);

  digitalWrite(NOWIFI, LOW);
  digitalWrite(YESWIFI, LOW);
  digitalWrite(SENDING_DATA, LOW);

  delay(1000);
  while(deviceID == 0){
    SetupTask();
    delay(10000); //Delay for 10 seconds
  }
}

void loop() {
    if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(100);
}

////////////////////////////////////////
//   Air Sensor Connection Functions  //
////////////////////////////////////////
void airInit(){
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
}



////////////////////////////////////////
// WIFI and MQTT Connection Functions //
////////////////////////////////////////
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
    WiFi.begin(ssid);
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
        String client_id = "ID";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connected to broker\n", client_id.c_str());
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
  
  timeClient.forceUpdate();
  delay(100);
  unsigned long epochTime = timeClient.getEpochTime();
  Serial.println(epochTime);
  
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

// need to change soon
void createDataString(char returnString[], int localDeviceID, char epochTimeString[], float dataToSend[]){
  // Convert deviceID to string using casting
  char deviceIDStr[10];
  char holdString[10];

  itoa(localDeviceID, deviceIDStr, 10);

  strcpy(returnString, deviceIDStr);
  strcat(returnString, " ");

  strcat(returnString, epochTimeString);
  strcat(returnString, " ");
    
  // Convert temperature to string using dtostrf
  for (int i = 0; i < 8; i++)
  {
    strcat(returnString, "X");
    char str[10];
    sprintf(str, "%d", i);
    strcat(returnString,str);  // Convert integer to string
    strcat(returnString, ":");

    // Convert dataToSend[i] to string with dynamic width
    char tempString[10];
    int width = snprintf(tempString, sizeof(tempString), "%.2f", dataToSend[i]);
    dtostrf(dataToSend[i], width, 2, holdString);

    strcat(returnString, holdString);
    strcat(returnString, " ");
  }

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

// This is a helper function for parseCallback. This will parse the topic edit the incoming lab and topic action
void parseTopic(char * incomingLab, char * topicAction, char* topic){

    char *separator = strchr(topic, '/');
    int separatorIndex = separator - topic;

    strncpy(incomingLab, topic, separatorIndex);
    strncpy(topicAction, separator, 15);
}


/////////////////////////////
//     Parsing Function    //
/////////////////////////////
void parseCallback(char* topic, byte* payload){
  char incomingLab[25] = "";
  char topicAction[25] = "";

  parseTopic(incomingLab, topicAction, topic);

  Serial.print("Incoming Lab: ");
  Serial.println(incomingLab);
  Serial.print("Incoming topic Action: ");
  Serial.println(topicAction);

  // Check to see if this is the correct Lab
  if(strcmp(incomingLab, labName) == 1){ Serial.println("ERROR, this is the incorrect Lab"); }
  else{
    Serial.println("This is the correct lab, we shall proceed");



    // 1) Incoming Inilization Message
    // nialab/INIT/IN: 0C:DC:7E:CB:6C:D0 2 10 Minute 11111111
    if(strcmp(topicAction, "/INIT/IN") == 0){
      if(false){ // True for Debug
        Serial.println("Incoming /INIT/IN message");
        return;
      }

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

        // Update New Device ID (global)
        recievedDeviceID = strtok(NULL, " ");
        deviceID = atoi(recievedDeviceID);

        // Update New Frequency (global)
        recievedFrequency = strtok(NULL, " ");
        frequency = atoi(recievedFrequency);
      
        // Set New Units (local)
        units = strtok(NULL, " ");
        periord = CalcPeriord(frequency, units);


        // Reset dataToCollect
        for (int i = 0; i < numDataPointsToCollect; i++) {
          dataToCollect[i] = 0;
        }
        char *token = strtok(NULL, " ");; // Define token variable
        while (token != NULL) {
          dataToCollect[atoi(token + 1)] = 1;
          token = strtok(NULL, " "); // Move to the next token
        } 


        Serial.print("Finished Formatting Parameters:");
        Serial.print("The periord is: ");
        Serial.println(periord);

        Serial.print("The dataToCollect is: ");
        for (int i = 0; i < numDataPointsToCollect; i++) {
          Serial.print(dataToCollect[i]);
          Serial.print(" ");
        }
        

        // Change State Parameters
        state = S3;
      }
      else{Serial.println("Incorret Device");}
    }





    // 2) Infoming Configuration Change
    // nialab/CONFIG 1 8 Hour 11110000
    if(strcmp(topicAction, "/CONFIG") == 0){
      if(false){ // True for Debug
        Serial.println("Incoming /CONFIG message");
        return;
      }
      int localDeviceID = atoi(strtok((char*)payload, " "));

      // If this is the correct device, configure the data
      if(localDeviceID == deviceID){
        char* frequency = strtok(NULL, " ");
        char* units = strtok(NULL, " ");
        
        // Reset dataToCollect
        for (int i = 0; i < numDataPointsToCollect; i++) {
          dataToCollect[i] = 0;
        }
        char *token = strtok(NULL, " ");; // Define token variable
        while (token != NULL) {
          dataToCollect[atoi(token + 1)] = 1;
          token = strtok(NULL, " "); // Move to the next token
        } 

        // Update the Periord
        periordChanged = true;
        Serial.print("Periord has been changed from: ");
        Serial.print(periord);
        periord = CalcPeriord(atof(frequency), (units));
        vTaskDelay(30 / portTICK_PERIOD_MS); // Delay for 30ms
        Serial.print(" -> ");
        Serial.println(periord);

        Serial.print("The dataToCollect is: ");
        for (int i = 0; i < numDataPointsToCollect; i++) {
          Serial.print(dataToCollect[i]);
          Serial.print(" ");
        }

      }else{Serial.println("Wrong Device");}
    }
    




    // 3) Incoming Status Check
    // nialab/STATUS/OUT: STATUS
    if(strcmp(topicAction, "/STATUS/OUT") == 0){
      if(false){ // True for Debug
        Serial.println("Incoming /STATUS/OUT message");
        return;
      }

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
    vTaskDelay(3000 / portTICK_PERIOD_MS); // Delay for 3 seconds
  }
}

// Task 2 function
  // 0	DH11	Temperature
  // 1	DH11	Humidity
  // 2	Air Sensor	CO
  // 3	Air Sensor	Alcohol
  // 4	Air Sensor	CO2
  // 5	Air Sensor	Toluene
  // 6	Air Sensor	NH4
  // 7	Air Sensor	Acetone
  // 8 ADD LIGHT SOON
void SampleSensor(void *pvParameters) {
  for (;;) {
    if(state == S3){
      float dataToSend[8];
      // Temperature
      if(dataToCollect[0]){
        MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

        dataToSend[0] = dht.readTemperature();
      }else{
        dataToSend[0] = 0;
      }

      // Humidity
      if(dataToCollect[1] ){
        MQ135.update(); // Update data, the arduino will read the voltage from the analog pin

        dataToSend[1] = dht.readHumidity();
      }else{
        dataToSend[1] = 0;
      }

      // CO
      if(dataToCollect[2]){
        dataToSend[2] = 6969;
        MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
        float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[2] = CO;
      }else{
        dataToSend[2] = 0;
      }

      // Alcohol
      if(dataToCollect[3]){
        dataToSend[3] = 6969;
        MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
        float Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[3] = Alcohol;
      }else{
        dataToSend[3] = 0;
      }

      // CO2
      if(dataToCollect[4]){
        dataToSend[4] = 6969;
        MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
        float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[4] = CO2;
      }else{
        dataToSend[4] = 0;
      }

      // Toluene
      if(dataToCollect[5]){
        dataToSend[5] = 6969;
        MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
        float Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[5] = Toluen;
      }else{
        dataToSend[5] = 0;
      }
      // TESTING

      // NH4
      if(dataToCollect[6]){
        dataToSend[6] = 6969;
        MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
        float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[6] = NH4;
      }else{
        dataToSend[6] = 0;
      }

      // Acetone
      if(dataToCollect[7]){
        dataToSend[7] = 6969;
        MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
        float Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
        dataToSend[7] = Aceton;
      }else{
        dataToSend[7] = 0;
      }

      // Get current Epoch Time
      char epochTimeString[20];
      getCurrentEpochTimeString(epochTimeString);
      
      // Create the message to publish via MQTT
      char message[100];
      createDataString(message, deviceID, epochTimeString, dataToSend);
      
      // Creat the return Topic
      char returnTopic[30] = "";
      strcpy(returnTopic, labName);
      strcat(returnTopic, "/DATA");

      // publish MQTT message
      sendMQTT(returnTopic, message);
      Serial.println(message);

      for (int i = 0; i < periord; i++)
      {
        if(!periordChanged){vTaskDelay(1 / portTICK_PERIOD_MS);}
        else{
          periordChanged = false;
          break;
        }
      }      
    }else{
      vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 2000ms
    }
  }
}


void SetupTask() {
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
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 2000ms
  }
}
