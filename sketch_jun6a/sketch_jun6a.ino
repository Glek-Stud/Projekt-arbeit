//Example for the use of WiFi with Arduino UNO R4 Wifi
// M. Arndt, March 2025, TH Mittelhessen

/* 
The following parameters have to be set: 
ssid = "*****"; 
password = "*****"; 
devicename 
*/

//Libraries
#include <WiFiS3.h>           // Library for Wifi with ESP32 module
#include <Wire.h>             // Wire Library
#include <Adafruit_INA219.h>  // Library for INA219 Board
#include <PubSubClient.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>  // Library for the LCD Display (optional)

// Wifi Credentials
const char* ssid = "THMnet";                    // Your Wi-Fi SSID
const char* password = "S63rg3pAerUDnUhLtyXg";  // Your Wi-Fi password

// Object definitions
LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD Display 4x20, first parameter is the I2C adress

// MQTT Broker Details
const char* mqtt_server = "test.mosquitto.org";
// MQTT broker (you can replace it with your MQTT broker IP)
const char* mqtt_topic_gen = "cesithmcoil2025/nancy-friedberg/test";
// Testtopic for Listening
const char* mqtt_topic_h = "cesithmcoil2025/nancy-friedberg/h-angle";
// Topic to control Servo 1
const char* mqtt_topic_v = "cesithmcoil2025/nancy-friedberg/v-angle";
// Topic to control Servo 2
const char* mqtt_topic_panel_voltage = "cesithmcoil2025/nancy-friedberg/friedberg/panelvoltage";
// Topic to send values
const char* mqtt_topic_panel_current = "cesithmcoil2025/nancy-friedberg/friedberg/panelcurrent";
// Topic to send values
const char* mqtt_topic_panel_power = "cesithmcoil2025/nancy-friedberg/friedberg/panelpower";
// Topic to send values
const char* mqtt_announce = "cesithmcoil2025/announcement";
// Topic to send Announcements


// Initialization of Global Variables and Constants
int pos = 0;               // Initial servo position in degrees
int i = 0;                 // General counting variable
float shuntvoltage = 0.2;  // INA219 output values
float busvoltage = 5.0;
float current_mA = 2.30;
float loadvoltage = 0;
float power_mW = 0;
char value[10] = "";
char msg_out[20] = "";
char msg_in[20] = "";
int t = 0;



// General counting variable
#define HOSTNAME "esp32s3-61d168"  //Define the hostname of the device for Wifi
#define SERVOPIN_H 9               //Pin for the horizontal servo
#define SERVOPIN_V 10              //Pin for the vertical servo


WiFiClient wifiClient;
Adafruit_INA219 ina219;
PubSubClient client(wifiClient);
Servo myservo_v;
Servo myservo_h;

unsigned long cs_counter;
unsigned long mqp_counter;
unsigned long mqs_counter;
unsigned long lcd_counter;

void reconnect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("WiFi Connected!");
  while (!client.connect(HOSTNAME)) {
    Serial.print(".");
    delay(1000);
  }
  client.subscribe(mqtt_topic_gen);
  client.subscribe(mqtt_topic_h);
  client.subscribe(mqtt_topic_v);
  Serial.println("MQTT Connected!");
  // while (!client.connected())  //Loop until reconnected
  // {
  //   if (client.connect(HOSTNAME)) {
  //     // Publish Announcement
  //     client.publish(mqtt_announce, HOSTNAME);
  //     // Publish Values
  //     dtostrf(loadvoltage, 5, 2, msg_out);
  //     client.publish(mqtt_topic_panel_voltage, msg_out);
  //     dtostrf(current_mA, 5, 2, msg_out);
  //     client.publish(mqtt_topic_panel_current, msg_out);
  //     dtostrf(power_mW, 5, 2, msg_out);
  //     client.publish(mqtt_topic_panel_power, msg_out);
  //     // resubscribe to the topics for the solar tracker
  //     client.subscribe(mqtt_topic_gen);  // Subscribe to the general topics
  //   } else {
  //     Serial.print("Failed");
  //     //Serial.print(client.state());
  //     Serial.println(" Trying again in 5 seconds...");
  //     //delay(5000);
  //   }
  // }
}

void send_mqtt_data(void) {
  if (client.connected()) {
    // Publish the values to the topic
    dtostrf(loadvoltage, 5, 2, msg_out);
    client.publish(mqtt_topic_panel_voltage, msg_out);
    dtostrf(current_mA, 5, 2, msg_out);
    client.publish(mqtt_topic_panel_current, msg_out);
    dtostrf(power_mW, 5, 2, msg_out);
    client.publish(mqtt_topic_panel_power, msg_out);
  }
}

void send_data_lcd(void) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Test Line 1");  // Print on LCD Display
}

// MQTT Callback function to listen to messages from the broker
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


// Setup Function --------------------------------------------------------
void setup() {
  Serial.begin(115200);  //define the baudrate of the serial interface to the
  while (!Serial) {
    delay(1000);  // wait for serial port to connect. Needed for native USB port only
  }

  // Servo Initialization
  myservo_h.attach(SERVOPIN_H);  //attach the servo signal pin. Pin 9-13 can be used.
  myservo_v.attach(SERVOPIN_V);  //attach the servo signal pin. Pin 9-13 can be used.
  myservo_h.write(0);
  myservo_v.write(0);
  // Initialization of servos END


  // Current Sensor Init
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip. No Measurement possible!");
  } else {
    // To use a lower 16V, 400mA range (higher precision on volts and amps):
    ina219.setCalibration_16V_400mA();
    Serial.println("Measuring voltage and current with INA219: Vmax=16V, Imax=400mA");
  }

  // Initialize LCD Display 4x20
  lcd.init();
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();
  // Initialization LCD Display END

  //Initialize the Wifi Network
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not ready! Programm stopped.");
    while (true)
      ;  // Do not continue!
  }

  // Set the hostname of your device
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME);  //define hostname

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("WiFi Connected!");

  //Initialize MQTT
  client.setServer(mqtt_server, 1883);  // Set the MQTT server

  client.setCallback(mqttcallback);  // Set the callback function for incoming messages

  //Check little iffy.
  while (!client.connect(HOSTNAME)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("MQTT Connected!");
  client.subscribe(mqtt_topic_gen);
  client.subscribe(mqtt_topic_h);
  client.subscribe(mqtt_topic_v);
  cs_counter = millis();
  mqp_counter = millis();
  mqs_counter = millis();
  lcd_counter = millis();
  //delay(1000);
  // Initialization of MQTT END
}

// Main Program ----------------------------------------------------------
void loop() {
  client.loop();  // Listen for incoming MQTT messages
  //Keep checking If connected to MQTT Server
  if (!client.connected())  // Reconnect to MQTT if the connection is lost
  {
    reconnect();
  }

  // // Publish Values
  //     dtostrf(loadvoltage,5,2,msg_out);
  //     client.publish(mqtt_topic_panel_voltage, msg_out);
  //     dtostrf(current_mA,5,2,msg_out);
  //     client.publish(mqtt_topic_panel_current, msg_out);
  //     dtostrf(power_mW,5,2,msg_out);
  //     client.publish(mqtt_topic_panel_power, msg_out);
  // client.loop();  // Listen for incoming MQTT messages
  // //MQTT Test END


  // Read Sensor Data every 100ms
  if (millis() - cs_counter > 100) {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    power_mW = loadvoltage * current_mA;
    cs_counter = millis();
  }

  if (millis() - mqp_counter > 1000) {
    mqp_counter = millis();
    send_mqtt_data();
  }

  if (millis() - lcd_counter > 500) {
    lcd_counter = millis();
    send_data_lcd();
  }
}
