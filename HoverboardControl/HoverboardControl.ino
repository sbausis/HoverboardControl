
#include <HoverboardAPI.h>
#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

/*
 *  WiFi Settings
 */
WiFiClient wifiClient;

IPAddress ip(172, 24, 1, 2);
IPAddress gateway(172, 24, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* ssid = "HoverboardAP";
const char* password = "1234567890";

void wifi_setup() {
  //if (WiFi.status() != WL_CONNECTED) return;
  
  WiFi.config(ip, gateway, subnet);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to");
    Serial.println(ssid);
    while(1) delay(500);
  }
  
  Serial.print("Ready! ");
  Serial.println(WiFi.localIP());
  
}

/*
 *  MQTT Settings
 */
const char* mqtt_server = "172.24.1.1";
const char* mqtt_user = "admin";
const char* mqtt_pass = "1234567890";

PubSubClient mqttClient(mqtt_server, 1883, wifiClient);

void mqtt_callback(char* topic, byte* payload, unsigned int length);

void mqtt_setup() {
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);
}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqtt_clientId = "HoverboardControl-";
    mqtt_clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(mqtt_clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("hoverboard_steer");
      mqttClient.subscribe("hoverboard_throttle");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
      break;
    }
  }
}

/*
 *  Serial Settings
 */
#define HOVERBOARD_SERIAL_RX_PIN 4
#define HOVERBOARD_SERIAL_TX_PIN 5
SoftwareSerial hoverboardSerial(HOVERBOARD_SERIAL_RX_PIN, HOVERBOARD_SERIAL_TX_PIN); // RX, TX
int hoverboardSend(unsigned char *data, int len) {
  return (int) hoverboardSerial.write(data,len);
}

const char* hoverboard_control_version = "0.1";
void serial_setup() {
  Serial.begin(115200);   // Serial for debugging
  Serial.print("Hoverboard Control v");
  Serial.print(hoverboard_control_version);
  Serial.println(" starting up ...");
  
  hoverboardSerial.begin(115200);  // Connection to hoverboard
}

/*
 *  Hoverboard Settings
 */
HoverboardAPI hoverboard = HoverboardAPI(hoverboardSend);

//bool hoverboard_connected = false;

// Variable for PWM
PROTOCOL_PWM_DATA PWMData;
PROTOCOL_BUZZER_DATA buzzer;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //Serial.println((char*)payload);
  /*
  snprintf (msg, length + 1, "%s", (char*)payload);
  double throttle2 = atof((char*)payload) - 1000.0;
  double throttle = (atof(msg) - 1000.0);
  Serial.println(throttle2);
  Serial.println(throttle);
  */

  double throttle = (atof((char*)payload) - 1000.0);
  //Serial.println(throttle);
  
  PWMData.pwm[0] = throttle;
  PWMData.pwm[1] = -throttle;
}

int hoverboardReceive() {
  // Read and Process Incoming data
  int i=0;
  while(hoverboardSerial.available() && i++ < 1024) { // read maximum 1024 bytes at once.
    hoverboard.protocolPush( hoverboardSerial.read() );
  }
  return i;
}

/*unsigned long hoverboard_last_message = 0;
void hall_cb( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
  //Serial.println("received Hall Data");
  hoverboard_last_message = millis();
  return;
}*/

/*void elec_cb( PROTOCOL_STAT *s, PARAMSTAT *param, uint8_t fn_type, unsigned char *content, int len ) {
  //Serial.println("received Electrical Data");
  
}*/

void hoverboard_setup() {
  
  // Request Hall sensor data every 100ms
  //hoverboard.updateParamHandler(HoverboardAPI::Codes::sensHall, hall_cb);
  hoverboard.scheduleRead(HoverboardAPI::Codes::sensHall, -1, 100);
  
  // Request Electrical Information every second
  //hoverboard.updateParamHandler(hoverboard.Codes::sensElectrical, elec_cb);
  hoverboard.scheduleRead(hoverboard.Codes::sensElectrical, -1, 1000);

  buzzer = {4, 0, 100};
  
  // Register Variable and send Buzzer values periodically
  //hoverboard.updateParamHandler(HoverboardAPI::Codes::setBuzzer, buzzer_cb);
  hoverboard.updateParamVariable(HoverboardAPI::Codes::setBuzzer, &buzzer, sizeof(buzzer));
  hoverboard.scheduleTransmission(HoverboardAPI::Codes::setBuzzer, -1, 200);
  
  hoverboard.sendBuzzer(4, 0, 100, PROTOCOL_SOM_NOACK);
  
  // Send PWM to 10% duty cycle for wheel1, 15% duty cycle for wheel2. Wheel2 is running backwards.
  PWMData.pwm[0] = 0.0;
  PWMData.pwm[1] = 0.0;

  // Register Variable and send PWM values periodically
  //hoverboard.updateParamHandler(HoverboardAPI::Codes::setPointPWM, pwm_cb);
  hoverboard.updateParamVariable(HoverboardAPI::Codes::setPointPWM, &PWMData, sizeof(PWMData));
  hoverboard.scheduleTransmission(HoverboardAPI::Codes::setPointPWM, -1, 30);

  // Set maxium PWM to 400, Minimum to -400 and threshold to 30. Require ACK (Message will be resend when not Acknowledged)
  hoverboard.sendPWMData(0, 0, 1000, -1000, 10, PROTOCOL_SOM_ACK);
  
}

void serial_print_hoverboard_data() {
  /*if (hoverboard_connected == true)*/ {
    // Print Speed on debug Serial.
    Serial.print("Motor Speed: ");
    Serial.print(hoverboard.getSpeed0_mms());
    Serial.print(" / ");
    Serial.print(hoverboard.getSpeed1_mms());
    Serial.println(" mm/s (wheel0 / wheel1)");
    
    Serial.print("Motor 0 Amperage: ");
    Serial.print(hoverboard.getMotorAmps(0));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorAmpsAvg(0));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorAmpsAvgAcc(0));
    Serial.println(" Ampere (Amps / Avg / Acc)");
    
    Serial.print("Motor 1 Amperage: ");
    Serial.print(hoverboard.getMotorAmps(1));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorAmpsAvg(1));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorAmpsAvgAcc(1));
    Serial.println(" Ampere (Amps / Avg / Acc)");
    /*
    Serial.print("Motor 0 PWM: ");
    Serial.print(hoverboard.getMotorPwmLimiter(0));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorPwmRequested(0));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorPwmActual(0));
    Serial.println(" Ampere (Limit / Req / Act)");
    
    Serial.print("Motor 1 PWM: ");
    Serial.print(hoverboard.getMotorPwmLimiter(1));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorPwmRequested(1));
    Serial.print(" / ");
    Serial.print(hoverboard.getMotorPwmActual(1));
    Serial.println(" Ampere (Limit / Req / Act)");
    */
    Serial.print("DC Current Limit: ");
    Serial.print(hoverboard.getDCCurrentLimit());
    Serial.print("A (");
    Serial.print(hoverboard.getDCCurrentLimitAdc());
    Serial.println(" ADC)");
    
    // Print battery Voltage on debug Serial.
    Serial.print("Battery Voltage: ");
    Serial.print(hoverboard.getBatteryVoltage());
    Serial.print("V (");
    Serial.print(hoverboard.getBatteryVoltageRaw());
    Serial.println(" ADC)");
    
    Serial.print("Board Temperature: ");
    Serial.print(hoverboard.getBoardTemperature());
    Serial.print("C (");
    Serial.print(hoverboard.getBoardTemperatureFiltered());
    Serial.print(" filtered) (");
    Serial.print(hoverboard.getBoardTemperatureRaw());
    Serial.println(" ADC)");
    
    Serial.print("Is Charging: ");
    Serial.println(hoverboard.isCharging());
  }
}

char msg[50];

double speed_m0 = 0;
double speed_m1 = 0;
int is_charging = false;
float batt_voltage = 0;
float board_temp = 0;

void mqtt_publish_hoverboard_data() {
  /*if (hoverboard_connected == true)*/ {
        double m0 = hoverboard.getSpeed0_mms();
    /*if (m0 != speed_m0)*/ {
      speed_m0 = m0;
      snprintf (msg, 50, "%f", speed_m0);
      //Serial.print("hoverboard_speed_m0: ");
      //Serial.println(msg);
      mqttClient.publish("hoverboard_speed_m0", msg);
    }
    
    double m1 = hoverboard.getSpeed1_mms();
    /*if (m1 != speed_m1)*/ {
      speed_m1 = m1;
      snprintf (msg, 50, "%f", speed_m1);
      //Serial.print("hoverboard_speed_m1: ");
      //Serial.println(msg);
      mqttClient.publish("hoverboard_speed_m1", msg);
    }
    
    int chg = hoverboard.isCharging();
    /*if (chg != is_charging)*/ {
      is_charging = chg;
      snprintf (msg, 50, "%d", is_charging);
      //Serial.print("hoverboard_charging: ");
      //Serial.println(msg);
      mqttClient.publish("hoverboard_charging", msg);
    }
    
    float battv = hoverboard.getBatteryVoltage();
    /*if (battv != batt_voltage)*/ {
      batt_voltage = battv;
      snprintf (msg, 50, "%.2f", battv);
      //Serial.print("hoverboard_voltage: ");
      //Serial.println(msg);
      mqttClient.publish("hoverboard_voltage", msg);
    }
    
    float btemp = hoverboard.getBoardTemperature();
    /*if (btemp != board_temp)*/ {
      board_temp = btemp;
      snprintf (msg, 50, "%.2f", btemp);
      //Serial.print("hoverboard_temperature: ");
      //Serial.println(msg);
      mqttClient.publish("hoverboard_temperature", msg);
    }
  }
}

void setup() {
  serial_setup();
  wifi_setup();
  mqtt_setup();
  hoverboard_setup();
}

//const long wifi_interval = 5000;
//const long mqtt_interval = 1000;
//const long serial_interval = 300;
const long hoverboard_interval = 300;
unsigned long hoverboard_last = 0;

unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

void loop() {

  //hoverboard_connected = (millis() - hoverboard_last_message < 1000);
  
  // Delay loop() for 300 ms use the time to send and receive UART protocol every 100 microseconds
  // This way messages which are received can be processed and scheduled messages can be sent.
  hoverboard_last = millis();
  while (millis() < hoverboard_last + hoverboard_interval){
    hoverboardReceive();
    // Send Messages from queue, re-send failed messages and process scheduled transmissions
    hoverboard.protocolTick();
    //delayMicroseconds(100);
    
  /*if (hoverboard_connected != true) {
    Serial.println("Hoverboard DISCONNECTED !!!");
  } else {
    //Serial.println("Hoverboard CONNECTED ...");
  }*/
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi DISCONNECTED !!!");
  } else {
    //Serial.println("WiFi CONNECTED ...");
  }
  
  if (!mqttClient.connected()) {
    Serial.println("MQTT DISCONNECTED !!!");
    mqtt_reconnect();
  } else {
    //Serial.println("MQTT CONNECTED ...");
    mqttClient.loop();
  }
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    serial_print_hoverboard_data();
    mqtt_publish_hoverboard_data();
  }
  
  if (Serial.available()) {
    uint8_t c = Serial.read();
    while(Serial.available() && Serial.read());;
    if (c == 'f') {
      hoverboard.sendBuzzer(4, 0, 100, PROTOCOL_SOM_NOACK);
      Serial.println("FORWARD");
      PWMData.pwm[0] = 100.0;
      PWMData.pwm[1] = -100.0;
    } else if (c == 'b') {
      hoverboard.sendBuzzer(4, 0, 100, PROTOCOL_SOM_NOACK);
      Serial.println("BACKWARD");
      PWMData.pwm[0] = -100.0;
      PWMData.pwm[1] = 100.0;
    } else {
      hoverboard.sendBuzzer(4, 0, 100, PROTOCOL_SOM_NOACK);
      Serial.println("STOPP");
      PWMData.pwm[0] = 0.0;
      PWMData.pwm[1] = 0.0;
    }
  }
  }
}
