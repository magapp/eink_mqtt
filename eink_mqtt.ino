#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic#include <PubSubClient.h>
#include <PubSubClient.h>
#include <time.h>           // for time() ctime()
#include <WiFiUdp.h>
#include <StateMachine.h>   // https://github.com/jrullan/StateMachine
#include <ArduinoOTA.h>
#include <neotimer.h>

// Use HSPI for EPD (and e.g VSPI for SD), e.g. with Waveshare ESP32 Driver Board
#define USE_HSPI_FOR_EPD
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include "GxEPD2_display_selection_new_style.h"

/* Fonts */
#include "OrangeJuice_26_pt_7b.h"
#include "Viral26pt7b.h"
#include "meteocons26pt7b.h"   // weather symbols


/* Defines */
#define PIN_BUTTON 0
#define OTA_PORT 3232


/* Static declarations */
SPIClass hspi(HSPI);

WiFiClient wifiClient;

WiFiManager wifiManager;
WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT server");
WiFiManagerParameter custom_mqtt_username("mqtt_username", "MQTT username");
WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT password");
WiFiManagerParameter custom_mqtt_server_port("mqtt_server_port", "MQTT server port");
WiFiManagerParameter custom_ntp_server("ntp_server", "NTP server");

time_t now;  // this are the seconds since Epoch (1970) - UTC
tm tm;           

PubSubClient mqttClient(wifiClient);

Neotimer timeoutTimer = Neotimer(); 

/* EEPROM map */
struct eeprom_storage {
  char wifi_ssid[32];
  char wifi_password[32];
  char ntp_server[30];

  char mqtt_server[30];
  char mqtt_username[30];
  char mqtt_password[30];
  uint16_t mqtt_server_port;
} eeprom;

struct import_data_storage {
  int8_t utomhus_temperature;
  float spabad_temperature;
  float bastu_temperature;

  bool aurora;
  float charge_1_kwh;
  float charge_2_kwh;
  float charge_1_current_kw;
  float charge_2_current_kw;

  int8_t windspeed;
  int16_t snowdepth;
  char current_weather[30];

  int8_t forecast_9_temperature;
  char forecast_9_weather[30];
  int8_t forecast_12_temperature;
  char forecast_12_weather[30];
  int8_t forecast_15_temperature;
  char forecast_15_weather[30];

  char rank_1_name[10];
  uint16_t rank_1_value;
  char rank_2_name[10];
  uint16_t rank_2_value;
  char rank_3_name[10];
  uint16_t rank_3_value;
  char rank_4_name[10];
  uint16_t rank_4_value;
  char rank_5_name[10];
  uint16_t rank_5_value;
  char rank_6_name[10];
  uint16_t rank_7_value;

  int8_t override_hour;  // used for debugging
} import_data;

/* Declare state machine */
StateMachine machine = StateMachine();
State* State_SetupDisplay = machine.addState(&setupDisplay);
State* State_CheckButtonOnBoot = machine.addState([](){});
State* State_SetupWifi = machine.addState(&setupWifi);
State* State_Configuration = machine.addState(&startConfigurationManager);
State* State_SetupOta = machine.addState(&setupOta);
State* State_SetupNtp = machine.addState(&setupNtp);
State* State_SetupMqtt = machine.addState(&setupMqtt);
State* State_Idle = machine.addState(&idle);
State* State_DayForecast = machine.addState(&display_day_forecast);
State* State_SpaTemperature = machine.addState(&display_spa_temperature);
State* State_Ranking = machine.addState(&display_ranks);
State* State_Charging = machine.addState(&display_charge);
State* State_UpdateDisplay = machine.addState([](){});
State* State_Reset = machine.addState(&reset);
State* State_EepromSave = machine.addState(&saveEeprom);
State* State_Error = machine.addState(&error);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  EEPROM.begin(sizeof(eeprom));
  EEPROM.get(0, eeprom);

  import_data.utomhus_temperature = -127;
  import_data.spabad_temperature = -127;
  import_data.bastu_temperature = -127;
  import_data.windspeed = -127;
  import_data.snowdepth = -127;
  import_data.forecast_9_temperature = -127;
  import_data.forecast_12_temperature = -127;
  import_data.forecast_15_temperature = -127;

  import_data.aurora = false;
  import_data.charge_1_kwh = -127;
  import_data.charge_2_kwh = -127;
  import_data.charge_1_current_kw = -127;
  import_data.charge_2_current_kw = -127;

  import_data.override_hour = -127;

  /* Setup transitions for states */
  State_SetupDisplay->addTransition(&waitTimeout, State_CheckButtonOnBoot);

  State_CheckButtonOnBoot->addTransition(&checkButtonPressed, State_Configuration);
  State_CheckButtonOnBoot->addTransition(&waitTimeout, State_SetupWifi);

  State_SetupWifi->addTransition(&checkSetupWifi, State_SetupOta);
  State_SetupWifi->addTransition(&waitTimeout, State_Error);

  State_SetupOta->addTransition(&checkTrue, State_SetupNtp);

  State_SetupNtp->addTransition(&checkSetupNtp, State_SetupMqtt);
  State_SetupNtp->addTransition(&waitTimeout, State_Error);

  State_SetupMqtt->addTransition(&checkSetupMqtt, State_Idle);
  State_SetupMqtt->addTransition(&waitTimeout, State_Error);

  /* Choose current state depnding time of day */
  State_Idle->addTransition(&mqttNotConnected, State_SetupMqtt);
  State_Idle->addTransition(&wifiNotConnected, State_SetupWifi);

  State_Idle->addTransition(&showCharging, State_Charging);
  State_Idle->addTransition(&showForecast, State_DayForecast);
  State_Idle->addTransition(&showSpaTemperature, State_SpaTemperature);
  State_Idle->addTransition(&showRank, State_Ranking);
  State_Idle->addTransition(&showBlank, State_Reset);

  /* Leave state when not time */
  State_Charging->addTransition([](){
    return !showCharging(); 
  }, State_Idle);

  State_DayForecast->addTransition([](){
    return !showForecast(); 
  }, State_Idle);

  State_Ranking->addTransition([](){
    return !showRank(); 
  }, State_Idle);

  State_SpaTemperature->addTransition([](){
    return !showSpaTemperature(); 
  }, State_Idle);

  State_Reset->addTransition([](){
    return !showBlank(); 
  }, State_Idle);

  /* Jump back to state (force executeOnce()-function to rerun) */
  State_UpdateDisplay->addTransition(&showCharging, State_Charging);
  State_UpdateDisplay->addTransition(&showForecast, State_DayForecast);
  State_UpdateDisplay->addTransition(&showSpaTemperature, State_SpaTemperature);
  State_UpdateDisplay->addTransition(&showRank, State_Ranking);
  State_UpdateDisplay->addTransition(&showBlank, State_Reset);
}

void loop() {
  ArduinoOTA.handle();
  machine.run();
  mqttClient.loop();

  time(&now);                         // read the current time
  localtime_r(&now, &tm);             // update the structure tm with the current time

  delay(5);
}

void screenLogg(String str) {
  display.setFont(&Viral26pt7b);
  display.println(str);
  display.display();
}

void error(void) {
  /*  Error state, restart */
  Serial.println("Error state, restarting");
  display.setRotation(0);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, 0);
  display.println();
  screenLogg("Error state, restarting");
  ESP.restart();
}

/* Setup display */
void setupDisplay(void) {
  if (machine.executeOnce) {
    hspi.begin(13, 12, 14, 15); // remap hspi for EPD (swap pins)
    display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    display.init(115200, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse
  
    display.setRotation(0);
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&Viral26pt7b);
    display.setCursor(0, 0);
    display.println();
  
    display.println("Press BOOT-button for setup");
    display.display();
  
    timeoutTimer.set(1000);
    timeoutTimer.start();
  }
}

/* Setup wifi */
void setupWifi(void) {
  if (machine.executeOnce) {
    Serial.println("Wifi: SSID " + String(eeprom.wifi_ssid));
    Serial.println("Wifi: Pwd "+ String(eeprom.wifi_password));
    Serial.println("Wifi: Connecting");
    screenLogg("Wifi: " + String(eeprom.wifi_ssid));

    WiFi.begin(eeprom.wifi_ssid, eeprom.wifi_password);

    timeoutTimer.set(30000);
    timeoutTimer.start();
  }
}

void startConfigurationManager(void) {
  wifiManager.setTitle("E-ink");

  custom_mqtt_server.setValue(eeprom.mqtt_server, sizeof(eeprom.mqtt_server));
  wifiManager.addParameter(&custom_mqtt_server);
  custom_mqtt_username.setValue(eeprom.mqtt_username, sizeof(eeprom.mqtt_username));
  wifiManager.addParameter(&custom_mqtt_username);
  custom_mqtt_password.setValue(eeprom.mqtt_password, sizeof(eeprom.mqtt_password));
  wifiManager.addParameter(&custom_mqtt_password);
  custom_mqtt_server_port.setValue(String(eeprom.mqtt_server_port).c_str(), 5);
  wifiManager.addParameter(&custom_mqtt_server_port);
  custom_ntp_server.setValue(eeprom.ntp_server, sizeof(eeprom.ntp_server));
  wifiManager.addParameter(&custom_ntp_server);

  wifiManager.setDebugOutput(true, WM_DEBUG_NOTIFY );

  Serial.println("CONFIG: starting portal");
  screenLogg("Starting configuration portal");

  WiFi.begin();
  String ap_name = "configure-";
  ap_name += String(WiFi.macAddress());
  ap_name.replace(":", "");

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.startConfigPortal(ap_name.c_str())) {
    Serial.println("SETUP: failed");
    screenLogg("Failed");
    machine.transitionTo(State_Error);
  }
}

/* Setup NTP client */
void setupNtp(void) {
  if (machine.executeOnce) {
    Serial.println("NTP: init");
    screenLogg("NTP: " + String(eeprom.ntp_server));
  }

  configTime(0, 0, eeprom.ntp_server);                      // 0, 0 because we will use TZ in the next line
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);            // Set environment variable with your time zone
  tzset();
  time(&now);                         // read the current time
  localtime_r(&now, &tm);             // update the structure tm with the current time
}

bool checkSetupNtp(void) {
  screenLogg(showTime().c_str());
  return true;
}

/* ---- Setup OTA ---- */
void setupOta(void) {
  if (machine.executeOnce) {
    Serial.println("OTA: Setup");
    ArduinoOTA.setPort(OTA_PORT);
    //ArduinoOTA.setHostname("myesp32");
    //ArduinoOTA.setPassword("");

    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      display_init_full();
      screenLogg("OTA: Start");
      })
      .onEnd([]() {
        screenLogg("OTA: End");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        if (progress % 1000 == 0) {
          Serial.println("OTA: Progress: " + String(progress) + "/" + String(total));
        }
      })
      .onError([](ota_error_t error) {
        Serial.printf("OTA: Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Serial.println("OTA: Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
          Serial.println("OTA: Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
          Serial.println("OTA: Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("OTA: Receive Failed");
        } else if (error == OTA_END_ERROR) {
          Serial.println("OTA: End Failed");
        }
    });
    ArduinoOTA.begin();
  }
}

/* Setup mqtt */
void setupMqtt(void) {
  if (machine.executeOnce) {
    Serial.println("MQTT: " + String(eeprom.mqtt_server));
    screenLogg("MQTT: " + String(eeprom.mqtt_server));
    mqttClient.setServer(eeprom.mqtt_server, eeprom.mqtt_server_port);
    mqttClient.setCallback(mqttCallback);

    String client_id = "eink-";
    client_id += String(WiFi.macAddress());
    client_id.replace(":", "");

    if (!mqttClient.connect(client_id.c_str(), eeprom.mqtt_username, eeprom.mqtt_password)) {
      Serial.println("MQTT error: " + String(mqttClient.state()));
      screenLogg("MQTT error: " + String(mqttClient.state()));
    }
    timeoutTimer.set(5000);
    timeoutTimer.start();
  }
}

bool mqttNotConnected(void) {
  if (mqttClient.connected()) {
    return false;
  }
  Serial.println("MQTT: Lost connection");
  return true;
}

bool checkSetupMqtt(void) {
  if (mqttClient.connected()) {
    Serial.println("MQTT: Connected");
    mqttClient.subscribe("homeassistant/export/bjornrike/#");
    mqttClient.subscribe(getTopic("command/#").c_str());
    mqttClient.publish(getTopic("online").c_str(), "1");
    return true;
  }
  return false;
}

/* Generate topic for MQTT */
String getTopic(String t) {
  String topic = "eink/";
  topic += String(WiFi.macAddress());
  topic.replace(":", "");
  topic = topic + "/" + t;
  return topic;
}

/* ----- Transitions ----- */

bool checkSetupWifi(void) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wifi: " + WiFi.localIP().toString());
    screenLogg("Wifi: " + WiFi.localIP().toString());
    return true;
  }
  return false;
}

bool wifiNotConnected(void) {
  if (WiFi.status() == WL_CONNECTED) {
    return false;
  }
  Serial.print("Wifi: Not conencted");
  return true;
}

bool checkTrue(void) {
  return true;
}

bool waitTimeout(void) {
  return timeoutTimer.done();
}

bool checkButtonPressed(void) {
  return digitalRead(PIN_BUTTON) == 0;  // 0 = pressed
}
bool checkButtonNotPressed(void) {
  return digitalRead(PIN_BUTTON) == 1;  // 1 = not pressed
}

bool showCharging(void) {
  if (import_data.charge_1_current_kw > 0) {
    return true;
  }
  if (import_data.charge_2_current_kw > 0) {
    return true;
  }
  return false;
}

bool showForecast(void) {

  /* 06:00 - 15:00 and 21:00 - 02:00 */
  switch(getHour()) {
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:

    case 21:
    case 22:
    case 23:
    case 0:
    case 1:
    case 2:
      return true;
    default:
      break;
  }
  return false;
}

bool showSpaTemperature(void) {

  if (showRank() == true) {
    return false;
  }

  switch(getHour()) {
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
      return true;
    default:
      break;
  }
  return false;
}

bool showRank(void) {
  if (import_data.rank_1_value > 0) {
    switch(getHour()) {
      case 16:
      case 18:
      case 19:
        return true;
      default:
        break;
    }   
  }
  return false;
}

bool showBlank(void) {

  /* 02:00 - 05:00 */
  switch(getHour()) {
    case 2:
    case 3:
    case 4:
    case 5:
      return true;
    default:
      break;
  }
  return false;
}

/* ----- Callbacks ----- */
void saveConfigCallback () {
  Serial.println("CONFIG: Save config");
  screenLogg("Save config");

  //Serial.println("CONFIG: ssid " + wifiManager.getWiFiSSID());
  //screenLogg("CONFIG: ssid " + wifiManager.getWiFiSSID());
  memset(eeprom.wifi_ssid, 0, sizeof(eeprom.wifi_ssid));
  strncpy(eeprom.wifi_ssid, wifiManager.getWiFiSSID().c_str(), sizeof(eeprom.wifi_ssid));

  //Serial.println("CONFIG: pwd " + wifiManager.getWiFiPass());
  //screenLogg("CONFIG: pwd " + wifiManager.getWiFiPass());
  memset(eeprom.wifi_password, 0, sizeof(eeprom.wifi_password));
  strncpy(eeprom.wifi_password, wifiManager.getWiFiPass().c_str(), sizeof(eeprom.wifi_password));

  //Serial.println("CONFIG: MQTT server " + String(custom_mqtt_server.getValue()));
  //screenLogg("CONFIG: MQTT server " + String(custom_mqtt_server.getValue()));
  memset(eeprom.mqtt_server, 0, sizeof(eeprom.mqtt_server));
  strncpy(eeprom.mqtt_server, custom_mqtt_server.getValue(), sizeof(eeprom.mqtt_server));

  //Serial.println("CONFIG: MQTT username " + String(custom_mqtt_username.getValue()));
  //screenLogg("CONFIG: MQTT username " + String(custom_mqtt_username.getValue()));
  memset(eeprom.mqtt_username, 0, sizeof(eeprom.mqtt_username));
  strncpy(eeprom.mqtt_username, custom_mqtt_username.getValue(), sizeof(eeprom.mqtt_username));

  //Serial.println("CONFIG: MQTT password " + String(custom_mqtt_password.getValue()));
  //screenLogg("CONFIG: MQTT password " + String(custom_mqtt_password.getValue()));
  memset(eeprom.mqtt_password, 0, sizeof(eeprom.mqtt_password));
  strncpy(eeprom.mqtt_password, custom_mqtt_password.getValue(), sizeof(eeprom.mqtt_password));

  //Serial.println("CONFIG: MQTT port " + String(custom_mqtt_server_port.getValue()));
  //screenLogg("CONFIG: MQTT port " + String(custom_mqtt_server_port.getValue()));
  eeprom.mqtt_server_port = String(custom_mqtt_server_port.getValue()).toInt();

  //Serial.println("CONFIG: NTP server " + String(custom_ntp_server.getValue()));
  strncpy(eeprom.ntp_server, custom_ntp_server.getValue(), sizeof(eeprom.ntp_server));

  machine.transitionTo(State_EepromSave);
}

bool isState(State *s) {
  if (machine.currentState == s->index) {
    return true;
  }
  return false;
}

void mqttCallback(char* t, byte* payload, unsigned int length) {

  /* mqttc.publish("homeassistant/export/bjornrike/rank_5_name", "Magnus")
     mqttc.publish("homeassistant/export/bjornrike/utomhus_temperatur", "-10")
     mqttc.publish("homeassistant/export/bjornrike/spabad_temperatur", "10")
     mqttc.publish("homeassistant/export/bjornrike/bastu_temperatur", "63")
     mqttc.publish("homeassistant/export/bjornrike/override_hour", "10")
  */
  String buff = "";
  String topic = t;

  for (size_t i = 0; i < length; i++) {
    buff.concat((char)payload[i]);
  }

  Serial.println("MQTT: Topic: " + topic);
  Serial.println("MQTT: Received: " + buff);
    display.println(machine.currentState);
    display.println(State_Idle->index);

  if (String(topic).endsWith("utomhus_temperatur")) {
    if ((import_data.utomhus_temperature != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.utomhus_temperature = buff.toInt();
  }

  if (String(topic).endsWith("spabad_temperatur")) {
    if ((import_data.spabad_temperature != buff.toFloat()) && (isState(State_SpaTemperature))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.spabad_temperature = buff.toFloat();
  }

  if (String(topic).endsWith("bastu_temperatur")) {
    if ((import_data.bastu_temperature != buff.toFloat()) && (isState(State_SpaTemperature))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.bastu_temperature = buff.toFloat();
  }

  if (String(topic).endsWith("windspeed")) {
    if ((import_data.windspeed != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.windspeed = buff.toInt();
  }

  if (String(topic).endsWith("snowdepth")) {
    if ((import_data.snowdepth != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.snowdepth = buff.toInt();
  }

  if (String(topic).endsWith("9_temperature")) {
    if ((import_data.forecast_9_temperature != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.forecast_9_temperature = buff.toInt();
  }
  if (String(topic).endsWith("9_weather")) {
    strncpy(import_data.forecast_9_weather, buff.c_str(), sizeof(import_data.forecast_9_weather));
  }
  if (String(topic).endsWith("12_temperature")) {
    if ((import_data.forecast_12_temperature != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.forecast_12_temperature = buff.toInt();
  }
  if (String(topic).endsWith("12_weather")) {
    strncpy(import_data.forecast_12_weather, buff.c_str(), sizeof(import_data.forecast_12_weather));
  }
  if (String(topic).endsWith("15_temperature")) {
    if ((import_data.forecast_15_temperature != buff.toInt()) && (isState(State_DayForecast))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.forecast_15_temperature = buff.toInt();
  }
  if (String(topic).endsWith("15_weather")) {
    strncpy(import_data.forecast_15_weather, buff.c_str(), sizeof(import_data.forecast_15_weather));
  }

  if (String(topic).endsWith("reset_state")) {  // user for debugging
    machine.transitionTo(State_Reset);
  }

  if (String(topic).endsWith("update_display")) {  // user for debugging
    machine.transitionTo(State_UpdateDisplay);
  }

  if (String(topic).endsWith("restart")) {  // user for debugging
    machine.transitionTo(State_Error);
  }

  if (String(topic).endsWith("weather")) {
    strncpy(import_data.current_weather, buff.c_str(), sizeof(import_data.current_weather));
  }

  if (String(topic).endsWith("rank_1_name")) {
    strncpy(import_data.rank_1_name, buff.c_str(), sizeof(import_data.rank_1_name));
  }
  if (String(topic).endsWith("rank_1_value")) {
    if ((import_data.rank_1_value != buff.toInt()) && (isState(State_Ranking))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.rank_1_value = buff.toInt();
  }
  if (String(topic).endsWith("rank_2_name")) {
    strncpy(import_data.rank_2_name, buff.c_str(), sizeof(import_data.rank_2_name));
  }
  if (String(topic).endsWith("rank_2_value")) {
    if ((import_data.rank_2_value != buff.toInt()) && (isState(State_Ranking))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.rank_2_value = buff.toInt();
  }
  if (String(topic).endsWith("rank_3_name")) {
    strncpy(import_data.rank_3_name, buff.c_str(), sizeof(import_data.rank_3_name));
  }
  if (String(topic).endsWith("rank_3_value")) {
    if ((import_data.rank_3_value != buff.toInt()) && (isState(State_Ranking))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.rank_3_value = buff.toInt();
  }
  if (String(topic).endsWith("rank_4_name")) {
    strncpy(import_data.rank_4_name, buff.c_str(), sizeof(import_data.rank_4_name));
  }
  if (String(topic).endsWith("rank_4_value")) {
    if ((import_data.rank_4_value != buff.toInt()) && (isState(State_Ranking))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.rank_4_value = buff.toInt();
  }
  if (String(topic).endsWith("rank_5_name")) {
    strncpy(import_data.rank_5_name, buff.c_str(), sizeof(import_data.rank_5_name));
  }
  if (String(topic).endsWith("rank_5_value")) {
    if ((import_data.rank_5_value != buff.toInt()) && (isState(State_Ranking))) { 
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.rank_5_value = buff.toInt();
  }

  if (String(topic).endsWith("aurora")) {
    if (buff == "off") {
      if ((import_data.aurora) && (isState(State_DayForecast))) {
        machine.transitionTo(State_UpdateDisplay);
      }
      if ((import_data.aurora) && (isState(State_SpaTemperature))) {
        machine.transitionTo(State_UpdateDisplay);
      }
      import_data.aurora = false;
    }
    if (buff == "on") {
      if ((!import_data.aurora) && (isState(State_DayForecast))) {
        machine.transitionTo(State_UpdateDisplay);
      }
      if ((!import_data.aurora) && (isState(State_SpaTemperature))) {
        machine.transitionTo(State_UpdateDisplay);
      }
      import_data.aurora = true;
    }
  }

  if (String(topic).endsWith("charge_1_kwh")) {
    if (import_data.charge_1_kwh != buff.toFloat()) {
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.charge_1_kwh = buff.toFloat();
  }

  if (String(topic).endsWith("charge_2_kwh")) {
    if (import_data.charge_2_kwh != buff.toFloat()) {
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.charge_2_kwh = buff.toFloat();
  }

  if (String(topic).endsWith("charge_1_current_kw")) {
    if (import_data.charge_1_current_kw != buff.toFloat()) {
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.charge_1_current_kw = buff.toFloat();
  }

  if (String(topic).endsWith("charge_2_current_kw")) {
    if (import_data.charge_2_current_kw != buff.toFloat()) {
      machine.transitionTo(State_UpdateDisplay);
    }
    import_data.charge_2_current_kw = buff.toFloat();
  }

  if (String(topic).endsWith("override_hour")) {
    import_data.override_hour = buff.toInt();
  }
}

void saveEeprom(void) {
  EEPROM.put(0, eeprom);
  delay(100);
  EEPROM.commit();
  Serial.println("EEPROM: Saved");
  machine.transitionTo(State_Idle);
}

void idle(void) {
  if (machine.executeOnce) {
    Serial.println(F("idle"));
    display_init_full();

    display.setCursor(200, 200);
    display.println(F("Heja Magnus"));
    display.setCursor(0, 470);
    screenLogg(showTime().c_str());
  }
}

void reset(void) {
  if (machine.executeOnce) {
    Serial.println(F("reset score"));
    // Reset high score each day 
    import_data.rank_1_value = 0;
    import_data.rank_2_value = 0;
    import_data.rank_3_value = 0;
    import_data.rank_4_value = 0;
    import_data.rank_5_value = 0;

    // reset display
    display.setRotation(0);
    display.setFullWindow();
    display.fillScreen(GxEPD_WHITE);
    display.display();
    //display.hibernate();
  }
}

void display_init_full(void) {
  display.init(115200, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse
  display.setRotation(0);
  display.setFullWindow();
  display.setCursor(0, 0);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&Viral26pt7b);
  display.println("");
}

void display_ranks(void) {

  if (machine.executeOnce) {

    Serial.println(F("State: Ranks"));

    display_init_full();

    display.setFont(&Viral26pt7b);
    display.setCursor(290, 70);
    display.print("Topplista");

    display.setFont(&orange_juice_2_026pt7b);
    uint16_t x = 130;
    uint16_t new_y = 160;
    if (import_data.rank_1_value > 0) {
      display.setCursor(x, new_y);
      display.print(import_data.rank_1_name);
      display.setCursor(x+410, new_y);
      display.print(import_data.rank_1_value);
      new_y = new_y + 60;
    }
    if (import_data.rank_2_value > 0) {
      display.setCursor(x, new_y);
      display.print(import_data.rank_2_name);
      display.setCursor(x+410, new_y);
      display.print(import_data.rank_2_value);
      new_y = new_y + 60;
    }
    if (import_data.rank_3_value > 0) {
      display.setCursor(x, new_y);
      display.print(import_data.rank_3_name);
      display.setCursor(x+410, new_y);
      display.print(import_data.rank_3_value);
      new_y = new_y + 60;
    }
    if (import_data.rank_4_value > 0) {
      display.setCursor(x, new_y);
      display.print(import_data.rank_4_name);
      display.setCursor(x+410, new_y);
      display.print(import_data.rank_4_value);
      new_y = new_y + 60;
    }
    if (import_data.rank_5_value > 0) {
      display.setCursor(x, new_y);
      display.print(import_data.rank_5_name);
      display.setCursor(x+410, new_y);
      display.print(import_data.rank_5_value);
      new_y = new_y + 60;
    }
    display.display();
  }
}

void display_charge(void) {

  if (machine.executeOnce) {

    Serial.println(F("State: Charge"));

    display_init_full();

    display.setFont(&Viral26pt7b);
    display.setCursor(290, 70);
    display.print("Laddning");

    display.setCursor(110, 160);
    display.print("Laddare 1");

    display.setCursor(110+390, 160);
    display.print("Laddare 2");

    display.setCursor(110, 280);
    display.println(printDataFloat(import_data.charge_1_current_kw) + " kW");

    display.setCursor(110, 280+60);
    display.println(printDataFloat(import_data.charge_1_kwh) + " kWh");


    display.setCursor(110+390, 280);
    display.println(printDataFloat(import_data.charge_2_current_kw) + " kW");

    display.setCursor(110+390, 280+60);
    display.println(printDataFloat(import_data.charge_2_kwh) + " kWh");

    display.display();
  }
}


void display_day_forecast(void) {

  if (machine.executeOnce) {

    Serial.println(F("State: Day forecast"));

    display_init_full();

    display.setFont(&Viral26pt7b);
    display.setCursor(200, 70);
    display.print("Utomhus ");
    display.setCursor(470, 70);
    display.println(printData(import_data.utomhus_temperature) + "'");

    display.setFont(&Viral26pt7b);
    display.setCursor(200, 130);
    display.print("Vind    ");
    display.setCursor(470, 130);
    display.println(printData(import_data.windspeed) + " m/s");

    if (import_data.aurora) {
      display.setFont(&Viral26pt7b);
      display.setCursor(200, 200);
      display.print("CHANS NORRSKEN");
    } else if (import_data.snowdepth > 3) {
      display.setFont(&Viral26pt7b);
      display.setCursor(200, 190);
      display.print("Djup    ");
      display.setCursor(470, 190);
      display.println(printData(import_data.snowdepth) + " cm");
    }

    display.setFont(&Viral26pt7b);
    display.setCursor(200, 330);
    display.println(" 9:00");
    display.setCursor(470, 330);
    display.println(printData(import_data.forecast_9_temperature) + "'");
    display.setFont(&meteocons26pt7b);
    display.setCursor(570, 330+5);
    display.write(getWeatherIcon(import_data.forecast_9_weather));

    display.setFont(&Viral26pt7b);
    display.setCursor(200, 330+60);
    display.println("12:00");
    display.setCursor(470, 330+60);
    display.println(printData(import_data.forecast_12_temperature) + "'");
    display.setFont(&meteocons26pt7b);
    display.setCursor(570, 330+60+5);
    display.write(getWeatherIcon(import_data.forecast_12_weather));

    display.setFont(&Viral26pt7b);
    display.setCursor(200, 330+120);
    display.println("15:00");
    display.setCursor(470, 330+120);
    display.println(printData(import_data.forecast_15_temperature) + "'");
    display.setFont(&meteocons26pt7b);
    display.setCursor(570, 330+120+5);
    display.write(getWeatherIcon(import_data.forecast_15_weather));
    display.display();
  }
}

void display_spa_temperature(void) {
  if (machine.executeOnce) {

    Serial.println(F("State: spa temperature"));

    display_init_full();

    uint16_t x = 100;
    uint16_t y = 50;

    if (import_data.aurora) {
      display.setFont(&Viral26pt7b);
      display.setCursor(210, 130);
      display.print("CHANS NORRSKEN");
    }

    display.setFont(&Viral26pt7b);
    display.setCursor(210, 200);
    display.print("Badtunna");
    display.setCursor(450, 200);
    display.println(printDataFloat(import_data.spabad_temperature) + "'");

    if (import_data.bastu_temperature > 40) {
      display.setCursor(210, 270);
      display.print("Bastu");
      display.setCursor(450, 270);
      display.println(printDataFloat(import_data.bastu_temperature) + "'");
    }

    display.display();
  }
}

String printData(int8_t data) {
  if (data == -127) {
    return String("-");
  }
  return String(data);
}

String printDataFloat(float data) {
  if (data == -127) {
    return String("-");
  }
  return String(data, 1);
}

uint8_t getWeatherIcon(String weather) {
  if (weather.equals("fog")) {
    return 0x4c;
  }

  if (weather.equals("sunny")) {
      return 0x42;
  }

  if (weather.equals("cloudy")) {
    //return 0x4e;
    return 0x59;
  }

  if (weather.equals("partlycloudy")) {
    if (getHour() > 17) {
      return 0x49;
    }
    return 0x48;
  }

  if (weather.equals("fair")) {
    if (getHour() > 17) {
      return 0x49;
    }
    return 0x48;
  }

  if (weather.equals("clearsky")) {
    if (getHour() > 17) {
      return 0x43;
    }
    return 0x42;
  }

  if (weather.equals("heavyrain")) {
    return 0x52;
  }

  if (weather.equals("heavyrainshowers")) {
    return 0x52;
  }

  if (weather.equals("heavyrainshowersandthunder")) {
    return 0x4f;
  }

  if (weather.equals("heavyrainandthunder")) {
    return 0x5a;
  }

  if (weather.equals("rain")) {
    return 0x51;
  }

  if (weather.equals("lightrain")) {
    return 0x51;
  }

  if (weather.equals("lightrainshowers")) {
    return 0x51;
  }

  if (weather.equals("rainshowers")) {
    return 0x51;
  }

  if (weather.equals("rainy")) {
    return 0x51;
  }

  if (weather.equals("sleet")) {
    return 0x58;
  }

  if (weather.equals("lightsleet")) {
    return 0x58;
  }

  if (weather.equals("lightsleetshowers")) {
    return 0x58;
  }

  if (weather.equals("sleetshowers")) {
    return 0x58;
  }

  if (weather.equals("heavysleetshowers")) {
    return 0x58;
  }

  if (weather.equals("heavysleet")) {
    return 0x58;
  }

  if (weather.equals("heavysnow")) {
    return 0x57;
  }

  if (weather.equals("heavysnowshowers")) {
    return 0x57;
  }

  if (weather.equals("snow")) {
    return 0x55;
  }

  if (weather.equals("lightsnow")) {
    return 0x55;
  }

  if (weather.equals("lightsnowshowers")) {
    return 0x55;
  }

  if (weather.equals("snowshowers")) {
    return 0x55;
  }

  if (weather.equals("heavysleetandthunder")) {
    return 0x50;
  }

  if (weather.equals("heavysnowshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("thunder")) {
    return 0x50;
  }

  if (weather.equals("lightssleetshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("lightssnowshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("snowshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("lightsnowandthunder")) {
    return 0x50;
  }

  if (weather.equals("snowandthunder")) {
    return 0x50;
  }


  if (weather.equals("lightrainshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("rainshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("lightsleetandthunder")) {
    return 0x50;
  }

  if (weather.equals("sleetandthunder")) {
    return 0x50;
  }

  if (weather.equals("lightrainandthunder")) {
    return 0x50;
  }

  if (weather.equals("rainandthunder")) {
    return 0x50;
  }

  if (weather.equals("heavysnowandthunder")) {
    return 0x50;
  }

  if (weather.equals("heavysleetshowersandthunder")) {
    return 0x50;
  }

  if (weather.equals("sleetshowersandthunder")) {
    return 0x50;
  }
  return 0;
}

uint8_t getHour(void) {
  if (import_data.override_hour != -127) {
    return import_data.override_hour;
  }
  return tm.tm_hour;
}

String showTime() {
  char s[50];
  strftime(s, 50, "%Y-%m-%d %H:%M:%S ", &tm); 

  String str = s;

  if (tm.tm_isdst == 1)               // Daylight Saving Time flag
    str = str + " DST";
  else
    str = str + " Standard";
  return str;
}
