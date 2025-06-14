/**
 * @file EpEverSolarMonitor.ino
 * @brief Monitor and report EpEver solar charger metrics via RS485, MQTT, and HTTP.
 * @author Andrew Morty
 * @version 1.0.0
 * @license MIT
 *
 * This project reads data from an EpEver charge controller using Modbus RTU over RS485,
 * and publishes metrics to Home Assistant via MQTT with auto-discovery. A styled HTML
 * dashboard provides local access, and OTA firmware updates are supported.
 *
 * Features:
 *   - Modbus RTU polling of EpEver registers
 *   - RS485 MAX485 interface control
 *   - MQTT publishing and Home Assistant integration
 *   - HTML dashboard with real-time solar data
 *   - OTA firmware updates
 *
 * Wiring:
     Ethernet Green/White -> RS485MAX "B"
     Ethernet Blue/White  -> RS485MAX "A"
     Ethernet Brown/White -> NodeMCU G
     Ethernet Orange      -> NodeMCU VIN
     RS485MAX DI   -> GPI01 TX
     RS485MAX RO   -> GPI01 RX
     RS485MAX DE   -> D2
     RS485MAX RE   -> D1
     RS485MAX VCC  -> NodeMCU 3v
     RS485MAX G    -> Node MCU G (also ground the ethernet cable brown)
 
 */


#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>


// ============================================================================
// GLOBAL DECLARATIONS / CONSTANTS
// ============================================================================

#define FIRMWARE_VERSION "1.0.0"

#define UPTIME_REBOOT_INTERVAL_MS  3600000000UL  // ~41.66 days

// Contains wifi_ssid, wifi_pass, mqtt credentials, hostname, etc.
#include "secrets.h"

// === Pin Definitions ===

#define HB               D4
#define MAX485_DE        D2
#define MAX485_RE        D1

// === ModBus Register Definitions ===

#define PANEL_VOLTS     0x00
#define PANEL_AMPS      0x01
#define PANEL_POWER_L   0x02
#define PANEL_POWER_H   0x03
#define BATT_VOLTS      0x04
#define BATT_AMPS       0x05
#define BATT_POWER_L    0x06
#define BATT_POWER_H    0x07
#define LOAD_VOLTS      0x0C
#define LOAD_AMPS       0x0D
#define LOAD_POWER_L    0x0E
#define LOAD_POWER_H    0x0F


// define modbus error code constants
#define MODBUS_ERR_SUCCESS_BUT_INVALID_VALUE 255
#define MODBUS_STATUS_NOT_YET_READ 0xFE

// -----------------------------------------------------------------------------
// RS485 Polling Stats: Tracks success/failure counts for each Modbus register
// -----------------------------------------------------------------------------
unsigned long reg_0x3100_success_count = 0;
unsigned long reg_0x3100_fail_count    = 0;
unsigned long reg_0x311a_success_count = 0;
unsigned long reg_0x311a_fail_count    = 0;
unsigned long reg_0x3106_success_count = 0;
unsigned long reg_0x3106_fail_count    = 0;
unsigned long reg_0x3110_success_count = 0;
unsigned long reg_0x3110_fail_count    = 0;
unsigned long reg_0x311B_success_count = 0;
unsigned long reg_0x311B_fail_count    = 0;
unsigned long reg_0x3111_success_count = 0;
unsigned long reg_0x3111_fail_count    = 0;

boolean reg0x3100_success = false;
boolean reg0x3106_success = false;
boolean reg0x311A_success = false;
boolean reg0x311B_success = false;
boolean reg0x3110_success = false;
boolean reg0x3111_success = false;

uint8_t reg_0x3100_last_status = MODBUS_STATUS_NOT_YET_READ;
uint8_t reg_0x311a_last_status = MODBUS_STATUS_NOT_YET_READ;
uint8_t reg_0x3106_last_status = MODBUS_STATUS_NOT_YET_READ;
uint8_t reg_0x3110_last_status = MODBUS_STATUS_NOT_YET_READ;
uint8_t reg_0x311B_last_status = MODBUS_STATUS_NOT_YET_READ;
uint8_t reg_0x3111_last_status = MODBUS_STATUS_NOT_YET_READ;





// -----------------------------------------------------------------------------
// Solar Metrics: These values reflect real-time readings from the charge controller
// -----------------------------------------------------------------------------
// Voltage from PV panel (solar input)
float pv_voltage, pv_current, pv_power;         // PV panel metrics
float battery_voltage, battery_current, battery_power;
float load_voltage, load_current, load_power;
float battery_charge_power, battery_soc, battery_temp;




// -----------------------------------------------------------------------------
// Runtime Counters: Track system state and timings (Wi-Fi, Modbus, MQTT, etc.)
// -----------------------------------------------------------------------------
unsigned long millis_startup = 0;
unsigned long modbus_last_poll_millis  = 0;  // last Modbus polling time
unsigned long modbus_retry_start_millis  = 0;  // Modbus retry timing
unsigned long  ct_mqtt = 0;              // MQTT publish counter
unsigned int mqtt_last_status = 0;     // MQTT status: 1 = success, 0 = fail
unsigned long mqtt_last_sent_millis = 0;      // Timestamp of last successful MQTT send
String mqtt_last_payload;              // JSON sent to MQTT last
unsigned long millis_last_success = 0;  // last successful rs485 read of registers
bool ha_discovery_sent = false;         // home assistant discovery
String battery_temp_source = "none";


// === HTML Footer with Version and Timestamp ===
const char* footer_html =
"<div style='margin-top:2rem;font-size:0.75rem;color:#777;text-align:center;'>"
"Morty Labs solar charger firmware v" FIRMWARE_VERSION
" &mdash; compiled " __DATE__ " " __TIME__
"</div>";




String ota_status              = "none";

ModbusMaster node;
SoftwareSerial RS485Serial(RX, TX);   // RX, TX
ESP8266WebServer server(80);          // Create a webserver object that listens for HTTP request on port 80
ESP8266WiFiMulti wifiMulti;           // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'


WiFiClient    espClient;
PubSubClient  mqttClient(espClient);


void handle_not_found();
void handle_restart();                 // function prototypes for HTTP handlers


// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

String get_uptime (unsigned long my_millis) {
  unsigned long secs_elapsed = (millis() - my_millis) / 1000;
  unsigned long hours        = secs_elapsed / 60 / 60;
  unsigned long mins         = secs_elapsed / 60.00 - hours * 60;
  unsigned long secs         = secs_elapsed - hours * 60 * 60 - mins * 60;
  String up_time             = "";
  
  if (hours < 10) {
    up_time += "0";
  }
  up_time = up_time + hours;
  up_time = up_time + ":";
  if (mins < 10 ) {
    up_time += "0";
  }
  up_time = up_time + mins;
  up_time = up_time + ":";
  if (secs < 10 ) {
    up_time += "0";
  }
  up_time = up_time + secs;
  return up_time;
  
}


String get_elapsed(unsigned long last, unsigned long count) {
  if (count == 0) return "-";
  unsigned long delta = (millis() - last) / 1000;
  char buf[12];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", delta / 3600, (delta / 60) % 60, delta % 60);
  return String(buf);
}


// ============================================================================
// RS485 TRANSMISSION HOOKS / MODBUS
// ============================================================================

void pre_transmission()
{
  digitalWrite(MAX485_RE, HIGH);
  digitalWrite(MAX485_DE, HIGH);
  delayMicroseconds(50);  // Allow line to settle before transmitting
}


void post_transmission()
{
  delayMicroseconds(50);  // Wait before switching to receive
  digitalWrite(MAX485_RE, LOW);
  digitalWrite(MAX485_DE, LOW);
}



int read_modbus_registers_with_retry (unsigned long address, unsigned int address_len) {
  modbus_retry_start_millis = millis();  // Start timer
  uint8_t result = 0xFF;  // Assume failure initially
  node.clearResponseBuffer();
  
  while ((result != node.ku8MBSuccess) && (millis() - modbus_retry_start_millis < 800)) {
    //rs485_transmit_mode();
    result = node.readInputRegisters(address, address_len);  // Automatically calls pre/post hooks
    //rs485_receive_mode();
    if (result == node.ku8MBSuccess) {
      millis_last_success = millis();   // Track last Modbus success timestamp
      break;
    }
    delay(50);  // Give the bus a moment before retrying
    yield();    // Let Wi-Fi, OTA, and watchdog breathe
  }
  
  return result;
}



// ============================================================================
// MQTT / WIFI FUNCTIONS
// ============================================================================

void on_mqtt_message_received(char* p_topic, byte* p_payload, unsigned int p_length) {
}

void connect_to_wifi() {
  unsigned long ct_attempts = 0;
  unsigned long led_status = 1;
  WiFi.hostname(hostname);
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(200);
    //Serial.print(".");
    ct_attempts += 1;
    if (ct_attempts > 300) {
      ESP.restart();
    }
    if (led_status == 1) {
      led_status = 0;
      digitalWrite(HB, HIGH);
    } else {
      led_status = 1;
      digitalWrite(HB, LOW);
    }
  }
  
  digitalWrite(HB, HIGH);
  
  mqttClient.setBufferSize(1024);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(on_mqtt_message_received);
  if (!mqttClient.connected()) {
    reconnect_to_mqtt();
  }
  
}


void reconnect_to_mqtt() {
  unsigned long millis_mqtt_started = millis();
  while (!mqttClient.connected() and (millis() - millis_mqtt_started <= 10E3)) {
    //Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect( hostname, mqtt_user, mqtt_pass)) {
      if (!ha_discovery_sent) {
        publish_home_assistant_discovery();
        ha_discovery_sent = true;
        delay(20);
      }
    } else {
      delay(200);
    }
  }
}

void publish_home_assistant_discovery() {
  String topic_prefix = ha_discovery_topic;

  struct {
    const char* id;
    const char* name;
    const char* unit;
    const char* device_class;
    const char* state_class;
  } sensors[] = {
    {"epever_pv_voltage", "PV Voltage", "V", "voltage", "measurement"},
    {"epever_pv_current", "PV Current", "A", "current", "measurement"},
    {"epever_pv_power",   "PV Power",   "W", "power",   "measurement"},
    {"epever_battery_voltage", "Battery Voltage", "V", "voltage", "measurement"},
    {"epever_battery_current", "Battery Current", "A", "current", "measurement"},
    {"epever_battery_power",   "Battery Power",   "W", "power",   "measurement"},
    {"epever_load_voltage", "Load Voltage", "V", "voltage", "measurement"},
    {"epever_load_current", "Load Current", "A", "current", "measurement"},
    {"epever_load_power",   "Load Power",   "W", "power",   "measurement"},
    {"epever_battery_charge_power", "Battery Charge Power", "W", "power", "measurement"},
    {"epever_battery_temperature", "Battery Temperature", "C", "temperature", "measurement"},
    {"epever_battery_soc", "Battery SOC", "%", "battery", "measurement"},
    {"epever_wifi", "WiFi Signal", "dBm", "signal_strength", "measurement"}
  };

  String device_info =
  "\"device\":{\"identifiers\":[\"EpEverDevice\"],\"name\":\"EpEver Solar Monitor\",\"manufacturer\":\"MortyLabs\",\"model\":\"ESP8266 RS485 EPEVER Monitor\"}";

  for (auto& s : sensors) {
    String config = "{\"name\":\"" + String(s.name) +
    "\",\"state_topic\":\"" + String(mqtt_topic) +
    "\",\"unit_of_measurement\":\"" + s.unit +
    "\",\"value_template\":\"{{ value_json." + s.id + " }}\"," +
    "\"device_class\":\"" + s.device_class +
    "\",\"state_class\":\"" + s.state_class + "\"," +
    "\"unique_id\":\"" + String(s.id) + "\"," +
    device_info + "}";

    String discovery_topic = topic_prefix + s.id + "/config";
    mqttClient.publish(discovery_topic.c_str(), config.c_str(), true);
  }
}




String create_json_payload() {
  String json = "{\n";
  json += " \"epever_pv_voltage\": " + (reg0x3100_success ? String(pv_voltage) : "null") + ",\n";
  json += " \"epever_pv_current\": " + (reg0x3100_success ? String(pv_current) : "null") + ",\n";
  json += " \"epever_pv_power\": " + (reg0x3100_success ? String(pv_power) : "null") + ",\n";
  json += " \"epever_battery_voltage\": " + (reg0x3100_success ? String(battery_voltage) : "null") + ",\n";
  json += " \"epever_battery_current\": " + (reg0x3100_success ? String(battery_current) : "null") + ",\n";
  json += " \"epever_battery_power\": " + (reg0x3100_success ? String(battery_power) : "null") + ",\n";
  json += " \"epever_load_voltage\": " + (reg0x3100_success ? String(load_voltage) : "null") + ",\n";
  json += " \"epever_load_current\": " + (reg0x3100_success ? String(load_current) : "null") + ",\n";
  json += " \"epever_load_power\": " + (reg0x3100_success ? String(load_power) : "null") + ",\n";
  json += " \"epever_battery_charge_power\": " + (reg0x3106_success ? String(battery_charge_power) : "null") + ",\n";
  json += " \"epever_battery_temperature\": " + (battery_temp != NAN && battery_temp != 0  ? String(battery_temp) : "null") + ",\n";
  json += " \"epever_battery_soc\": " + (reg0x311A_success ? String(battery_soc) : "null") + ",\n";
  
  json += " \"epever_last_3100\": \"" + String(reg0x3100_success ? "OK" : get_modbus_error_description(reg_0x3100_last_status)) + "\",\n";
  json += " \"epever_last_3106\": \"" + String(reg0x3106_success ? "OK" : get_modbus_error_description(reg_0x3106_last_status)) + "\",\n";
  json += " \"epever_last_3110\": \"" + String(reg0x3110_success ? "OK" : get_modbus_error_description(reg_0x3110_last_status)) + "\",\n";
  json += " \"epever_last_311A\": \"" + String(reg0x311A_success ? "OK" : get_modbus_error_description(reg_0x311a_last_status)) + "\",\n";
  json += " \"epever_last_311B\": \"" + String(reg0x311B_success ? "OK" : get_modbus_error_description(reg_0x311B_last_status)) + "\",\n";
  json += " \"epever_last_3111\": \"" + String(reg0x3111_success ? "OK" : get_modbus_error_description(reg_0x3111_last_status)) + "\",\n";

  
  
  
  json += " \"RSSI\": " + String(WiFi.RSSI()) + ",\n";
  json += " \"MAC\": \"" + WiFi.macAddress() + "\",\n";
  json += " \"firmware_version\": \"" + String(FIRMWARE_VERSION) + "\"\n";
  
  json += "}";
  mqtt_last_payload = json;
  return json;
}




// ============================================================================
// HTTP / WEB RENDERING
// ============================================================================

const char html_template[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang='en'>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>EPEVER Solar Monitor</title>
<style>
body {
  background: #0e0e0e;
  color: #e0e0e0;
  font-family: 'Segoe UI', sans-serif;
  margin: 0;
  padding: 1rem;
}
h1, h2 {
  color: #2ecc71;
  margin-top: 2rem;
}
.metrics {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
  gap: 1rem;
}
.metric {
  background: #1e1e1e;
  padding: 1rem;
  border-radius: 10px;
  text-align: center;
}
.metric label {
  display: block;
  font-size: 0.85rem;
  color: #aaa;
}
.metric .value {
  font-size: 1.4rem;
  font-weight: bold;
  margin-top: 0.2rem;
}
table {
  width: 100%;
  margin-top: 1rem;
  border-collapse: collapse;
}
th, td {
  border: 1px solid #444;
  padding: 0.5rem;
  text-align: center;
}
th {
  background: #222;
  color: #2ecc71;
}
.success-count {
  color: #8f8;
}
.error-count.zero {
  color: #888;
}
.error-count.nonzero {
  color: #f88;
}
.mqtt-section {
  margin-top: 2rem;
}
.mqtt-row {
  margin-bottom: 1rem;
}
.mqtt-row label {
  font-weight: bold;
  color: #2ecc71;
  display: inline-block;
  min-width: 140px;
}
.mqtt-value.connected {
  color: #8f8;
}
.mqtt-value.disconnected {
  color: #f88;
}
.mqtt-payload {
  background: #1e1e1e;
  padding: 1rem;
  border-radius: 8px;
  white-space: pre-wrap;
  overflow-x: auto;
}
.upload-btn {
  display: inline-block;
  padding: 0.5rem 1rem;
  margin-top: 0.5rem;
  background-color: #2ecc71;
  color: #000;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  font-weight: bold;
  text-decoration: none;
}
.upload-btn:hover {
  background-color: #27ae60;
}

</style>
</head>
<body>
<h1>🔋 EpEver Solar Monitor</h1>
<div class='metrics'>
{{solar_status}}
</div>
<h2>📈 RS485 Communication</h2>
<div class='metrics'>
{{rs485_stats}}
</div>
<h2>📡 MQTT Status</h2>
<div class='mqtt-section'>
{{mqtt_status}}
</div>
</body>
</html>
)rawliteral";


String get_modbus_error_description(uint8_t code) {
  switch (code) {
    case 0x00: return "OK";
    case 0x01: return "Illegal Function";
    case 0x02: return "Illegal Data Address";
    case 0x03: return "Illegal Data Value";
    case 0x04: return "Slave Device Failure";
    case 0x05: return "Acknowledge";
    case 0x06: return "Slave Device Busy";
    case 0x08: return "Memory Parity Error";
    
    case 0x0A: return "Gateway Path Unavailable";
    case 0x0B: return "Gateway Target Device Failed to Respond";
    
    // ModbusMaster-specific error codes
    case 0xE0: return "Invalid Slave ID";
    case 0xE1: return "Invalid Function";
    case 0xE2: return "Response Timeout";
    case 0xE3: return "Invalid CRC";
    case 0xE4: return "Invalid Response";
    // Custom app-level error code
    case MODBUS_ERR_SUCCESS_BUT_INVALID_VALUE: return "sensor returned 0°C (assumed invalid)";
    case MODBUS_STATUS_NOT_YET_READ: return "Not yet read";


    
    default: return "Unknown Error";
  }
}


String metric_style(bool success) {
  return success ? "'value'" : "'value' style=\"color:#f88;\"";
}


String render_metric_html(const char* label, const char* id, const String& value, bool success, String subtext = "") {
  String display_value = value;
  String display_label = label;
  if (!success) display_value = "-";
  if (subtext != "") {
    display_label += subtext;
  }
  return "<div class='metric'><label>" + String(display_label) + "</label><span class=" + metric_style(success) + " id='" + id + "'>" + display_value + "</span></div>";
}


String render_rs485_html(const String& label, unsigned long success,  unsigned long fail,  uint8_t lastStatus) {
  String status;
  
  if (fail > 0) {
    // Format error code as 0xE2
    String hex_code = "0x" + String(lastStatus, HEX);
    hex_code.toUpperCase();
    
    // Format status message
    String desc = (success ? "OK" : get_modbus_error_description(lastStatus));

    status = "<span style='color:#f88; white-space:nowrap;'>" + hex_code + " – " + desc + "</span>";
  } else {
    status = "<span style='color:#8f8;'>OK</span>";
  }
  
  return "<tr>"
  "<td style='white-space:nowrap;'>" + label + "</td>"
  "<td class='success-count'>" + String(success) + "</td>"
  "<td class='error-count " + (fail != 0 ? "nonzero" : "zero") + "'>" + String(fail) + "</td>"
  "<td style='white-space:nowrap;'>" + status + "</td>"
  "</tr>";
}


String render_dashboard_html() {
  String page = FPSTR(html_template);
  
  
  String battery_temp_src_subtext;
  String battery_temp_src_subtext_colour = "grey";
  if (battery_temp_source == "0x311B") {
    battery_temp_src_subtext_colour = "green";
  } else if (battery_temp_source == "0x3110") {
    battery_temp_src_subtext_colour = "orange";
  }
  else if (battery_temp_source == "0x3111") {
    battery_temp_src_subtext_colour = "red";
  }
  battery_temp_src_subtext = " <span style='color:"+battery_temp_src_subtext_colour+"; font-size:smaller'>" + battery_temp_source + "</span>";
  
  String solar = "";
  solar += render_metric_html("Panel Voltage", "pv_voltage", String(pv_voltage) + " V", reg0x3100_success);
  solar += render_metric_html("Panel Current", "pv_current", String(pv_current) + " A", reg0x3100_success);
  solar += render_metric_html("Panel Power", "pv_power", String(pv_power) + " W", reg0x3100_success);
  solar += render_metric_html("Battery Voltage", "battery_voltage", String(battery_voltage) + " V", reg0x3100_success);
  solar += render_metric_html("Battery Current", "battery_current", String(battery_current) + " A", reg0x3100_success);
  solar += render_metric_html("Battery Power", "battery_power", String(battery_power) + " W", reg0x3100_success);
  solar += render_metric_html("Load Voltage", "load_voltage", String(load_voltage) + " V", reg0x3100_success);
  solar += render_metric_html("Load Current", "load_current", String(load_current) + " A", reg0x3100_success);
  solar += render_metric_html("Load Power", "load_power", String(load_power) + " W", reg0x3100_success);
  solar += render_metric_html("Battery Charge Power", "batt_charge_power", String(battery_charge_power) + " W", reg0x3106_success);
  solar += render_metric_html("Battery Temp C", "battery_temp", String(battery_temp) + " C", (battery_temp != NAN && battery_temp != 0), battery_temp_src_subtext);
  solar += render_metric_html("Battery SOC", "battery_soc", String(battery_soc) + " %", reg0x311A_success);
  solar += render_metric_html("Uptime", "uptime", get_uptime(millis_startup), true);
  //solar += render_metric_html("Last Poll", "last_poll", get_timestamp(millis_last_success), true);
  
  
  String rs485 = "<table><tr><th>Register</th><th>Success</th><th>Fail</th><th>Last Status</th></tr>";
  
  rs485 += render_rs485_html("Battery+PV (0x3100)", reg_0x3100_success_count,  reg_0x3100_fail_count, reg_0x3100_last_status);
  rs485 += render_rs485_html("Battery SOC (0x311A)", reg_0x311a_success_count, reg_0x311a_fail_count,  reg_0x311a_last_status);
  rs485 += render_rs485_html("Charge Power (0x3106)", reg_0x3106_success_count, reg_0x3106_fail_count,  reg_0x3106_last_status);
  rs485 += render_rs485_html("Battery Temp1 (0x311B)", reg_0x311B_success_count, reg_0x311B_fail_count,  reg_0x311B_last_status);
  rs485 += render_rs485_html("Battery Temp2 (0x3110)", reg_0x3110_success_count, reg_0x3110_fail_count,  reg_0x3110_last_status);
  rs485 += render_rs485_html("Battery Temp3 (0x3111)", reg_0x3111_success_count, reg_0x3111_fail_count,  reg_0x3111_last_status);
  rs485 += "</table>";
  
  
  String mqtt = "<div class='mqtt-section'>";
  mqtt += "<div class='mqtt-row'><label>Last Payload:</label><span class='mqtt-value'>" + String(mqtt_last_status ? "✅ Success" : "❌ Fail") + " (" + get_elapsed(mqtt_last_sent_millis, 1) + " ago)</span></div>";
  mqtt += "<div class='mqtt-row'><pre class='mqtt-payload'>" + mqtt_last_payload + "</pre></div>";
  
  mqtt += "</div>";
  mqtt += "<div class='mqtt-row'><a href='/firmware' class='upload-btn'>Upload Firmware</a></div>";
  mqtt += "<div class='mqtt-row'><a href='/reboot' class='upload-btn'>Reboot</a></div>";
  
  
  page.replace("{{solar_status}}", solar);
  page.replace("{{rs485_stats}}", rs485);
  page.replace("{{mqtt_status}}", mqtt);
  page += footer_html;
  
  
  return page;
}


void handle_firmware_upload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.setDebugOutput(true);
    WiFiUDP::stopAll();
    if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Update Success: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
    Serial.setDebugOutput(false);
  }
  yield();
}

void serve_firmware_form() {
  server.send(200, "text/html",
  "<html><head><title>Firmware Upload</title>"
  "<style>body{font-family:sans-serif;background:#121212;color:#eee;padding:2em;}"
  "form{background:#1e1e1e;padding:2em;border-radius:10px;}"
  "input[type=file]{margin-bottom:1em;}"
  "progress{width:100%;}</style>"
  "<script>"
  "function startUpload(){"
  "  var form = document.querySelector('form');"
  "  var progress = document.getElementById('prog');"
  "  var status = document.getElementById('status');"
  "  var xhr = new XMLHttpRequest();"
  "  xhr.upload.addEventListener('progress', function(e) {"
  "    if (e.lengthComputable) {"
  "      var percent = (e.loaded / e.total) * 100;"
  "      progress.value = percent;"
  "    }"
  "  });"
  "  xhr.onreadystatechange = function() {"
  "    if (xhr.readyState == 4 && xhr.status == 200) {"
  "      status.innerHTML = 'Upload complete. Rebooting...';"
  "    }"
  "  };"
  "  xhr.open('POST', '/firmware', true);"
  "  xhr.send(new FormData(form));"
  "  status.innerHTML = 'Uploading...';"
  "  return false;"
  "}"
  "</script></head><body>"
  "<h2>MortyLabs OTA Firmware Upload</h2>"
  "<form onsubmit='return startUpload();'>"
  "<input type='file' name='update' required><br>"
  "<input type='submit' value='Upload'><br><br>"
  "<progress id='prog' value='0' max='100'></progress>"
  "</form><div id='status'></div>"
  "</body></html>");
}




void handle_restart() {
  server.send(200, "text/html",
    "<html><head>"
    "<meta http-equiv='refresh' content='3;url=/' />"
    "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;padding-top:5rem;}</style>"
    "</head><body>"
    "<h1>♻️ Rebooting...</h1>"
    "<p>You’ll be redirected to the dashboard shortly.</p>"
    "</body></html>"
  );
  delay(1500);  // Give browser a chance to render
  ESP.restart();
}


void handle_not_found() {
  server.send(404, "text/plain", "Ungaas baba, I 404 you."); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}



// ============================================================================
// CORE ARDUINO
// ============================================================================


void setup()
{
  pinMode(HB, OUTPUT);                // Initialize the LED_BUILTIN pin as an output
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  
  
  delay(100);
  
  Serial.begin(115200);
  delay(200);
  while (!Serial) {
    delay(1);
  }
  
  
  wifiMulti.addAP(wifi_ssid, wifi_pass);   // add Wi-Fi networks you want to connect to
  connect_to_wifi();
  
  RS485Serial.begin(115200);
  node.begin(1, RS485Serial);
  //node.begin(1, Serial);
  
  //  Callbacks
  node.preTransmission(pre_transmission);
  node.postTransmission(post_transmission);
  
  server.on("/", []() {server.send(200, "text/html",  render_dashboard_html());});
  
  server.on("/json", []() {
    server.send(200, "application/json", create_json_payload());
  });
  
  
  server.on("/reboot", handle_restart);
  
  server.onNotFound(handle_not_found);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handle_not_found"
  server.begin();                           // Actually start the server
  
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_pass);
  
  ArduinoOTA.onStart([]() {
    ota_status = "starting";
  });
  ArduinoOTA.onEnd([]() {
    ota_status = "done";
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    ota_status = "in progress...";
    ;
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    ota_status = "ERROR";
    if (error == OTA_AUTH_ERROR) {
      ota_status = "auth failed";
    }
    else if (error == OTA_BEGIN_ERROR) {
      ota_status = "Begin failed";
    }
    else if (error == OTA_CONNECT_ERROR) {
      ota_status = "Connect failed";
    }
    else if (error == OTA_RECEIVE_ERROR) {
      ota_status = "Receive failed";
    }
    else if (error == OTA_END_ERROR) {
      ota_status = "End failed";
    }
  });
  
  ArduinoOTA.begin();
  ota_status = "OTA ready";
  millis_startup = millis();
}










void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connect_to_wifi();
  }
  
  
  // Restart if no success transmittion in 5  mins, or uptime > 41 days
  if (millis_last_success + (1000 * 60 * 5) < millis() || millis() - millis_startup > UPTIME_REBOOT_INTERVAL_MS ) {
    ESP.restart();
  }
  
  server.handleClient();                    // Listen for HTTP requests from clients
  ArduinoOTA.handle();
  
  
  if (modbus_last_poll_millis == 0 || (millis() - modbus_last_poll_millis > 1000 * 30)) {
    
    modbus_last_poll_millis = millis();
    uint8_t result;
    pv_voltage = NAN;
    pv_current = NAN;
    pv_power = NAN;
    battery_voltage = NAN;
    battery_current = NAN;
    battery_power = NAN;
    load_voltage = NAN;
    load_current = NAN;
    load_power = NAN;

    // Read 16 registers starting at 0x3100)
    reg0x3100_success = false;
    result = read_modbus_registers_with_retry(0x3100, 16);
    if (result == node.ku8MBSuccess) {
      pv_voltage = node.getResponseBuffer(PANEL_VOLTS) / 100.0f;
      pv_current = node.getResponseBuffer(PANEL_AMPS) / 100.0f;
      pv_power = (node.getResponseBuffer(PANEL_POWER_L) |
      (node.getResponseBuffer(PANEL_POWER_H) << 8)) / 100.0f;
      
      battery_voltage = node.getResponseBuffer(BATT_VOLTS) / 100.0f;
      battery_current = node.getResponseBuffer(BATT_AMPS) / 100.0f;
      battery_power = (node.getResponseBuffer(BATT_POWER_L) |
      (node.getResponseBuffer(BATT_POWER_H) << 8)) / 100.0f;
      
      load_voltage = node.getResponseBuffer(LOAD_VOLTS) / 100.0f;
      load_current = node.getResponseBuffer(LOAD_AMPS) / 100.0f;
      load_power = (node.getResponseBuffer(LOAD_POWER_L) |
      (node.getResponseBuffer(LOAD_POWER_H) << 8)) / 100.0f;
      
      reg_0x3100_success_count++;
      reg0x3100_success = true;
      reg_0x3100_last_status = 0x00; // OK
      
    } else {
      reg_0x3100_fail_count += 1;
      reg_0x3100_last_status = result;
    }
    
    
    battery_charge_power = NAN;
    reg0x3106_success = false;
    result = read_modbus_registers_with_retry(0x3106, 2);
    if (result == node.ku8MBSuccess) {
      battery_charge_power = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
      reg0x3106_success = true;
      reg_0x3106_success_count++; 
      reg_0x3106_last_status = 0x00;
      
    } else {
      reg_0x3106_fail_count++; 
      reg_0x3106_last_status = result;
    }
    
    
    // ========================
    // Read Battery Temperature
    // ========================
    battery_temp = NAN;
    reg0x311B_success = false;
    battery_temp_source = "N/A";
    result = read_modbus_registers_with_retry(0x311B, 1);
    if (result == node.ku8MBSuccess)  {
      battery_temp = node.getResponseBuffer(0x00);
      
      if (battery_temp  != 0) {
        reg0x311B_success = true;
        reg_0x311B_success_count++;
        battery_temp_source = "0x311B";
        reg_0x311B_last_status = 0x00;
        
      } else {
        reg_0x311B_last_status = MODBUS_ERR_SUCCESS_BUT_INVALID_VALUE;
        reg_0x311B_fail_count++;
        
      }
      
    } else  {
      reg_0x311B_fail_count++;
      reg_0x311B_last_status = MODBUS_ERR_SUCCESS_BUT_INVALID_VALUE;
      
    }
    
    if (!reg0x311B_success) {
      reg0x3110_success = false;
      result = read_modbus_registers_with_retry(0x3110, 2);
      if (result == node.ku8MBSuccess)  {
        battery_temp = node.getResponseBuffer(0x00) / 100.0f;
        if (battery_temp !=  0) {
          reg0x3110_success = true;
          reg_0x3110_success_count++;
          battery_temp_source = "0x3110";
          reg_0x3110_last_status = 0x00;
          
          
        }  else {
          reg_0x3110_last_status = MODBUS_ERR_SUCCESS_BUT_INVALID_VALUE; //"Success but 0C temp";
          reg_0x3110_fail_count++;
        }
      }
      else  {
        reg_0x3110_fail_count += 1;
        reg_0x3110_last_status = result;
        
        if (battery_temp == 0 or !reg0x3110_success) {
          result = read_modbus_registers_with_retry(0x3111, 2);
          if (result == node.ku8MBSuccess) {
            reg0x3111_success = true;
            reg_0x3111_success_count++;
            battery_temp_source = "0x3111";
            uint16_t raw = node.getResponseBuffer(0);
            battery_temp = (raw >> 4) & 0x0F; // Extract temp bits from BATTERY_STATUS
            reg_0x3111_last_status = 0x00;
          } else {
            reg_0x3111_fail_count++;
            reg_0x3111_last_status = result;
          }
        }
      }
    }
    
    
    battery_soc = NAN;
    // Battery SOC
    //result = read_modbus_registers_with_retry(0x311A, 2);
    result = read_modbus_registers_with_retry(0x311A, 1);
    reg0x311A_success = false;
    if (result == node.ku8MBSuccess)  {
      reg0x311A_success = true;
      reg_0x311a_success_count = reg_0x311a_success_count + 1;
      battery_soc = node.getResponseBuffer(0x00);
      reg_0x311a_last_status = 0x00;
    } else  {
      reg_0x311a_fail_count = reg_0x311a_fail_count + 1;
      reg_0x311a_last_status = result;
    }

    if (!mqttClient.connected()) {
      reconnect_to_mqtt();
    }
    
    mqtt_last_status = mqttClient.publish(mqtt_topic, create_json_payload().c_str(), true);
    if (mqtt_last_status) mqtt_last_sent_millis = millis();
    ct_mqtt = ct_mqtt + 1;
    
    
    digitalWrite(HB, HIGH);
    delay(100);
    digitalWrite(HB, LOW);
  }
  
}

