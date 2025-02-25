#include <WiFi.h>
#include <PubSubClient.h>

/*
Subscribe means get reading MQTT Broker
Publish   means send values MQTT Broker
For testing, we will use the free Mosquitto broker (test.mosquitto.org).

*/

#define RTD_SLAVE_ID 3  // ADAM 4015 ID
#define RS485_BAUD_RATE 9600
#define REG_COUNT 5
#define RS485_TX_PIN 17
#define RS485_RX_PIN 16
#define RS485_DE_RE_PIN 4  // DE and RE pins tied together and connected to GPIO 4

HardwareSerial RS485Serial(1);

const char* ssid = "ECC";
const char* password = "123456789";
// const char* mqtt_server = "test.mosquitto.org"; // online mosquitto broker
const char* mqtt_server = "192.168.136.236"; // local pc mosquitto broker

WiFiClient espClient;
PubSubClient client(espClient);

bool ON = LOW;
bool OFF = HIGH;

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("Chiller_Master")) {
      Serial.println(" Connected!");
      client.subscribe("SmartAgri/light");  // Subscribe to a topic
    } else {
      Serial.print(" Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();

  if (String(topic) == "SmartAgri/light") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("Light ON");
    } else if (messageTemp == "off") {
      Serial.println("Light OFF");
    }
  }
}

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode

  connectWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

unsigned long lastMsg = 0;
uint16_t tempReg[5];
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  if (millis() - lastMsg > 1000) {
    lastMsg = millis();
    // String message = "Hello from ESP32!";
    // client.publish("esp32/test", message.c_str());
    char tempString[10];
    // dtostrf(Temp, 2, 1, tempString); // float to MQTT
    sprintf(tempString, "%u", tempReg[0]);  // Convert uint16_t to string
    client.publish("SmartAgri/temperature", tempString);
  }

  if (readHoldingRegisters(RTD_SLAVE_ID, 0, REG_COUNT, tempReg)) {
    // for (int i = 0; i < REG_COUNT; i++) {
    //   Serial.print(tempReg[i]);
    //   Serial.print("  ");
    // }
    // Serial.println();
  } else {
    Serial.println("No Response !");
  }
  delay(300);
}

uint16_t calculateCRC(uint8_t* buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void clearSerialBuffer() {
  while (RS485Serial.available()) RS485Serial.read();  // Clear any old data
}

void sendModbusRequest(uint8_t slaveId, uint8_t functionCode, uint16_t startAddress, uint16_t registerCount) {
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = functionCode;
  request[2] = startAddress >> 8;
  request[3] = startAddress & 0xFF;
  request[4] = registerCount >> 8;
  request[5] = registerCount & 0xFF;
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;
  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();  // Ensure all data is sent
}

bool readModbusResponse(uint8_t* response, uint16_t length) {
  uint16_t index = 0;
  if (RS485Serial.available() > 0) {
    index = RS485Serial.readBytes(response, length);
    if (index >= length) {
      return true;
    }
  }
  return false;
}

bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t* buffer) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  sendModbusRequest(slaveId, 0x03, startAddress, registerCount);
  clearSerialBuffer();                 // Ensure buffer is empty
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set DE/RE pin low for receive mode
  delay(100);
  uint8_t response[5 + 2 * registerCount];
  if (readModbusResponse(response, sizeof(response))) {
    uint16_t crc = (response[sizeof(response) - 1] << 8) | response[sizeof(response) - 2];
    if (calculateCRC(response, sizeof(response) - 2) == crc) {
      for (int i = 0; i < registerCount; i++) {
        buffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
      }
      return true;
    } else {
      Serial.println("CRC error!");
    }
  } else {
    Serial.println("No response or timeout!");
  }
  return false;
}
