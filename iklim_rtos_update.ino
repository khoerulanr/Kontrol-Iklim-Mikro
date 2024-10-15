#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

// MQTT
const char* mqtt_server = "broker.mqtt-dashboard.com";
const int mqtt_port = 1883;

const char* topic_temperature = "greenhouse/sensor/temperature";
const char* topic_humidity = "greenhouse/sensor/humidity";
const char* topic_ldr = "greenhouse/sensor/ldr";

const char* topic_kontrol_kipas = "greenhouse/Kontrol/kipas";
const char* topic_kontrol_sprayer = "greenhouse/kontrol/sprayer";
const char* topic_kontrol_lampu = "greenhouse/kontrol/lampu";

const char* topic_set_temperature = "greenhouse/set/temperature";
const char* topic_set_humidity = "greenhouse/set/humidity";
const char* topic_set_ldr = "greenhouse/set/ldr";

char* ssid = "bebas"; 
char* pass = "akunulisaja";          

#define DHTPIN 4
#define DHTTYPE DHT22
#define relay_kipas 19
#define relay_sprayer 18
#define relay_lampu 17
#define LDR_PIN 34

LiquidCrystal_I2C lcd(0x27, 20, 4);  
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

int ldrValue;
float temperature, humidity;
float set_temperature = 30;
float set_humidity = 80;
float set_ldr = 500;

void setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);

void monitoring_task(void *pvParameters);
void kontrol_kipas_task(void *pvParameters);
void kontrol_sprayer_task(void *pvParameters);
void kontrol_lampu_task(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(relay_kipas, OUTPUT);
  pinMode(relay_sprayer, OUTPUT);
  pinMode(relay_lampu, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  setup_wifi();
  dht.begin();
  lcd.backlight();
  lcd.setCursor(3, 1);
  lcd.print("START PROGRAM");
  delay(1000);
  lcd.clear();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Core 0 untuk monitoring
  xTaskCreatePinnedToCore(monitoring_task, "monitoring_task", 3000, NULL, 1, NULL, 0); 
  
  // Core 1 untuk kontrol perangkat
  xTaskCreatePinnedToCore(kontrol_kipas_task, "kontrol_kipas_task", 1000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(kontrol_sprayer_task, "kontrol_sprayer_task", 1000, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(kontrol_lampu_task, "kontrol_lampu_task", 1000, NULL, 1, NULL, 1);
}

void loop() {
  if (!client.connected()) {
      reconnect(); 
    }
    client.loop();  }

// Task monitoring sensor
void monitoring_task(void *pvParameters) {
  while (true) {
   

    temperature = random(25, 36);
    humidity = random(70, 90);
    ldrValue = random(40, 1000);

    // Serial monitor
   // Serial.print("Suhu : "); Serial.println(temperature);
   // Serial.print("Kelembaban : "); Serial.println(humidity);
   // Serial.print("Cahaya : "); Serial.println(ldrValue);

    // Print LCD
    lcd.setCursor(0, 1);
    lcd.print("Temp: "); lcd.print(temperature); lcd.print(" C");
    lcd.setCursor(0, 2);
    lcd.print("Hum: "); lcd.print(humidity); lcd.print(" %");
    lcd.setCursor(0, 3);
    lcd.print("LDR: "); lcd.print(ldrValue);

    // Publish MQTT
    client.publish(topic_temperature, String(temperature).c_str());
    client.publish(topic_humidity, String(humidity).c_str());
    client.publish(topic_ldr, String(ldrValue).c_str());

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task kontrol kipas
void kontrol_kipas_task(void *pvParameters) {
  while (true) {
    temperature = random(25, 36);
    set_temperature = 30;
  //  30 < temp <= 35
    if (temperature > set_temperature) {
      digitalWrite(relay_kipas, HIGH);
      Serial.println("Kipas On");
    } else {
      digitalWrite(relay_kipas, LOW);
      Serial.println("Kipas Off");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Task kontrol sprayer
void kontrol_sprayer_task(void *pvParameters) {
  while (true) {
    humidity = random(70, 90);
    set_humidity = 80;
   //  60 <= hum < 80
    if (humidity < set_humidity) {
      digitalWrite(relay_sprayer, HIGH);
      Serial.println("sprayer on");
    } else {
      digitalWrite(relay_sprayer, LOW);
      Serial.println("sprayer off");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Task kontrol lampu
void kontrol_lampu_task(void *pvParameters) {
  while (true) {
    ldrValue = random(40, 1000);
     set_ldr = 500;
   // 200 <= ldr < 1000
    if (ldrValue < set_ldr) {
      digitalWrite(relay_lampu, HIGH);
      Serial.println("lampu on");
    } else {
      digitalWrite(relay_lampu, LOW);
      Serial.println("lampu off");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Setup WiFi
void setup_wifi() {
  Serial.println();
  Serial.print("Menghubungkan ke ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Terhubung ke WiFi dengan IP: ");
  Serial.println(WiFi.localIP());
}

// Reconnect to MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("terhubung");
      client.subscribe(topic_kontrol_kipas);
      client.subscribe(topic_kontrol_sprayer);
      client.subscribe(topic_kontrol_lampu);
      client.subscribe(topic_set_temperature);
      client.subscribe(topic_set_humidity);
      client.subscribe(topic_set_ldr);
    } else {
      Serial.print("Gagal terhubung, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// MQTT callback
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Receive Topic: ");
  Serial.println(topic);

  Serial.print("Payload: ");
  
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';  
  Serial.println(msg);

  // kontrol kipas
  if (!strcmp(topic, topic_kontrol_kipas)) {
    if (!strncmp(msg, "on", length)) {
      digitalWrite(relay_kipas, HIGH);
      Serial.println("Kipas ON");
    } else if (!strncmp(msg, "off", length)) {
      digitalWrite(relay_kipas, LOW);
      Serial.println("Kipas OFF");
    }
  }

  //kontrol sprayer
  if (!strcmp(topic, topic_kontrol_sprayer)) {
    if (!strncmp(msg, "on", length)) {
      digitalWrite(relay_sprayer, HIGH);
      Serial.println("Sprayer ON");
    } else if (!strncmp(msg, "off", length)) {
      digitalWrite(relay_sprayer, LOW);
      Serial.println("Sprayer OFF");
    }
  }

  //kontrol lampu
  if (!strcmp(topic, topic_kontrol_lampu)) {
    if (!strncmp(msg, "on", length)) {
      digitalWrite(relay_lampu, HIGH);
      Serial.println("Lampu ON");
    } else if (!strncmp(msg, "off", length)) {
      digitalWrite(relay_lampu, LOW);
      Serial.println("Lampu OFF");
    }
  }

  // Set suhu 
  if (!strcmp(topic, topic_set_temperature)) {
    set_temperature = atoi(msg); 
    Serial.print("Set suhu menjadi : ");
    Serial.println(set_temperature);
  }

  // Set kelembaban
  if (!strcmp(topic, topic_set_humidity)) {
    set_humidity = atoi(msg); 
    Serial.print("Set kelembaban menjadi : ");
    Serial.println(set_humidity);
  }

  // Set cahaya
  if (!strcmp(topic, topic_set_ldr)) {
    set_ldr = atoi(msg); 
    Serial.print("Set LDR menjadi : ");
    Serial.println(set_ldr);
  }
  
}
