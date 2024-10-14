//yesy

#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "RTClib.h"
#include <Arduino_FreeRTOS.h>

//MQTT
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = "";     // Username MQTT (opsional)
const char* mqtt_password = ""; // Password MQTT (opsional)

const char* topic_temperature = "greenhouse/sensor/temperature";
const char* topic_humidity = "greenhouse/sensor/humidity";
const char* topic_ldr = "greenhouse/sensor/ldr";

const char* topic_kontrol_kipas = "greenhouse/Kontrol/kipas";
const char* topic_kontrol_sprayer = "greenhouse/kontrol/sprayer";
const char* topic_kontrol_lampu = "greenhouse/kontrol/lampu";

const char* topic_set_temperature = "greenhouse/set/temperature";
const char* topic_set_humidity = "greenhouse/set/humidity";
const char* topic_set_ldr = "greenhouse/set/ldr";

char ssid[] = "bebas"; 
char pass[] = "akunulisaja";          

#define DHTPIN 4 //analog pin
#define DHTTYPE DHT22 
#define relay_kipas 19
#define relay_sprayer = 18;
#define relay_lampu = 17;
#define LDR_PIN = 34; //analog pin

LiquidCrystal_I2C lcd(0x27, 20, 4);  
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
RTC_DS1307 rtc;
int setWaktu1 [] = {4,0,0}; //jam, menit, detik
int setWaktu2 [] = {5,0,0};
int setWaktu3 [] = {18,0,0};

int ldrValue;
float temperature, humidity;
float set_temperature = 30;
float set_humidity = 80;
float set_ldr = 1000;
long lama_cahaya, lama_lampu;

void baca_RTC();
void setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);

void monitoring_task(void *pvParameters);
void manual_task(void *pvParameters);
void kontrol_kipas(void *pvParameters);
void kontrol_sprayer(void *pvParameters);
void kontrol_lampu(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(relay_kipas, OUTPUT);
  pinMode(relay_sprayer, OUTPUT);
  pinMode(relay_lampu, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  setup_wifi();
  dht.begin();
  lcd.init();         
  lcd.backlight();    
  lcd.setCursor(3,1);
	lcd.print("START  PROGRAM");
	lcd.setCursor(3,2);
	lcd.print("==============");
	delay(1000);
	lcd.clear();
	if (! rtc.begin()) {
		Serial.println("RTC tidak ditemukan");
		Serial.flush();
		abort();
	}
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //core 0
  xTaskCreatePinnedToCore(monitoring_task, "monitoring_task", 8198, NULL, 2, NULL, 0); 
  xTaskCreatePinnedToCore(manual_task, "manual_task", 8198, NULL, 2, NULL, 0);
  //core 1
  xTaskCreatePinnedToCore(kontrol_kipas, "kontrol_kipas", 8198, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(kontrol_sprayer, "kontrol_sprayer", 8198, NULL, 2, NULL, 1);  
  xTaskCreatePinnedToCore(kontrol_lampu, "kontrol_lampu", 8198, NULL, 2, NULL, 1); 
}

void loop() { }

//Task monitoring
void monitoring_task(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      reconnect(); 
    }
    client.loop();  

    //baca sensor
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    ldrValue = analogRead(LDR_PIN);
    baca_RTC();

    //serial monitor
    Serial.print("Suhu : "); Serial.println(temperature);
    Serial.print("Kelembaban : "); Serial.println(humidity);
    Serial.print("Cahaya : "); Serial.println(ldrValue);
    Serial.println("");

    //Print LCD
    lcd.setCursor(0, 1);
    lcd.print("Temp: "); lcd.print(temperature); lcd.print(" C");
    lcd.setCursor(0, 2);
    lcd.print("Hum: "); lcd.print(humidity); lcd.print(" %");
    lcd.setCursor(0, 3);
    lcd.print("LDR: "); lcd.print(ldrValue);

    //Print MQTT
		client.publish(topic_temperature, String(temperature).c_str());
    client.publish(topic_humidity, String(humidity).c_str());
    client.publish(topic_ldr, String(ldrValue).c_str());

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

//Task pengkondisian kipas
void kontrol_kipas(void *pvParameters){
  while (true) {
    temperature = dht.readTemperature();
    //setpoint = 30
    //30 < temp <= 35
    if (temperature <= set_temperature + 5 && temperature > set_temperature){
      digitalWrite (relay_kipas, HIGH);
       vTaskDelay(7000 / portTICK_PERIOD_MS);
      digitalWrite (relay_kipas, LOW);
    }
    //temp > 35
    else if (temperature > set_temperature + 5){
      digitalWrite (relay_kipas, HIGH);
       vTaskDelay(12000 / portTICK_PERIOD_MS);
      digitalWrite (relay_kipas, LOW);
    }
    //temp < 30
    else if (temperature <= set_temperature) {
      digitalWrite (relay_kipas, LOW);
    }

		vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
  }
}

//Task pengkondisian sprayer
void kontrol_sprayer(void *pvParameters){
  while (true) {
    humidity = dht.readHumidity();
    //setpoint = 80 %
    //60 <= hum < 80
    if (humidity >= set_humidity - 20 && humidity < set_humidity){
      digitalWrite (relay_sprayer, HIGH);
      vTaskDelay(4000 / portTICK_PERIOD_MS);
      digitalWrite (relay_sprayer, LOW);
    }
    //hum < 60
    else if (humidity < set_humidity - 20){
      digitalWrite (relay_sprayer, HIGH);
       vTaskDelay(8000 / portTICK_PERIOD_MS);
      digitalWrite (relay_sprayer, LOW);
    }
    //hum >= 30
    else if (humidity >= set_humidity) {
      digitalWrite (relay_sprayer, LOW);
    }

		vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
  }
}

//Task pengkondisian lampu
void kontrol_lampu(void *pvParameters){
  while (true) {
    DateTime now = rtc.now(); 
    //jam 4 pagi, reset
    if (now.hour() == setWaktu1[0]){
      lama_cahaya = 0;
    }
    //jam 5 sampai jam 18, menghitung lama cahaya
		if (now.hour() > setWaktu2[0] && now.hour() < setWaktu3[0]){
        if (ldrValue >= set_ldr) {
          lama_cahaya ++;
           vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else {
           vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    //jam 18 , menyalakan lampu
    if (now.hour() == setWaktu3[0]){
      lama_lampu = (36000 - lama_cahaya);
      for (int i = 1; i <= lama_lampu; i++) {
        digitalWrite (relay_lampu, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      digitalWrite (relay_lampu, LOW);
    }

		vTaskDelay(10 / portTICK_PERIOD_MS); // Delay 10ms
  }
}

//Task tombol manual
void manual_task(void *pvParameters) {
  while (true) {
    if (!client.connected()) {
      		reconnect();
    }
    client.loop();

		vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
  }
}

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

void setup_wifi() {
  delay(10);
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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("terhubung");

      // Suscribe ke topik
      client.subscribe(topic_kontrol_kipas);
      client.subscribe(topic_kontrol_sprayer);
      client.subscribe(topic_kontrol_lampu);
      client.subscribe(topic_set_temperature);
      client.subscribe(topic_set_humidity);
      client.subscribe(topic_set_ldr);

    } else {
      Serial.print("Gagal terhubung, rc=");
      Serial.print(client.state());
      Serial.println(" Tunggu 5 detik sebelum mencoba lagi");
      delay(5000);
    }
  }
}

void baca_RTC(){
	//Serial monitor
	Serial.print("Current time: ");
	Serial.print(now.year(), DEC);
	Serial.print('/');
	Serial.print(now.month(), DEC);
	Serial.print('/');
	Serial.print(now.day(), DEC);
	Serial.print(" (");
	Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
	Serial.print(") ");
	Serial.print(" - ");
	Serial.print(now.hour(), DEC);
	Serial.print(':');
	Serial.print(now.minute(), DEC);
	Serial.print(':');
	Serial.print(now.second(), DEC);
	Serial.println();
	//LCD
	lcd.setCursor(0,0);
	lcd.print(now.year(), DEC);
	lcd.print('/');
	lcd.print(now.month(), DEC);
	lcd.print('/');
	lcd.print(now.day(), DEC);
	lcd.print(" - ");
	lcd.print(now.hour(), DEC);
	lcd.print(':');
	lcd.print(now.minute(), DEC);
	lcd.print(':');
	lcd.print(now.second(), DEC);
}
