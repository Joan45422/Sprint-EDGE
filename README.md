O projeto de Internet das Coisas (IoT) desenvolvido pela equipe (Felipe Bonilha, Felipe Rodrigues, Gabriel Salles, Joan Ferreira e Levi de Jesus) demonstra a comunicação bidirecional completa entre um dispositivo de borda (ESP32, simulado via Wokwi) e a plataforma de nuvem Adafruit IO, utilizando o protocolo MQTT.

A implementação atende integralmente aos requisitos da Entrega Hands-On, focando na integração, publicação, subscrição e visualização em tempo real.

1. Comunicação MQTT e Publicação de Dados
O dispositivo ESP32 estabelece uma conexão persistente e segura com o Adafruit IO, autenticada pelo usuário (Joanzin) e sua AIO Key. O sistema principal simula um rastreador GPS, publicando dados de telemetria – Latitude, Longitude e Velocidade – em Feeds dedicados a cada 10 segundos. Esta automação garante o monitoramento contínuo da localização do dispositivo.

2. Subscrição e Funcionamento em Tempo Real
Para demonstrar a comunicação bidirecional e o controle remoto, o código implementa a subscrição a um Feed de comando (joanzin/feeds/comando).

A função callback é acionada instantaneamente ao receber comandos (ex: '0' ou '1') enviados a partir do Dashboard do Adafruit IO.

Esta funcionalidade comprova a capacidade do sistema de reagir a instruções da nuvem em tempo real, atendendo ao requisito de subscrição.

3. Visualização e Demonstração Final
A comprovação dos resultados é feita através do Dashboard na plataforma Adafruit IO. Os dados de GPS publicados pelo Wokwi são visualizados dinamicamente por meio de blocos de Mapa (para rastreamento geográfico) e Medidor (para velocidade). A interação com um Toggle Switch no Dashboard permite testar a subscrição, com a resposta sendo imediatamente registrada no Monitor Serial do simulador Wokwi, validando a integração IoT de ponta a ponta. #include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h> 
#include "secrets.h"

// --- 1. Configurações de Rede e MQTT ---
const char* ssid = SECRET_SSID; 
const char* password = SECRET_PASS; 

WiFiClient espClient;
PubSubClient client(espClient);

const char* commandFeed = AIO_FEED_COMANDO; 

// --- 2. Configurações e Pinos do GPS ---
#define GPS_BAUDRATE 9600 
#define GPS_RX_PIN 16 
#define GPS_TX_PIN 17 
TinyGPSPlus gps; 

// --- 3. Variáveis de Controle ---
unsigned long lastUpdateTime = 0;
const long updateInterval = 10000;
const long locationChangeInterval = 30000;
unsigned long lastLocationChange = 0;
const long initialWaitTime = 10000; 

struct GpsPoint {
  float lat;
  float lon;
  float speed;
};

GpsPoint locations[] = {
  { -23.5505, -46.6333, 50.0 },  // São Paulo
  { -22.9068, -43.1729, 60.0 },  // Rio de Janeiro
  { -15.7797, -47.9297, 40.0 }   // Brasília
};
int current_location_index = 0;

// ===================================================
// FUNÇÃO CALLBACK (Subscrição)
// ===================================================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n[SUBSCRICAO] Mensagem recebida do Feed: [");
  Serial.print(topic);
  Serial.print("] - Payload: ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  if (strcmp(topic, commandFeed) == 0) {
    Serial.print(">>> COMANDO DE SUBSCRICAO RECEBIDO! Valor: ");
    Serial.println(message);
    
    if (message == "1") {
      Serial.println(">>> Comandado para LIGAR (Exemplo).");
    } else if (message == "0") {
      Serial.println(">>> Comandado para DESLIGAR (Exemplo).");
    }
  }
}


// ===================================================
// FUNÇÕES MQTT (Reconexão e Publicação)
// ===================================================

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT com Adafruit IO...");
    
    if (client.connect(AIO_USERNAME, AIO_USERNAME, AIO_KEY)) {
      Serial.println("conectado com sucesso!");
      
      client.subscribe(commandFeed); 
      Serial.print("Subscrito ao Feed: ");
      Serial.println(commandFeed);
      
    } else {
      Serial.print("Falha na conexão MQTT. Código: ");
      Serial.print(client.state());
      Serial.println(" Tente novamente em 5 segundos...");
      delay(5000);
    }
  }
}


bool publishMQTTData(const char* feed, float value) {
  char dataString[20];
  dtostrf(value, 4, 6, dataString); 
  Serial.print("  Publicando em "); Serial.print(feed); Serial.print(" -> "); Serial.println(dataString);
  return client.publish(feed, dataString);
}

// ===================================================
// FUNÇÃO PARA INJETAR NOVO COMANDO DE LOCALIZAÇÃO NO WOKWI
// ===================================================
void updateGpsLocation() {
  GpsPoint current = locations[current_location_index];
  
  Serial.print("gps:location ");
  Serial.print(current.lat, 4);
  Serial.print(",");
  Serial.print(current.lon, 4);
  Serial.print(",");
  Serial.print(0.0, 1); 
  Serial.print(",");
  Serial.println(current.speed, 1);
  
  current_location_index = (current_location_index + 1) % (sizeof(locations) / sizeof(locations[0]));
  
  lastLocationChange = millis();
  Serial.println(">>> Localização de Partida ALTERADA (Wokwi Command) <<<");
}
// ===================================================


void setup() {
  Serial.begin(115200); 
  Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println(F("ESP32 - GPS Tracker para Adafruit IO (MQTT)"));

  Serial.print("Conectando-se a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(AIO_SERVER, 1883); 
  client.setCallback(callback);
  
  updateGpsLocation(); 
  
  Serial.println("Aguardando fixo GPS para iniciar o envio...");
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop(); 

  if (millis() - lastLocationChange >= locationChangeInterval) {
    updateGpsLocation();
  }
  
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      
      if (millis() - lastUpdateTime > updateInterval) {
        
        Serial.println("--- Tentativa de Envio MQTT ao Adafruit IO ---");

        if (gps.location.isValid()) {
          float latitude = gps.location.lat();
          float longitude = gps.location.lng();
          float speed_kmph = gps.speed.kmph(); 

          Serial.print("Latitude: "); Serial.println(latitude, 6);
          Serial.print("Longitude: "); Serial.println(longitude, 6);
          Serial.print("Velocidade: "); Serial.print(speed_kmph, 2); Serial.println(" km/h");
          
          bool success = publishMQTTData(AIO_FEED_LATITUDE, latitude);
          success &= publishMQTTData(AIO_FEED_LONGITUDE, longitude);
          success &= publishMQTTData(AIO_FEED_SPEED, speed_kmph);

          if(success){
            Serial.println("Envio MQTT OK. ✔️");
          } else {
            Serial.println("Falha no envio MQTT. ❌");
          }
          
        } else {
          Serial.println("Localização GPS inválida (Sem Fixo). ❌");
        }
        
        lastUpdateTime = millis();
        Serial.println("----------------------------------------");
      } 
    } 
  } 

  if (millis() > initialWaitTime && gps.charsProcessed() < 10) {
    delay(1); 
  }
}
