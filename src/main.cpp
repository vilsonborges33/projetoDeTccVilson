#include "Arduino.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora\loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#include <Wifi.h>
#include <PubSubClient.h>

#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#endif
//#define ARDUINO_RUNNING_CORE 1

#define MESTRE 1

TaskHandle_t App_TaskHandle = nullptr;
TaskHandle_t EndDev_TaskHandle = nullptr;

extern LoRaClass loramesh;
extern volatile bool messageReceived;

char rxpacket[BUFFER_SIZE];

#if DISPLAY_ENABLE  
char display_line1[20];
char display_line2[20];
#endif

statemac nextstate;
uint16_t idx_response=0;
uint8_t send_pct = 0;
bool ledtoogle=0;

//variaveis especificas do device
long lastabstime = 0;
long lastscantime_ms = 0;
uint8_t actualslot=0;
long slot_period = 0;
bool syncronized = false;

uint32_t previous_FR = 0; // Previous free-running time from ESP32 B
uint32_t current_FR = 0;  // Current free-running time from ESP32 B
int32_t drift = 0;         // Measured drift
float adjustedPeriod = EXPECTED_PERIOD_MS; // Adjusted period for ESP32 A

const char* ssid = "301_MZNET_2.4G";
const char* password = "93139039";
const char* mqtt_server = "ec2-18-228-8-224.sa-east-1.compute.amazonaws.com";

//pubsubclient
WiFiClient leitorDeTensao;
PubSubClient client(leitorDeTensao);

//pubsubclient
void setup_wifi() {

    log_i("Connecting to ");
    log_i("Connecting to %s",ssid);
  
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      log_i(".");
    }
  
    log_i("WiFi connected");
    log_i("IP address: ");
    log_i("%s", WiFi.localIP());
  }
  
  void reconnect() {
    
    while (!client.connected()) {
        log_i("Attempting MQTT connection...");
      
      if (client.connect("leitorDetensao")) {
        log_i("connected");
       
      } else {
        log_i("failed, rc=");
        log_i("%s",client.state());
        log_i(" try again in 5 seconds");
        delay(5000);
      }
    }
  }

void setup()
{
    
    uint8_t ret=0;

    Serial.begin(9600);

    Heltec.begin(); //display

    loramesh.begin(); //radio

    pinMode(7, INPUT);
    pinMode(2, OUTPUT);

    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    
    setup_wifi(); //so mestre
    client.setServer(mqtt_server, 1883);
}

void coneccao(){
    if(WiFi.status() == WL_CONNECTED)
        digitalWrite(2, HIGH);
    else
        digitalWrite(2, LOW);
}

void loop() {
    
    
    uint8_t framesize;
    uint8_t rtaddr=0;
    uint32_t leitura_adc = analogRead(7);
    delay(500);
    int len = 0; 
    bool ret=0;
    uint32_t voltage=3300; 
    uint8_t *buf = loramesh.lastpkt.rxpacket;  // apontando para a struct
    uint8_t pos = 10;  // Vamos assumir que o valor começa em pos=3
    uint32_t valorLido;

    #if MESTRE
    framesize = loramesh.sendPacketReq(lastscantime_ms);

    if (messageReceived) {
        delay(3000);
        reconnect();
        coneccao();
        log_i("Mestre: mensagem recebida"); 
        messageReceived = false;
        ret = loramesh.receivePacket();
        
        valorLido  = ((uint32_t)buf[5] << 24);
        log_i("1 =  %x", buf[5]);
        valorLido |= ((uint32_t)buf[6] << 16);
        log_i("1 =  %x", buf[6]);
        valorLido |= ((uint32_t)buf[7] << 8);
        log_i("1 =  %x", buf[7]);
        valorLido |= ((uint32_t)buf[8]);
        log_i("1 =  %x", buf[8]);

        log_i("Tensão valor_lido %d", valorLido); 
        client.publish("/Leitor_Tensao", String(valorLido).c_str());

       
        
    } 
    

    #else
    if (messageReceived) {
        messageReceived = false;
        ret = loramesh.receivePacket();
        //Fazer a checagem do segundo byte
        framesize = loramesh.sendPacketRes(1, leitura_adc); //engereço de destino / valor
    } 
    #endif

    delay(10);

}

