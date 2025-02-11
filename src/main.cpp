#include "Arduino.h"
#include "devconfig.h"
#include "heltec.h"
#include "Lora\loramesh.h"
#include <Wire.h>
#include <RadioLib.h>
#include "esp_log.h"

#if DISPLAY_ENABLE
#include "OLED\SSD1306.h"
#endif
//#define ARDUINO_RUNNING_CORE 1

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

void AppTask(void * parameter) {

    bool ret=0;
    uint8_t framesize=0;

    for(;;){
#if 0
        switch (nextstate) {
            case ST_TXBEACON:
              if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) {
                send_pct = 1;
                nextstate = ST_RXWAIT; 
              }
            case ST_RXWAIT:
              if (loramesh.receivePacket()){
                if (loramesh.mydd.devtype == DEV_TYPE_ROUTER){
                    //o router nao faz nada com o dado
                    //somente calcula indices de desempenho
                    //log_i("Pacote eh para mim e eu sou Router!!!!");

                    //proximo estado eh enviar o beacon novamente
                    nextstate = ST_STANDBY;
                }
                else{
                    //o end device deve tratar o pacote e retransmitir se for beacon
                    //sincronizar o relogio atual
                    //log_i("RX PCT SEND RESP seqnum = %d",lastpkt.seqnum );
                    //send_pct = 1;
                    nextstate = ST_STANDBY; 
                }
              }
              break;
            case ST_TXDATA: 
               if ((loramesh.mydd.devtype == DEV_TYPE_ENDDEV) && (actualslot == DATA_SLOT)){
                   //aqui eh minha janela de dados...devo enviar o pacote de dados...
                    //log_i("Aqui eh hora de enviar os dados!!!!");
                    nextstate = ST_RXWAIT; 
               }    
            default:
              break;
        } 
#endif

        vTaskDelay(100 / portTICK_PERIOD_MS);
    } 
}


//calcula a quantidade de polls e erros
void setindpolls(){
    uint16_t lastmyseqnum = loramesh.getLastSeqNum();
    uint16_t lastpacketseqnum = loramesh.getLastPctSeqNum();
    if (lastmyseqnum == lastpacketseqnum){
        idx_response++;
        log_i("Tx.sn=%d Rx.sn=%d Rx.cnt=%d ",lastmyseqnum, lastpacketseqnum, idx_response);

        #if DISPLAY_ENABLE  
        {
            char display_line3[20];
            sprintf(display_line3,"Tx=%d Rx=%d", lastmyseqnum, idx_response);
            Heltec.DisplayShowAll(display_line1,display_line2,display_line3);
            //log_i("%s",display_line3);
        }    
        #endif

    }


}


void toogleled(uint8_t ledpin){
   ledtoogle = !ledtoogle;
   digitalWrite(ledpin, ledtoogle);
}

void ledblink(uint8_t ledpin){
   
   digitalWrite(ledpin, HIGH);
   delay(100);
   digitalWrite(ledpin, LOW);

}

#if 0
void standby() {
    //Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    //Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    //int16_t res = loramesh.standby();
    //if (res != 0)
    //    log_e("Standby gave error: %d", res);

    //Clear Dio Actions
    //loramesh.clearDioActions();

    //Suspend all tasks
    vTaskSuspend(App_TaskHandle);
    //vTaskSuspend(EndDev_TaskHandle);

    //Set previous priority
    vTaskPrioritySet(NULL, prevPriority);

}

void start() {
    // Get actual priority
    UBaseType_t prevPriority = uxTaskPriorityGet(NULL);

    // Set max priority
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    // Resume all tasks
    vTaskResume(App_TaskHandle);
    //vTaskResume(EndDev_TaskHandle);

    // Start Receiving
    loramesh.startReceiving();

    // Set previous priority
    vTaskPrioritySet(NULL, prevPriority);
}
#endif

void setup()
{
    uint8_t ret=0;

    Serial.begin(9600);

    Heltec.begin(); //display

    loramesh.begin(); //radio
}


void node_init_sync(uint32_t new_FR) {
    uint32_t auxajust;
    // Update current free-running timer value
    current_FR = new_FR;

    // If this is not the first measurement, calculate drift
    if (previous_FR != 0) {
        // Calculate drift (difference in free-running timers minus expected sync interval)
        drift = (current_FR - previous_FR) - SYNC_INTERVAL_MS;

        // Adjust period using a proportional control
        adjustedPeriod -= drift * ADJUSTMENT_FACTOR;
        // Ensure the adjusted period remains within valid bounds
        adjustedPeriod = MAX(MAX_VAL, MIN(MIN_VAL, adjustedPeriod)); 
    }

    // Update the timer period 
    //setTimerPeriod(adjustedPeriod);
     syncronized = true;

    //log_i("previous_FR=%d currentB_FR=%d adjustedPeriod=%d",previous_FR,current_FR, adjustedPeriod);

    // Store the current free-running timer as the previous value for the next cycle
    previous_FR = current_FR;
}

#if 0
//void setTimerPeriod(float period) {
//    timerAlarmWrite(timer, (uint32_t)(period * 1000), true); // Assuming microsecond-based timer
//}
uint32_t node_init_sync (uint32_t beacontime){
  uint32_t deltams;
  deltams = (millis() - lastabstime);  
  lastabstime = millis() + (deltams/4);
  lastscantime_ms = beacontime;
  actualslot=0;
  nextstate = ST_RXWAIT;

  log_i("Sincronizando lastscantime_ms=%d lastabstime=%d deltams=%d", lastscantime_ms, lastabstime,deltams);

  return deltams;
}

uint32_t ptk_get_cycle_duration_ms(){
    return (SLOT_INTERVAL * MAX_SLOTS);
}

uint32_t cycle_start_time_ms=0;
/*
 * 	See header file
 */
uint8_t cySetNextPossibleStart( uint32_t starttime_ms )
{
	uint32_t curr_time_ms = 0;
	if( cycle_start_time_ms != 0 ){	// start time already set
		return 1;
	}
	curr_time_ms = millis();

	while( starttime_ms <= curr_time_ms ){
		starttime_ms += ptk_get_cycle_duration_ms( );
	}
	cycle_start_time_ms = starttime_ms;
	return 0;
}

/*
 * 	See header file
 */
uint32_t ptk_proc_node_init_sync ( ptk_core_state_s *pstate, uint32_t to_ms )
{
	loramesh_sync_s best_beacon;

	if(node_sync_to_best_beacon( to_ms, &best_beacon ) == 0){
		// beacon found
		pstate->join_agent.fav_address = best_beacon.source_address;
		pstate->join_agent.agent_hop_count = best_beacon.num_hops;
		pstate->join_agent.agent_subnet_id = best_beacon.subnet_id;
		pstate->join_agent.role = JOIN_ROLE_JOINEE;
		pstate->join_agent.state = JOIN_STT_PREPARED;
		if( best_beacon.num_hops == 0 ){
			// direct join at the repeater
			pstate->join_agent.type = JOIN_TYPE_DIRECT;
		} else {
			pstate->join_agent.type = JOIN_TYPE_INDIRECT;
		}
		return best_beacon.utc_time;
	} else {
		// no beacon found
		return 0;
	}
}
#endif


void slottimecontrol(){
    long currscantime_ms = 0;

    if (lastabstime == 0){
        lastabstime = millis();
    }
    
    if (syncronized == true){
        syncronized = false;
        slot_period = (uint32_t) adjustedPeriod;
    }
    else
     slot_period = SLOT_INTERVAL;

    currscantime_ms = (millis() - lastabstime);  
    if (currscantime_ms >= slot_period){
        lastabstime = millis();
        lastscantime_ms += currscantime_ms;
        actualslot++;

        if (actualslot >= MAX_SLOTS){
            actualslot=0;
            if (loramesh.mydd.devtype == DEV_TYPE_ROUTER) 
                nextstate = ST_TXBEACON;
            else
                nextstate = ST_RXWAIT;
        }
        //log_i("ActualSlot=%d scantime=%d lastAbsTime=%d",actualslot,lastscantime_ms,lastabstime);
    }
}

//termina

void buf_io_put32_tl(uint32_t value, uint8_t *buf)
{
    buf[0] = (uint8_t)(value      );
    buf[1] = (uint8_t)(value >> 8 );
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
}
 
void buf_io_put16_tl(uint16_t value, uint8_t *buf)
{
    buf[0] = (uint8_t)(value     );
    buf[1] = (uint8_t)(value >> 8);
}
 
uint32_t buf_io_get32_fl(uint8_t *buf)
{
    uint32_t value = buf[0] | (buf[1] << 8 ) | (((uint64_t)buf[2]) << 16) | (((uint64_t)buf[3]) << 24);
    return value;
}
 
uint16_t buf_io_get16_fl(uint8_t *buf)
{
    uint16_t value = buf[0] | (buf[1] << 8);
    return value;
}
 

 
void recv(uint8_t *buffer)
{
    uint8_t *pbuf = buffer;
 
    uint8_t addr_slave = *pbuf++;
    uint8_t addr_master = *pbuf++;
    uint8_t func = *pbuf++;
 
    uint16_t seqnum = buf_io_get16_fl(pbuf);
    pbuf += 2;
 
    uint32_t dtvalue = buf_io_get32_fl(pbuf);
    pbuf += 4;
 
}

#define MESTRE 10

void loop() {
    
    
    uint8_t framesize;
    uint8_t rtaddr=0;
    
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
        
    } 
    

    #else
    if (messageReceived) {
        messageReceived = false;
        ret = loramesh.receivePacket();
        //Fazer a checagem do segundo byte
        framesize = loramesh.sendPacketRes(1, voltage); //engereço de destino / valor
    } 
    #endif

    delay(10);

}

