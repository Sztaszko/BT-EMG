#include "Arduino.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run 'make menuconfig' to enable it
#endif

bool device_connected = false;
bool congestedBT = false;
bool sending = false;
int send_samples_cnt = 0;
int samples_to_send = 0;
unsigned short value_to_send;
const int EMG_CH1 = 36;
const int EMG_CH2 = 39;
const int EMG_CH3 = 34;
const short channels = 3;
const int QUEUE_SIZE = channels*5000;
const int sampling_freq = 1000; //Hz
uint8_t *send_buffer = NULL;
const TickType_t queue_delay = 1 / portTICK_PERIOD_MS; //block for 1 ms

BluetoothSerial SerialBT;

unsigned short int debug_cnt;

TaskHandle_t TaskRead;
TaskHandle_t TaskSend;
QueueHandle_t dataQueue;

hw_timer_t *readTimer = NULL;


void sendData() {
//  if (samples_to_send == send_samples_cnt) {
//    sending = false;
//    send_samples_cnt = 0;
//    return;
//  }
//
//  send_samples_cnt++;
//  xQueueReceive(dataQueue, &value_to_send, queue_delay);
//  SerialBT.write(value_to_send);

//  uint8_t *samples = (uint8_t*) malloc(samples_to_send*sizeof(uint8_t));

  Serial.print("[DEBUG] samples size: ");
  Serial.println(samples_to_send);
  
  send_buffer = (uint8_t*) realloc(send_buffer, samples_to_send*sizeof(uint16_t));
  xQueueReceive(dataQueue, send_buffer, queue_delay);

  int bytes_sent;
  bytes_sent = SerialBT.write(send_buffer, samples_to_send*2);

  Serial.print("[DEBUG] send data bytes sent: ");
  Serial.println(bytes_sent);
}

void resendData() {
  int bytes_sent;
  bytes_sent = SerialBT.write(send_buffer, samples_to_send*2);
  Serial.print("[DEBUG] resend data bytes sent: ");
  Serial.println(bytes_sent);
}

void IRAM_ATTR readTimerISR() {
  unsigned short emg_value;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (device_connected) {
    emg_value = analogRead(EMG_CH1);
//    xQueueSendFromISR(dataQueue, &emg_value, &xHigherPriorityTaskWoken);
    debug_cnt++;
    xQueueSendFromISR(dataQueue, &debug_cnt, &xHigherPriorityTaskWoken);
    // switch context if necessary
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    
    emg_value = analogRead(EMG_CH2);
//    xQueueSendFromISR(dataQueue, &emg_value, &xHigherPriorityTaskWoken);
    debug_cnt++;
    xQueueSendFromISR(dataQueue, &debug_cnt, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
    }
    
    emg_value = analogRead(EMG_CH3);
//    xQueueSendFromISR(dataQueue, &emg_value, &xHigherPriorityTaskWoken);
    debug_cnt++;
    xQueueSendFromISR(dataQueue, &debug_cnt, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
    }
  }
}

// TODO: change to read from external ADC unit if used
void taskReadCode(void * pvParameters) {
   Serial.print("Task Read running on core ");
   Serial.println(xPortGetCoreID());
   

   readTimer = timerBegin(0, 80, true); // prescaler for 1 us
   timerAttachInterrupt(readTimer, &readTimerISR, true);
   timerAlarmWrite(readTimer, 1000, true);
   timerAlarmEnable(readTimer);
   
   for(;;) {
      delay(100);
   }
   vTaskDelete(NULL);
}

void taskSendCode(void * pvParameters) {
  Serial.print("Task Send running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;) {
//    unsigned int messages_no = uxQueueMessagesWaiting(dataQueue);
//    Serial.print("Messages in the queue ");
//    Serial.println(messages_no);

    if (!device_connected) {
      // wait for BT connection
      delay(50);
    } else {
      if (!sending) {
        samples_to_send = uxQueueMessagesWaiting(dataQueue);
//        Serial.print("[DEBUG] samples queue size: ");
//        Serial.println(samples_to_send);
        if (samples_to_send == 0)
          Serial.println("Buffer empty!");
       else {
          if (samples_to_send == QUEUE_SIZE)
            Serial.println("Buffer full!");
          sending = true;
          sendData();
       }
      } else {
        delay(5); //sending data, wait for finish
      }
    }
  }
  
  Serial.println("Deleting task Send");
  vTaskDelete(NULL);
}

static void BTCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch(event) {
        case ESP_SPP_SRV_OPEN_EVT:
          device_connected = true;
          Serial.println("BT client connected");
          timerRestart(readTimer);
          timerAlarmEnable(readTimer);
          break;
        case ESP_SPP_CLOSE_EVT:
          device_connected = false;
          Serial.println("BT client disconnected");
          timerAlarmDisable(readTimer);
          timerStop(readTimer);
          timerWrite(readTimer, 0);
          xQueueReset(dataQueue);
          if (send_buffer)
            free(send_buffer);
          break;
        case ESP_SPP_CONG_EVT:
          Serial.println("BT congested");
          if (param->cong.cong == 0) {
            if (congestedBT) {
              congestedBT = false;
              resendData();
            } else {
              sendData();
            }
          }
          break;
        case ESP_SPP_WRITE_EVT:
          if (param->write.status == ESP_SPP_SUCCESS) {
            if (param->write.cong == 0) {
              sendData();
            }
          } else {
            if (param->write.len == -1) {
              congestedBT = true;
            }
          }
          break;
        default:
          Serial.print("BT event: ");
          Serial.println(event);
    }
}


void setup() {
  Serial.begin(115200);
  delay(100); // give time to initialize
  
  SerialBT.register_callback(BTCallback);
  if (!SerialBT.begin("EMGsensor")) {
    Serial.println("Error initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized");
  }
  delay(100);
  
  Serial.println("The device has started");

  dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(unsigned short));
  if (dataQueue == NULL) {
    Serial.println("Error creating the data queue");
  }
  
  xTaskCreatePinnedToCore(
      taskReadCode,   /* Task function. */
      "Read",         /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &TaskRead,      /* Task handle to keep track of created task */
      0);             /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
      taskSendCode,   /* Task function. */
      "Send",         /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &TaskSend,      /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */
  delay(500);
}

void loop() {
  delay(500);
}