#include "Arduino.h"
#include "BluetoothSerial.h"
#include <freertos/stream_buffer.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run 'make menuconfig' to enable it
#endif

bool device_connected = false;
bool congestedBT = false;
const int EMG_CH1 = 36;
const int EMG_CH2 = 39;
const int EMG_CH3 = 34;
const short channels = 3;
const int sampling_freq = 1000; //Hz
const unsigned int send_buffer_size = channels*500;

union {
  uint8_t bytes[send_buffer_size*sizeof(uint16_t)];
  uint16_t values[send_buffer_size];
} send_buffer;

BluetoothSerial SerialBT;

TaskHandle_t TaskRead;
TaskHandle_t TaskSend;
StreamBufferHandle_t dataStream;
hw_timer_t *readTimer = NULL;


void IRAM_ATTR readTimerISR() {
  uint16_t emg_value;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (device_connected) {
    emg_value = analogRead(EMG_CH1);
    xStreamBufferSendFromISR(dataStream, &emg_value, sizeof(emg_value), &xHigherPriorityTaskWoken);
    // switch context if necessary
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    
    emg_value = analogRead(EMG_CH2);
    xStreamBufferSendFromISR(dataStream, &emg_value, sizeof(emg_value), &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
    }
    
    emg_value = analogRead(EMG_CH3);
    xStreamBufferSendFromISR(dataStream, &emg_value, sizeof(emg_value), &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
          portYIELD_FROM_ISR();
    }
  }
}

// TODO: change to read from external ADC unit if used
void taskReadCode(void * pvParameters) {
  Serial.print("Task Read running on core ");
  Serial.println(xPortGetCoreID());

  /* setup timer alarm, but enable when bluetooth connected */
  readTimer = timerBegin(0, 80, true); // prescaler for 1 us
  timerAttachInterrupt(readTimer, &readTimerISR, true);
  timerAlarmWrite(readTimer, 1000000/sampling_freq, true);

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
      delay(50u);
    } else {
      //ALERT == send_buffer_size to wait to fill send_buffer
      if (!congestedBT) {
        xStreamBufferReceive(dataStream, send_buffer.values, send_buffer_size*sizeof(uint16_t), portMAX_DELAY);
      }
      // Serial.print("Sending: ");
      // Serial.println(send_buffer.values[0]);
      SerialBT.write(send_buffer.bytes, send_buffer_size*sizeof(uint16_t));
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
          xStreamBufferReset(dataStream);
          break;
        case ESP_SPP_CONG_EVT:
          Serial.println("BT congested event");
          // if (param->cong.cong == 0) {
          //   if (congestedBT) {
          //     congestedBT = false;
          //     resendData();
          //   } else {
          //     sendData();
          //   }
          // }
          break;
        case ESP_SPP_WRITE_EVT:
          if (param->write.status == ESP_SPP_SUCCESS) {
            if (param->write.cong == 0) {
              congestedBT = false;
            }
          } else {
            if (param->write.len == -1) {
              Serial.println("Failed to write because of congestion");
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

  dataStream = xStreamBufferCreate(
    channels*sizeof(uint16_t)*sampling_freq*5,
    sizeof(send_buffer.bytes)); /* Alert at the size of send buffer */
  if (dataStream == NULL) {
    Serial.println("Error creating the data stream buffer");
  }
  
  xTaskCreatePinnedToCore(
      taskReadCode,   /* Task function. */
      "Read",         /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &TaskRead,      /* Task handle to keep track of created task */
      0);             /* pin task to core 0 */
  delay(100);

  xTaskCreatePinnedToCore(
      taskSendCode,   /* Task function. */
      "Send",         /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &TaskSend,      /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */
  delay(100);
}

void loop() {
  delay(500);
}