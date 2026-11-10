#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <tusb.h>

#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_DATA 1

#define MAX_MSG_LENGTH 64
#define DEBUG_MSG_LENGTH 64

enum state {
    // IDLE-tilassa sensoria luetaan ja etsitään uutta liikettä.
    IDLE = 1,
    // SENDING on varattu tunnistetun merkin lähettämistä varten.
    SENDING = 2,
    RECEIVING = 3,
    PLAY_BACK = 4
};

enum state programState = IDLE;

char debugBuffer[DEBUG_MSG_LENGTH];

char rxBuffer[MAX_MSG_LENGTH];
//char txBuffer[MAX_MSG_LENGTH];
//int msgIndex = 0;

char txBuffer[MAX_MSG_LENGTH] = "Testi Viesti\n\0";
int msgIndex = 14;

// Nämä jonot varataan tulevaa lähetystä ja vastaanottoa varten.
QueueHandle_t txQueue;
QueueHandle_t rxQueue;

// Suojaa programState-muuttujan samanaikaisen käytön FreeRTOS-tehtävissä.
SemaphoreHandle_t stateMutex;

static void sensor_task(void *arg) {
    (void)arg;

    // Alustetaan IMU ja otetaan kiihtyvyysanturi sekä gyroskooppi käyttöön.
    if (init_ICM42670() != 0) {
        usb_serial_print((char const *)"Failed to initialize ICM42670\n\0");
        usb_serial_flush();
        vTaskDelete(NULL);
        return;
    }

    if (ICM42670_start_with_default_values() != 0) {
        usb_serial_print((char const *)"Failed to start ICM42670\n\0");
        usb_serial_flush();
        vTaskDelete(NULL);
        return;
    }

    // Edellisiä ja suodatettuja kiihtyvyysarvoja käytetään liikkeen havaitsemiseen.
    float previous_ax, previous_ay;
    float filtered_ax, filtered_ay;
    float previous_gy, filtered_gy;

    const float accelerometer_threshold = 0.2f;
    const float accelerometer_alpha = 0.4f;
    const float gyroscope_threshold = 100.0f;
    const float gyroscope_alpha = 0.4f;

    // Y-akselin liike tuottaa pisteen, X-akselin liike viivan
    // ja riittävän voimakas Y-akselin pyöräytys välilyönnin.
    char detection_symbol;

    // Liikkeen on oltava rauhallinen usean näytteen ajan ennen merkin lähettämistä.
    uint8_t debounce_counter = 0;
    uint8_t rotation_counter = 0;

    // Säilyttää tasaisen 50 ms näytteenottovälin.
    TickType_t last_wake = xTaskGetTickCount();

    // Ensimmäinen näyte alustaa vertailu- ja suodatusarvot.
    if (ICM42670_read_sensor_data(&previous_ax, &previous_ay, NULL,
                                  NULL, &previous_gy, NULL, NULL) != 0)
    {
        usb_serial_print((char const *)"Failed to read initial ICM42670 data\n\0");
        usb_serial_flush();
        vTaskDelete(NULL);
        return;
    }

    filtered_ax = previous_ax;
    filtered_ay = previous_ay;
    filtered_gy = previous_gy;

    while (1) {
        // Suojataan tilan luku ja muutos muilta tehtäviltä.
        xSemaphoreTake(stateMutex, portMAX_DELAY);

        if (programState == IDLE) {
            float current_ax, current_ay;
            float current_gy;

            if (ICM42670_read_sensor_data(&current_ax, &current_ay, NULL, NULL, &current_gy, NULL, NULL) != 0) {
                usb_serial_print((char const *)"Failed to read ICM42670 data\n\0");
                usb_serial_flush();
                xSemaphoreGive(stateMutex);
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
                continue;
            }

            // Suodatus vähentää mittauskohinaa ennen liikkeen tunnistamista.
            filtered_gy = gyroscope_alpha * current_gy + (1 - gyroscope_alpha) * previous_gy;
            float rotation_speed = fabs(filtered_gy);
            filtered_ax = accelerometer_alpha * current_ax + (1 - accelerometer_alpha) * previous_ax;
            filtered_ay = accelerometer_alpha * current_ay + (1 - accelerometer_alpha) * previous_ay;

            float delta_ax = fabs(filtered_ax - previous_ax);
            float delta_ay = fabs(filtered_ay - previous_ay);

            // Tunnistetaan pyöräytys ennen pisteen tai viivan tunnistamista.
            if (rotation_speed > gyroscope_threshold) {
                rotation_counter++;

                if (rotation_counter >= 3) {
                    programState = SENDING;
                    detection_symbol = ' ';

                    snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                    usb_serial_print(debugBuffer);
                    usb_serial_flush();

                    rotation_counter = 0;
                }
            }
            else if (delta_ay > accelerometer_threshold && delta_ay > delta_ax) {
                programState = SENDING;
                detection_symbol = '.';

                snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                usb_serial_print(debugBuffer);
                usb_serial_flush(); 
                
                debounce_counter = 0;
            }
            else if (delta_ax > accelerometer_threshold) {
                programState = SENDING;
                detection_symbol = '-';

                snprintf(debugBuffer, DEBUG_MSG_LENGTH, "Movement detected: '%c'\n", detection_symbol);
                usb_serial_print(debugBuffer);
                usb_serial_flush();

                debounce_counter = 0;
            }
            else {
                rotation_counter = 0;
            }

            // Käytetään suodatettua näytettä seuraavan kierroksen vertailuarvona.
            previous_ax = filtered_ax;
            previous_ay = filtered_ay;
            previous_gy = filtered_gy;
        }
        else if (programState == SENDING) {
            float current_ax, current_ay;
            float current_gy;

            if (ICM42670_read_sensor_data(&current_ax, &current_ay, NULL, NULL, &current_gy, NULL, NULL) != 0) {
                usb_serial_print((char const *)"Failed to read ICM42670 data\n\0");
                usb_serial_flush();
                xSemaphoreGive(stateMutex);
                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
                continue;
            }

            // Päivitetään suodatetut arvot ja lasketaan liikkeen voimakkuus.
            filtered_ax = accelerometer_alpha * current_ax + (1 - accelerometer_alpha) * previous_ax;
            filtered_ay = accelerometer_alpha * current_ay + (1 - accelerometer_alpha) * previous_ay;
            float delta_ax = fabs(filtered_ax - previous_ax);
            float delta_ay = fabs(filtered_ay - previous_ay);
            filtered_gy = gyroscope_alpha * current_gy + (1 - gyroscope_alpha) * previous_gy;
            float rotation_speed = fabs(filtered_gy);

            // Lähetetään merkki vasta, kun laite on ollut rauhallinen viisi näytettä.
            if (delta_ax < accelerometer_threshold && delta_ay < accelerometer_threshold && rotation_speed < gyroscope_threshold) {
                debounce_counter++;
                if (debounce_counter >= 5) {
                    if (xQueueSend(txQueue, &detection_symbol, 0) == pdPASS) {
                        programState = IDLE;
                        debounce_counter = 0;
                    }
                    else {
                        usb_serial_print((char const *)"Failed to send symbol to txQueue\n\0");
                        usb_serial_flush();
                    }
                }
            }
            else {
                debounce_counter = 0;
            }
            previous_ax = filtered_ax;
            previous_ay = filtered_ay;
            previous_gy = filtered_gy;
        }

        xSemaphoreGive(stateMutex);

        // Odotetaan seuraavaan näytteenottohetkeen.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

// Painikkeen keskeytyskäsittelijä.
void button_callback(uint gpio, uint32_t events) {
    programState = SENDING;
}

// Hoitaa USB-pinon.
void usb_task(void *arg) {
    (void)arg;
    while (1) {
        tud_task();
        // Ei vTaskDelay
    }
}

// Lähettää viesti bufferin sisällön.
void transmit_task(void *arg) {
    (void)arg;

    while (1) {
        semaphoreTake(stateMutex, portMAX_DELAY);
        if (programState == SENDING) {
            tud_cdc_n_write(CDC_ITF_DATA, txBuffer, sizeof(char)* msgIndex);
            tud_cdc_n_write_flush(CDC_ITF_DATA);
            // Resetoi viestin indeksi seuraavaa viestiä varten
            //msgIndex = 0; 
            programState = IDLE;
        }
        semaphoreGive(stateMutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Vastaanottaa datan.
void tud_cdc_rx_cb(uint8_t itf) {

    // Vastaan otetaan itf:stä riippumatta, jotta bufferit tyhjenee.
    uint32_t count = tud_cdc_n_read(itf, rxBuffer, sizeof(rxBuffer));

    // Käsitellään vain CDC_ITF_DATA-liitännän kautta tuleva data.
    if (itf == CDC_ITF_DATA) {
        // Käsittele data täällä (esim. tallenna puskuriin tai lähetä toiselle tehtävälle).

        // Vastataan kohteliaasti OK
        tud_cdc_n_write(itf, (uint8_t const *) "OK\n", 3);
        tud_cdc_n_write_flush(itf);
    }
}

int main() {

    // Alustaa HAT-laitteen ja sen oletusarvoisen I2C-väylän.
    init_hat_sdk();
    sleep_ms(300);

    TaskHandle_t handleSensorTask = NULL;

    // Alustaa painikkeen ja määrittää sen keskeytyskäsittelijän.
    gpio_init(BUTTON1);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, button_callback);

    // Alustetaan USB yhteys ennen debug viestejä
    tusb_init();
    usb_serial_init();

    // Mutex estää samanaikaiset ristiriitaiset programState-muutokset.
    stateMutex = xSemaphoreCreateMutex();

    if (stateMutex == NULL) {
        usb_serial_print((char const *)"Failed to create mutex\n\0");
        usb_serial_flush();
        return 0;
    }

    // Varataan jonot tulevia viestinvälitystehtäviä varten.
    txQueue = xQueueCreate(64, sizeof(char));
    rxQueue = xQueueCreate(64, sizeof(char));

    if (txQueue == NULL || rxQueue == NULL)
    {
        printf("Failed to create queues\n");
        return 0;
    }
    
    // Sensoritehtävä lukee IMU:ta ja tunnistaa liikkeet. Prioriteetti 2
    // määrittää tehtävän ajamisen suhteessa muihin FreeRTOS-tehtäviin.
    BaseType_t sensorTaskResult = xTaskCreate(
        sensor_task,
        "sensor",           // Tehtävän nimi
        DEFAULT_STACK_SIZE, // Tehtävän pinon koko
        NULL,               // Tehtävän parametrit
        2,                  // Tehtävän prioriteetti
        &handleSensorTask
    ); // Tehtävän kahva

    if (sensorTaskResult != pdPASS){
        usb_serial_print((char const *)"Sensor task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t usbTaskResult = xTaskCreate(
        usb_task,
        "usb",
        DEFAULT_STACK_SIZE,
        NULL,
        3,                  // Suurempi prioriteetti kuin muilla tehtävillä
        NULL
    );
    if (usbTaskResult != pdPASS){
        usb_serial_print((char const *)"USB task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    BaseType_t transmitTaskResult = xTaskCreate(
        transmit_task,
        "transmit",
        DEFAULT_STACK_SIZE,
        NULL,
        2,
        NULL
    );
    if (transmitTaskResult != pdPASS){
        usb_serial_print((char const *)"Transmit task creation failed\n\0");
        usb_serial_flush();
        return 0;
    }

    // Käynnistyksen jälkeen FreeRTOS Scheduler hallitsee tehtävien ajoitusta.
    vTaskStartScheduler();

    // Schedulerin ei normaalisti pitäisi palata tähän.
    return 0;
}
