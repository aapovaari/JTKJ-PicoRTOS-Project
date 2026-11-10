#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1

enum state
{
    // IDLE-tilassa sensoria luetaan ja etsitään uutta liikettä.
    IDLE = 1,
    // SENDING on varattu tunnistetun merkin lähettämistä varten.
    SENDING = 2,
    RECEIVING = 3,
    PLAY_BACK = 4
};

enum state programState = IDLE;

typedef struct {
    uint32_t frequency;
    uint32_t length; // millisekuntia
} Note;

// Super Mario 1-Up -ääni
const Note kuittausmelodia[] = {
    {659, 133},  // E5
    {784, 133},  // G5
    {1319, 133}, // E6
    {1046, 133}, // C6
    {1175, 133}, // D6
    {1568, 133}  // G6
};

// Nämä jonot varataan tulevaa lähetystä ja vastaanottoa varten.
QueueHandle_t txQueue;
QueueHandle_t rxQueue;

// Suojaa programState-muuttujan samanaikaisen käytön FreeRTOS-tehtävissä.
SemaphoreHandle_t stateMutex;

// Funktio, jolla saadaan tämänhetkinen tila merkkijonona.
const char* get_state_string(enum state s) {
    switch (s) {
        case IDLE:       return "IDLE";
        case SENDING:    return "SENDING";
        case RECEIVING:  return "RECEIVING";
        case PLAY_BACK:  return "PLAYBACK";
    }
    return "UNKNOWN";
}

void ui_task(void *pvParameters) {
    char character;
    char msg_buffer[64];    
    uint8_t index = 0;
    bool msg_ended = false;

    int len_kuittausmelodia = sizeof(kuittausmelodia) / sizeof(kuittausmelodia[0]);

    // Oma muuttuja tilan tallentamiseen turvallisesti
    enum state current_state = IDLE;

    // Alustetaan näyttö, summeri ja ledi sekä tyhjennetään viestipuskuri roskadatasta
    init_display();
    init_buzzer();
    init_led();
    msg_buffer[0] = '\0';

    while (1) {
        xQueueReceive(rxQueue, &character, portMAX_DELAY);

        // Lukitaan tila kymmenen tikin ajaksi, kun luetaan tila. 
        // Jos epäonnistuu, current_state pysyy samana kuin aikaisemmin.
        if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
            current_state = programState;
            xSemaphoreGive(stateMutex);
        }
        
        // Lähetetään summerilla melodia, kun jonoon saapuu oman lähetyksen lopetusmerkki
        if (character == 0x04) {
            // Käydään läpi kuittausmelodian kaikki nuotit
            for (int i = 0; i < len_kuittausmelodia; i++) {
                buzzer_play_tone(kuittausmelodia[i].frequency, kuittausmelodia[i].length - 20);
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            // Muutetaan tila IDLE:ksi
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                programState = IDLE;
                current_state = IDLE; // Päivitetään paikallinen kopio
                xSemaphoreGive(stateMutex);
            }

            // Päivitetään näyttö
            clear_display();
            write_text_xy(0, 0, get_state_string(current_state));
            write_text_xy(0, 16, msg_buffer);
        }
        
        // Lopetusmerkki
        else if (character == '\n') {
            // Muutetaan tila PLAY_BACK:ksi toiston ajaksi
            if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
                programState = PLAY_BACK;
                current_state = PLAY_BACK;
                xSemaphoreGive(stateMutex);
            }
            
            // Lisätään * viestin päättymisen merkiksi, jos puskurissa on tilaa
            if (index < 62) {
                msg_buffer[index] = '*';
                msg_buffer[index + 1] = '\0';
            }
            
            // Päivitetään näyttö
            clear_display();
            write_text_xy(0, 0, get_state_string(current_state));
            write_text_xy(0, 16, msg_buffer);

            // Toistetaan vastaanotettu viesti ledillä ja summerilla.
            for (int i = 0; i < index; i++) {
                char c = msg_buffer[i];
                
                // Piste
                if (c == '.') {
                    set_led_status(true);
                    buzzer_play_tone(641, 100);
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                // Viiva
                else if (c == '-') {
                    set_led_status(true);
                    buzzer_play_tone(641, 300);
                    set_led_status(false);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                // Välilyönti
                else if (c == ' ') {
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
            }
            
            // Muutetaan tila takaisin IDLE:ksi
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                programState = IDLE;
                current_state = IDLE;
                xSemaphoreGive(stateMutex);
            }
            
            // Nollataan puskuri ja indeksi
            msg_buffer[0] = '\0';
            index = 0;
        }

        else {
            // Muutetaan tila RECEIVING:ksi, jos se ei vielä ole.
            // Ilman if-lausetta tämä yrittäisi muuttaa tilaa jokaisella kierroksella.
            if (current_state != RECEIVING) {
                if (xSemaphoreTake(stateMutex, 10) == pdTRUE) {
                    programState = RECEIVING;
                    current_state = RECEIVING;
                    xSemaphoreGive(stateMutex);
                }
            }
            
            // Lisätään merkki puskuriin ja lisätään nollatavu perään
            msg_buffer[index] = character;
            msg_buffer[index + 1] = '\0';

            // Nostetaan indeksiä, mutta varmistetaan ettei tule ylivuotoa
            if (index >= 62) index = 0;
            else index++;
            
            // Päivitetään näyttö
            clear_display();
            write_text_xy(0, 0, get_state_string(current_state));
            write_text_xy(0, 16, msg_buffer);
        }
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

    // IMU:n tunnistus ja perusasetusten määrittely tehdään sensoritehtävässä,
    // koska sensoridataa käytetään tässä tehtävässä koko ohjelman ajan.
    if (init_ICM42670() != 0)
    {
        printf("Failed to initialize ICM42670\n");
        vTaskDelete(NULL); // Poistetaan tehtävä, koska anturin alustaminen epäonnistui.
        return;
    }

    // Käynnistää kiihtyvyysanturin ja gyroskoopin SDK:n oletusasetuksilla.
    if (ICM42670_start_with_default_values() != 0)
    {
        printf("Failed to start ICM42670\n");
        vTaskDelete(NULL); // Poistetaan tehtävä, koska anturin käynnistäminen epäonnistui.
        return;
    }

    float previous_ax, previous_ay;

    // Kynnysarvo on g-yksiköissä. Sitä suurempi näytteiden välinen muutos
    // tulkitaan liikkeeksi.
    const float threshold = 0.3f;

    // is_moving estää saman jatkuvan liikkeen tulkitsemisen useaksi merkiksi.
    bool is_moving = false;

    // Y-akselin liike tuottaa pisteen ja X-akselin liike viivan.
    char detection_symbol;

    // Liikkeen pitää olla poissa usean näytteen ajan ennen uuden liikkeen
    // hyväksymistä. Tämä vähentää mittauskohinan aiheuttamia vääriä tunnistuksia.
    uint8_t debounce_counter = 0;

    // Ensimmäinen näyte toimii vertailukohtana seuraavalle mittaukselle.
    if (ICM42670_read_sensor_data(&previous_ax, &previous_ay, NULL, NULL, NULL, NULL, NULL) != 0)
    {
        printf("Failed to read initial ICM42670 data\n");
        vTaskDelete(NULL); // Poistetaan tehtävä, koska anturin lukeminen epäonnistui.
        return;
    }

    while (1)
    {
        // Pidetään tila lukittuna sensorin lukemisen ja mahdollisen
        // tilamuutoksen ajan.
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        if (programState == IDLE)
        {
            float current_ax, current_ay;
            // Sensorin näytteet luetaan 50 millisekunnin välein.
            if (ICM42670_read_sensor_data(&current_ax, &current_ay, NULL, NULL, NULL, NULL, NULL) != 0)
            {
                printf("Failed to read ICM42670 data\n");
                xSemaphoreGive(stateMutex);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Peräkkäisten näytteiden erotus kuvaa havaittua liikettä.
            float delta_ax = fabs(current_ax - previous_ax);
            float delta_ay = fabs(current_ay - previous_ay);

            // Liikkeen tunnistuksen debug-tulostus.
            // printf("Current ax: %f, Current ay: %f\n", current_ax, current_ay);
            // printf("Delta ax: %f, Delta ay: %f\n", delta_ax, delta_ay);

            if (!is_moving)
            {
                // Liikkeen alkaessa muodostetaan yksi character ja asetetaan
                // tila aktiiviseksi, jotta samaa liikettä ei tunnisteta uudelleen.
                if (delta_ay > threshold)
                {
                    is_moving = true;
                    detection_symbol = '.';

                    // Symbolin tunnistuksen debug-tulostus.
                    // printf("Movement detected: %c\n", detection_symbol);

                    debounce_counter = 0;
                }
                else if (delta_ax > threshold)
                {
                    is_moving = true;
                    detection_symbol = '-';

                    // Symbolin tunnistuksen debug-tulostus.
                    // printf("Movement detected: %c\n", detection_symbol);

                    debounce_counter = 0;
                }
            }
            else
            {
                // Liike kuitataan vasta viiden peräkkäisen rauhallisen näytteen
                // jälkeen. Näytteenottovälillä 50 ms tämä vastaa noin 250 ms:ia.
                if (delta_ax < threshold && delta_ay < threshold)
                {
                    debounce_counter++;
                    if (debounce_counter >= 5)
                    {
                        is_moving = false;
                        debounce_counter = 0;
                    }
                }
                else
                {
                    debounce_counter = 0;
                }
            }
            // Nykyinen näyte on seuraavan kierroksen vertailuarvo.
            previous_ax = current_ax;
            previous_ay = current_ay;
        }
        xSemaphoreGive(stateMutex);
        // Viive vapauttaa suorittimen ja määrittää näytteenottotaajuudeksi
        // noin 20 mittausta sekunnissa.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main()
{
    // Alustaa Pico SDK:n standarditulostuksen USB:n kautta.
    stdio_init_all();

    // Odotetaan USB-monitorin yhdistämistä, jotta käynnistyksen viestejä
    // ei menetetä ennen kuin Serial Monitor on valmis vastaanottamaan niitä.
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }

    // Alustaa HAT-laitteen ja sen oletusarvoisen I2C-väylän.
    init_hat_sdk();
    sleep_ms(300);

    TaskHandle_t handleSensorTask = NULL;

    // Varataan jonot tulevia viestinvälitystehtäviä varten.
    txQueue = xQueueCreate(64, sizeof(char));
    rxQueue = xQueueCreate(64, sizeof(char));

    if (txQueue == NULL || rxQueue == NULL)
    {
        printf("Failed to create queues\n");
        return 0;
    }

    // Mutex estää samanaikaiset ristiriitaiset programState-muutokset.
    stateMutex = xSemaphoreCreateMutex();

    if (stateMutex == NULL)
    {
        printf("Failed to create mutex\n");
        return 0;
    }

    // Sensoritehtävä lukee IMU:ta ja tunnistaa liikkeet. Prioriteetti 2
    // määrittää tehtävän ajamisen suhteessa muihin FreeRTOS-tehtäviin.
    BaseType_t result = xTaskCreate(sensor_task,
                                    "sensor",           // Tehtävän nimi
                                    DEFAULT_STACK_SIZE, // Tehtävän pinon koko
                                    NULL,               // Tehtävän parametrit
                                    2,                  // Tehtävän prioriteetti
                                    &handleSensorTask); // Tehtävän kahva

    if (result != pdPASS)
    {
        printf("Sensor task creation failed\n");
        return 0;
    }

    // Käynnistyksen jälkeen FreeRTOS Scheduler hallitsee tehtävien ajoitusta.
    vTaskStartScheduler();

    // Schedulerin ei normaalisti pitäisi palata tähän.
    return 0;
}
