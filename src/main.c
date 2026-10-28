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

// Nämä jonot varataan tulevaa lähetystä ja vastaanottoa varten.
QueueHandle_t txQueue;
QueueHandle_t rxQueue;

// Suojaa programState-muuttujan samanaikaisen käytön FreeRTOS-tehtävissä.
SemaphoreHandle_t stateMutex;

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
                // Liikkeen alkaessa muodostetaan yksi merkki ja asetetaan
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
