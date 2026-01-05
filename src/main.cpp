// Copyright 2024 SAA7706H Attack Project
// SPDX-License-Identifier: MIT
//
// Main entry point - ESP-IDF with serial menu

#include "saa7706h/saa7706h.h"
#include "i2s/i2s_loopback.h"
#include "attack/register_attack.h"
#include "config/config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
const char* kTag = "MAIN";
}

void PrintMenu() {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║           SAA7706H ATTACK TOOL - ESP-IDF                 ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  1. Read all registers                                   ║\n");
    printf("║  2. Apply Linux driver init                              ║\n");
    printf("║  3. Run I2S Attack (44.1kHz 16-bit sweep)                ║\n");
    printf("║  4. Quick I2S test (SEL=0x200011)                        ║\n");
    printf("║  5. Run full SEL sweep                                   ║\n");
    printf("║  6. Play WAV file                                        ║\n");
    printf("║  7. Generate 1kHz tone                                   ║\n");
    printf("║  8. Test I2S1 source                                     ║\n");
    printf("║  9. Test I2S2 source                                     ║\n");
    printf("║  0. Test SPDIF source                                    ║\n");
    printf("║  h. Show this menu                                       ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    printf("\nSelect option: ");
    fflush(stdout);
}

extern "C" void app_main() {
    ESP_LOGI(kTag, "SAA7706H Attack Tool starting...");
    ESP_LOGI(kTag, "WAV Format: 44100 Hz, 16-bit, stereo");
    ESP_LOGI(kTag, "I2S Format: Master, I2S Standard (Philips)");

    // Initialize attack module (includes I2C and I2S init)
    saa7706h::RegisterAttack& attack = saa7706h::GetRegisterAttack();
    esp_err_t err = attack.Init();
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to initialize attack module!");
        return;
    }

    ESP_LOGI(kTag, "All systems initialized!");

    // Show menu
    PrintMenu();

    // Main loop - process serial commands
    while (true) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        printf("%c\n", (char)c);

        switch (c) {
            case '1':
                ESP_LOGI(kTag, "Reading all registers...");
                saa7706h::GetSAA7706H().PrintAllRegisters();
                break;

            case '2':
                ESP_LOGI(kTag, "Applying Linux driver init...");
                saa7706h::GetSAA7706H().ApplyLinuxDriverInit();
                ESP_LOGI(kTag, "Done!");
                break;

            case '3':
                attack.RunI2SAttack();
                break;

            case '4':
                attack.RunQuickI2STest();
                break;

            case '5':
                attack.RunSelAttack();
                break;

            case '6':
                attack.RunWavTest();
                break;

            case '7':
                attack.RunToneTest(1000);
                break;

            case '8':
                attack.TestI2S1();
                break;

            case '9':
                attack.TestI2S2();
                break;

            case '0':
                attack.TestSPDIF();
                break;

            case 'h':
            case 'H':
            case '?':
                PrintMenu();
                break;

            case '\r':
            case '\n':
                // Ignore newlines
                break;

            default:
                printf("Unknown command: %c\n", (char)c);
                PrintMenu();
                break;
        }

        printf("\nSelect option (h for menu): ");
        fflush(stdout);
    }
}
