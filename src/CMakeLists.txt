# SAA7706H Register Attack - ESP-IDF Component
# Copyright 2024 SAA7706H Attack Project

idf_component_register(
    SRCS
        "main.cpp"
        "saa7706h/saa7706h.cpp"
        "i2s/i2s_loopback.cpp"
        "attack/register_attack.cpp"
    INCLUDE_DIRS
        "../include"
    REQUIRES
        driver
        freertos
        esp_timer
    EMBED_FILES
        "audio/no.wav"
)
