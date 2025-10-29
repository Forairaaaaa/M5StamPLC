/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5StamPLC.h>

M5StamPLC_AC stamplc_ac;

void setup()
{
    /* Init M5StamPLC */
    M5StamPLC.begin();

    /* Init M5StamPLC-AC */
    while (!stamplc_ac.begin()) {
        printf("M5StamPLC-AC init failed, retry in 1s...\n");
        delay(1000);
    }
}

void loop()
{
    static bool relay_state = false;

    /* Toggle M5StamPLC-AC relay state */
    relay_state = !relay_state;
    stamplc_ac.writeRelay(relay_state);
    printf("Write M5StamPLC-AC Relay to %s\n", stamplc_ac.readRelay() ? "ON" : "OFF");

    delay(1000);
}
