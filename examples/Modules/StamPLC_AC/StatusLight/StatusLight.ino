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
    /* Set status light to red */
    stamplc_ac.setStatusLight(1, 0, 0);
    printf("Set M5StamPLC-AC status light to red\n");
    delay(1000);

    /* Set status light to green */
    stamplc_ac.setStatusLight(0, 1, 0);
    printf("Set M5StamPLC-AC status light to green\n");
    delay(1000);

    /* Set status light to blue */
    stamplc_ac.setStatusLight(0, 0, 1);
    printf("Set M5StamPLC-AC status light to blue\n");
    delay(1000);

    /* Set status light to white */
    stamplc_ac.setStatusLight(1, 1, 1);
    printf("Set M5StamPLC-AC status light to white\n");
    delay(1000);

    /* Set status light to black */
    stamplc_ac.setStatusLight(0, 0, 0);
    printf("Set M5StamPLC-AC status light to black\n");
    delay(1000);
}
