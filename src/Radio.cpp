#include "Radio.h"
#include <Arduino.h>

void Radio::begin() {
    PCICR |= (1 << PCIE0);   // enable PCMSK0 scan - PORT B OF THE ARDUINO MEGA
    for (int i = 0; i < RADIO_CHANNELS_NUMBER; i++)
        PCMSK0 |= (1 << i); // Set pin to trigger an interrupt on state change - CHANNEL i.
}

void Radio::update() {
  current_count = micros();
  for (int i = 0; i < RADIO_CHANNELS_NUMBER; i++)
  {
    if (PINB & (1 << i))
    {
        if (channels[i].last_state == 0)
            {
                channels[i].last_state = 1;
                channels[i].counter = current_count;
            }
        }
        else if (channels[i].last_state == 1)
        {
            channels[i].last_state = 0;
            inputs[i] = current_count - channels[i].counter;
        }
    }
}

void Radio::printInput(int number) {
    Serial.print("Channel_");
    Serial.print(number + 1);
    Serial.print(":");
    Serial.print(inputs[number]);
    Serial.print(" ");
}

void Radio::printInputs() {
    for (int i = 0; i < RADIO_CHANNELS_NUMBER; i++)
    {
        this->printInput(i);
    }
}