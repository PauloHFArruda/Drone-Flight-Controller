#include "Radio.h"
#include <Arduino.h>

void Radio::begin() {
    PCICR |= (1 << PCIE0);   // enable PCMSK0 scan - PORT B OF THE ARDUINO MEGA
    PCMSK0 |= (1 << PCINT0); // Set pin D53 trigger an interrupt on state change - CHANNEL 1.
    PCMSK0 |= (1 << PCINT1); // Set pin D52 trigger an interrupt on state change - CHANNEL 2.
    PCMSK0 |= (1 << PCINT2); // Set pin D51 trigger an interrupt on state change - CHANNEL 3.
    PCMSK0 |= (1 << PCINT3); // Set pin D50 trigger an interrupt on state change - CHANNEL 4.
    PCMSK0 |= (1 << PCINT4); // Set pin D10 trigger an interrupt on state change - CHANNEL 5.
    PCMSK0 |= (1 << PCINT5); // Set pin D11 trigger an interrupt on state change - CHANNEL 6.
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

void Radio::printInputs() {
    for (int i = 0; i < RADIO_CHANNELS_NUMBER; i++)
    {
        Serial.print("Channel_");
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(inputs[i]);
        Serial.print(" ");
    }
}