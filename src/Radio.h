#pragma once

#define RADIO_CHANNELS_NUMBER 6

enum RadioInput
{
  ARM,
  FLY_MODE,
  YAW,
  THROTTLE,
  PITCH,
  ROLL,
};

typedef struct
{
  char last_state;
  unsigned counter;
} channel_data;

class Radio {
    private:
        unsigned int current_count;
        channel_data channels[RADIO_CHANNELS_NUMBER];
    
    public:
        int inputs[RADIO_CHANNELS_NUMBER];
        void begin();
        void update();
        void printInputs();
};