#pragma once

#define RADIO_CHANNELS_NUMBER 8 // max value = 8

enum RadioInput
{
  ARM,
  FLY_MODE,
  YAW,
  THROTTLE,
  PITCH,
  ROLL,
  PARAM_ADJUST_1,
  PARAM_ADJUST_2
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
        void printInput(int number);
};