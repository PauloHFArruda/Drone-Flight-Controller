#include "SerialComunication.h"
#include "utils.h"

#define SEPARATOR ','

void SerialComunication::parseMsg(int len) {
    char msgType = buff[0];
    float data[16];
    int currentData = 0;

    buff[len-1] = '\0';

    if (len > 1) {
        int aux = 1;

        for (int i = 1; i < len; i++) {
            if (buff[i] == SEPARATOR) {
                buff[i] = '\0';
                data[currentData++] = strToFloat(buff+aux);
                aux = i + 1;
            }
        }   
        data[currentData++] = strToFloat(buff+aux);
    }

    this->handler(msgType, data, currentData); 
}

void SerialComunication::begin(serialComHandler handlerFunction) {
    handler = handlerFunction;
    currentPos = 0;
}

void SerialComunication::updateBuffer(char c) {
    buff[currentPos++] = c;

    if (c == MSG_END_CHAR) {
        parseMsg(currentPos);
        currentPos = 0;
    }
}