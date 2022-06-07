#ifndef SERIAL_COMUNICATION
    #define SERIAL_COMUNICATION

    #define MSG_END_CHAR '#'

    enum MsgType {
        GET_PID_PARAMS = 65,
        GET_ROT = 66,
        GET_PWM = 67,
        GET_ROT_AND_PWM = 68,

        SET_PHI_KP = 70,
        SET_PHI_KI = 71,
        SET_PHI_KD = 72,
        SET_THETA_KP = 73,
        SET_THETA_KI = 74,
        SET_THETA_KD = 75,
        SET_PSI_KP = 76,
        SET_PSI_KI = 77,
        SET_PSI_KD = 78,

        RESP_PID_PARAMS = 80,
        RESP_ROT = 81,
        RESP_PWM = 82,
        RESP_ROT_AND_PWM = 83,
    };

    typedef void (*serialComHandler)(char msgType, float data[], int len);

    class SerialComunication {
        private:
            char buff[128];
            int currentPos = 0;
            char msgType;
            float data[16];
            serialComHandler handler;
            void parseMsg(int len);

        public:
            SerialComunication() {};
            void begin(serialComHandler handlerFunction);
            void updateBuffer(char c);  
    };

#endif