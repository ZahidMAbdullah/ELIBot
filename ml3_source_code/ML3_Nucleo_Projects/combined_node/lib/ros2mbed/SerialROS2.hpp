#ifndef _SERIALROS2
#define _SERIALROS2

#define MAXIMUM_BUFFER_SIZE  32

#include "mbed.h"
#include <chrono>

struct DataRecv{
    int32_t num_recv;
    char data[MAXIMUM_BUFFER_SIZE] = {0};
};

char** split(char sep, char text[]);
void freeSplit(char** t);

class SerialROS2{
    public:
        PinName tx;
        PinName rx;
        Ticker  send_timer; 

        SerialROS2(PinName _tx, PinName _rx, uint32_t _baud_rate=9600): 
                    tx(_tx), rx(_rx), pc(_tx, _rx, _baud_rate),
                    baud_rate(_baud_rate){}

        BufferedSerial pc;

        DataRecv recv();
        void recvVals(char sep='/');
        void send(void* data_send, uint32_t size);
        void sendVals();
        bool init();

        void attach(void (*func)(), chrono::milliseconds t);
        void (*recvCallback)(float* data);

        uint32_t baud_rate;
};

#endif