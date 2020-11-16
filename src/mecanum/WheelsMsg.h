//RosPlainSerial message for Wheels speed.

#ifndef WHEELS_MSG_H_
#define WHEELS_MSG_H_

#include <stdint.h>

#include <PlainSerial.h>

class WheelsMsg : public StructMem
{
public:
    typedef struct WheelsDataType{
            struct{
                float fl;
                float rl;
                float rr;
                float fr;
            };
    }wheels_data_t;

    wheels_data_t wheel;

    virtual inline int size() { return MAX_P_WHEELS_MSG_SIZE; }
    virtual inline uint8_t *ptr() { 
        _data.wheels = wheel;
        return _data.ptr; 
    }

    virtual inline message_t msg_id()
    {
        return (message_t)WHEELS_MSG_ID;
    }

private:
    enum
    {
        WHEELS_MSG_ID = 128,
        MAX_P_WHEELS_MSG_SIZE = 16, //byte
    };

    typedef union _DataFrameType
    {
        uint8_t ptr[MAX_P_WHEELS_MSG_SIZE];
        wheels_data_t wheels;
    } _data_frame_t;

    _data_frame_t _data;
};

#endif
