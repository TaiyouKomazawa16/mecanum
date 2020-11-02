//RosPlainSerial message for Imu.

#ifndef IMU_MSG_H_
#define IMU_MSG_H_

#include <stdint.h>

#include <PlainSerial.h>

class ImuMsg : public StructMem
{
public:
    typedef struct ImuDataType{
            struct{
                float x;
                float y;
                float z;
                float w;
            }pose;
            struct{
                int16_t x;
                int16_t y;
                int16_t z;
            }acc;
            struct{
                float x;
                float y;
                float z;
            }gyro;
    }imu_data_t;

    imu_data_t data;

    virtual inline int size() { return MAX_P_IMU_MSG_SIZE; }
    virtual inline uint8_t *ptr() { 
        _data.imu = data;
        return _data.ptr; 
    }

    virtual inline message_t msg_id()
    {
        return (message_t)IMU_MSG_ID;
    }

private:
    enum
    {
        IMU_MSG_ID = 127,
        MAX_P_IMU_MSG_SIZE = 34, //byte
    };

    typedef union _DataFrameType
    {
        uint8_t ptr[MAX_P_IMU_MSG_SIZE];
        imu_data_t imu;
    } _data_frame_t;

    _data_frame_t _data;
};

#endif
