#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <QByteArray>

#define mHeader     0xb0 //Fix header
#define mWrite      0x01 //Write request
#define mRead       0x02 //Read request
#define mArmed      0x03
#define mDisArmed   0x04
#define mTest       0x05
#define mForward    0xa0
#define mBackward   0xa1
#define mLeft       0xb0
#define mRight      0xb1
#define mPP         0xc0
#define mPD         0xc1
#define mPC         0xc2
#define mPV         0xc3
#define mAC         0xd0
#define mSD         0xd1
#define mSpeak      0xe0
#define mData       0xe1

#define MaxPayload 1024

//message len max is 256, header, command, rw and cheksum total len is 8, therefore payload max len is 248
//max input bluetooth buffer in this chip allows a payload max 0x38

typedef struct {
    uint8_t header;
    uint8_t len;
    uint8_t rw;
    uint8_t command;
    uint8_t data[MaxPayload];
    uint8_t CheckSum[2];
} MessagePack;


class Message
{
public:
    Message();
    bool parse(uint8_t *dataUART, uint8_t size, MessagePack *message);
    uint8_t create_pack(uint8_t RW,uint8_t command, QByteArray dataSend, uint8_t *dataUART);

};

#endif // MESSAGE_H
