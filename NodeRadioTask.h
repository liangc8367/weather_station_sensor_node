/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TASKS_NODERADIOTASKTASK_H_
#define TASKS_NODERADIOTASKTASK_H_

#include "stdint.h"

struct IoTSensorData {
    int32_t     _cpuTemp;
    uint32_t    _cpuVolt;

    int32_t     _bme280Temp;
    uint32_t    _bme280Pressure;
    uint32_t    _bme280Humidity;
};

struct IoTSensorConfig {
    uint16_t    _sampleRate;
};

/* Fixed target address for IoT Hub: "IoTHub" */
#define RADIO_CONCENTRATOR_ADDRESS     0x55AA496F54487562
#define RADIO_EASYLINK_MODULATION     EasyLink_Phy_Custom

#define RADIO_PACKET_TYPE_ACK_PACKET             1
#define RADIO_PACKET_TYPE_SENSOR_DATA            2
#define RADIO_PACKET_TYPE_SENSOR_CONF            3

struct PacketHeader {
    uint64_t _sourceAddress;
    uint8_t _packetType;
};
#define OTA_PACKET_HEADER_SIZE  9

struct AckPacket {
    struct PacketHeader _header;
    uint32_t            _result;
};

struct IoTSensorPacket {
    struct PacketHeader     _header;
    struct IoTSensorData    _sensorData;
};

struct IoTSensorConfigPacket {
    struct PacketHeader     _header;
    struct IoTSensorConfig  _sensorConfig;
};

enum NodeRadioOperationStatus {
    NodeRadioStatus_Success,
    NodeRadioStatus_Failed,
    NodeRadioStatus_FailedNotConnected,
};

/* Initializes the NodeRadioTask and creates all TI-RTOS objects */
void NodeRadioTask_init(void);

/* Send IoT sensor data to hub, blocking call. */
enum NodeRadioOperationStatus NodeRadioTask_sendSensorData(const struct IoTSensorData *sensorData);
enum NodeRadioOperationStatus NodeRadioTask_sendSensorConfig(const struct IoTSensorConfig *sensorConfig);

/* Device MAC address */
uint8_t nodeIeeeAddr[8];

#endif /* TASKS_NODERADIOTASKTASK_H_ */
