/*
 * Wireless layer for IoT Sensor Node
 *
 * - 2018/5/4, liangc, rewrite based on TI's example.
 */

/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */ 
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "NodeRadioTask.h"

/***** Defines *****/
#define NODERADIO_TASK_STACK_SIZE 1024
#define NODERADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                 0xFFFFFFFF
#define RADIO_EVENT_SEND_SENSOR_DATA       (uint32_t)(1 << 0)
#define RADIO_EVENT_SEND_SENSOR_CONFIG       (uint32_t)(1 << 2)
#define RADIO_EVENT_SEND_FAIL           (uint32_t)(1 << 3)

#define NODERADIO_MAX_RETRIES 2
#define NORERADIO_ACK_TIMEOUT_TIME_MS (160)

/***** Type declarations *****/
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};


/***** Variable declarations *****/
uint8_t nodeIeeeAddr[8];
uint64_t iotHubAddr = 0;
static Task_Params nodeRadioTaskParams;
Task_Struct nodeRadioTask;        /* not static so you can see in ROV */
static uint8_t nodeRadioTaskStack[NODERADIO_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;
Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;
Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;
static struct RadioOperation currentRadioOperation;

static const struct IoTSensorData *sensorDataToSend = NULL;

static void serializePacketHeader(uint8_t packetType)
{
    memcpy(currentRadioOperation.easyLinkTxPacket.payload, nodeIeeeAddr, sizeof(nodeIeeeAddr));
    currentRadioOperation.easyLinkTxPacket.payload[sizeof(nodeIeeeAddr)] = packetType;
}

static void serializeSensorData(const struct IoTSensorData *sensorData)
{
    memcpy(currentRadioOperation.easyLinkTxPacket.payload + OTA_PACKET_HEADER_SIZE,
           sensorData, sizeof(struct IoTSensorData));
    currentRadioOperation.easyLinkTxPacket.len = OTA_PACKET_HEADER_SIZE + sizeof(struct IoTSensorData);
}

static void sendSensorData()
{
    /* Set destination address in EasyLink API */
    memcpy(currentRadioOperation.easyLinkTxPacket.dstAddr, &iotHubAddr, 8);

    /* Setup Tx packet */
    serializePacketHeader(RADIO_PACKET_TYPE_SENSOR_DATA);
    serializeSensorData(sensorDataToSend);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = NODERADIO_MAX_RETRIES;
    currentRadioOperation.ackTimeoutMs = NORERADIO_ACK_TIMEOUT_TIME_MS;
    currentRadioOperation.retriesDone = 0;
    // TODO: clean this piece, we don't need ack.
    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(NORERADIO_ACK_TIMEOUT_TIME_MS));

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }
#if defined(Board_DIO30_SWPWR)
    /* this was a blocking call, so Tx is now complete. Turn off the RF switch power */
    PIN_setOutputValue(blePinHandle, Board_DIO30_SWPWR, 0);
#endif

    Semaphore_post(radioResultSemHandle);
}

static void nodeRadioTaskFunction(UArg arg0, UArg arg1)
{
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

    easyLink_params.ui32ModType = RADIO_EASYLINK_MODULATION;

    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success){
        System_abort("EasyLink_init failed");
    }

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Set the filter to device's self MAC address */
    if (EasyLink_enableRxAddrFilter(nodeIeeeAddr, 8, 1) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_enableRxAddrFilter failed");
    }

    /* Enter main task loop */
    while (true)
    {
        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we should send ADC data */
        if (events & RADIO_EVENT_SEND_SENSOR_DATA) {
            sendSensorData();
        }

    }
}

/* Return 64bit MAC address (unique per chip) */
uint8_t * NodeRadioTask_getMACAddress()
{
    return nodeIeeeAddr;
}

void NodeRadioTask_init(void)
{
    if(EasyLink_getIeeeAddr(nodeIeeeAddr) != EasyLink_Status_Success) {
        System_abort("Unable to get device MAC address!\n");
    }


    /* Initialize IoT Hub Address */
    iotHubAddr = RADIO_CONCENTRATOR_ADDRESS;

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem, 0, &semParam);
    radioResultSemHandle = Semaphore_handle(&radioResultSem);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the radio protocol task */
    Task_Params_init(&nodeRadioTaskParams);
    nodeRadioTaskParams.stackSize = NODERADIO_TASK_STACK_SIZE;
    nodeRadioTaskParams.priority = NODERADIO_TASK_PRIORITY;
    nodeRadioTaskParams.stack = &nodeRadioTaskStack;
    Task_construct(&nodeRadioTask, nodeRadioTaskFunction, &nodeRadioTaskParams, NULL);
}

/* Send IoT sensor data to hub, blocking call. */
enum NodeRadioOperationStatus NodeRadioTask_sendSensorData(const struct IoTSensorData *sensorData)
{
    // Acquire send lock, and signal radio task to send data.
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);
    sensorDataToSend = sensorData;
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_SENSOR_DATA);

    // Wait until radio task completed sending, and release send lock.
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);
    Semaphore_post(radioAccessSemHandle);

    return NodeRadioStatus_Success;
}
