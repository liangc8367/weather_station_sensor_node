/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

/***** Includes *****/
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
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "NodeTask.h"
#include "NodeRadioTask.h"

/* EasyLink API Header file for MAC address */
#include "easylink/EasyLink.h"

/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* Not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* Not static so you can see in ROV */

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);


/***** Function definitions *****/
void NodeTask_init(void)
{
    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static Display_Handle   display;
static I2C_Handle       i2c;    // i2c handle for BME280

static PIN_Handle       buttonPinHandle;
static PIN_State buttonPinState;
/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};
static void buttonCallback(PIN_Handle handle, PIN_Id pinId);

const unsigned int      sample_interval_min = 2; // in seconds
const unsigned int      sample_interval_max = 500; // in seconds
static unsigned int     sample_interval = sample_interval_min; // in seconds

#include "bme280.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)

/* Board specific I2C addresses */
#define Board_TMP_ADDR          (0x40)
#define Board_SENSORS_BP_TMP_ADDR Board_TMP_ADDR
#define Board_BME280_ADDR       (0x76)
#define Board_I2C_BME280        CC1310_LAUNCHXL_I2C0

// BME280 HAL
void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    usleep(period * 1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    I2C_Transaction i2cTransaction;
    uint8_t txBuffer = reg_addr;

    i2cTransaction.slaveAddress = Board_BME280_ADDR;
    i2cTransaction.writeBuf = &txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = len;

    if(!I2C_transfer(i2c, &i2cTransaction)) {
        Display_printf(display, 0, 0, "I2C Bus fault during reading\n");
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

#define BME280_TXBUF_SIZE  20

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[BME280_TXBUF_SIZE];
    uint8_t rxBuffer;

    if(len > (BME280_TXBUF_SIZE - 1)) {
        Display_printf(display, 0, 0, "Buffer to long for I2C, buffer size = %d\n", len);
        return BME280_E_INVALID_LEN;
    }

    // compose write command
    txBuffer[0] = reg_addr;
    memcpy(txBuffer+1, reg_data, len);

    i2cTransaction.slaveAddress = Board_BME280_ADDR;
    i2cTransaction.writeBuf = &txBuffer;
    i2cTransaction.writeCount = len+1;
    i2cTransaction.readBuf = &rxBuffer;
    i2cTransaction.readCount = 1;

    if(!I2C_transfer(i2c, &i2cTransaction)) {
        Display_printf(display, 0, 0, "I2C Bus fault during reading\n");
        rslt = BME280_E_COMM_FAIL;
    }
    return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        Display_printf(display, 0, 0, "temp = %ld, pressure = %ld, humidity = %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void GetCPUBatteryAndTemp()
{
    int32_t temp = AONBatMonTemperatureGetDegC();
    uint32_t vol = AONBatMonBatteryVoltageGet();
    Display_printf(display, 0, 0, "CPU Temperature = %i, Voltage = %.2f\n", temp, vol/256.0);
}


/*
 *  ======== mainThread ========
 */
//void *mainThread(void *arg0)
static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    I2C_Params      i2cParams;

    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while(1) {
            sleep(100);
        }
    }

    /* Fetch MAC address */
    if(EasyLink_getIeeeAddr(nodeIeeeAddr) != EasyLink_Status_Success) {
        Display_printf(display, 0, 0, "Unable to get device MAC address!\n");
        while(1) {
            sleep(100);
        }
    }

    Display_printf(display, 0, 0, "Sensor node: MAC address 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                   nodeIeeeAddr[0], nodeIeeeAddr[1], nodeIeeeAddr[2], nodeIeeeAddr[3],
                   nodeIeeeAddr[4], nodeIeeeAddr[5], nodeIeeeAddr[6], nodeIeeeAddr[7]
                  );

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if (!buttonPinHandle)
    {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallback) != 0)
    {
        System_abort("Error registering button callback function");
    }

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_BME280, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    struct bme280_dev dev;

    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);

    Display_printf(display, 0, 0, "BME280 Init result = %d\n", rslt);

    // configure BME280 in forced mode
    /* Weather station, ultra-low power */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_1X;
    dev.settings.osr_t = BME280_OVERSAMPLING_1X;
    dev.settings.filter = BME280_FILTER_COEFF_OFF;

    uint8_t settings_sel;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    Display_printf(display, 0, 0, "BME280 configuration result = %d\n", rslt);

    // periodically read BME280 sensor in forced mode
    while(true) {

        struct bme280_data comp_data;
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        /* Wait for the measurement to complete and print data @25Hz */
        dev.delay_ms(40);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        Display_printf(display, 0, 0, "BME280 get_sensor_data() = %d\n", rslt);

        print_sensor_data(&comp_data);
        GetCPUBatteryAndTemp();

        struct IoTSensorData sensorData;
        sensorData._cpuTemp = AONBatMonTemperatureGetDegC();
        sensorData._cpuVolt = AONBatMonBatteryVoltageGet();
        sensorData._bme280Temp = comp_data.temperature;
        sensorData._bme280Pressure = comp_data.pressure;
        sensorData._bme280Humidity = comp_data.humidity;

        enum NodeRadioOperationStatus wsn_res;
        wsn_res = NodeRadioTask_sendSensorData(&sensorData);
        Display_printf(display, 0, 0, "WSN send result = %d\n", wsn_res);

        sleep(sample_interval);
    }

    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

}

/*
 *  ======== buttonCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */
static void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);

    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0) {
        sample_interval *= 2;
        if(sample_interval > sample_interval_max) {
            sample_interval = sample_interval_min;
        }
    }
}
