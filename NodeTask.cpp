/*
 * Sensor Node Main Task
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#if 0
/* TI-RTOS Header files */ 
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

#include "Board.h"

#include "easylink/EasyLink.h"
#include "NodeTask.h"
#include "NodeRadioTask.h"

class SensorNodeManager
{
public:
    SensorNodeManager();
    void initializeTask();

    enum {
        EV_ALL      = 0xFFFFFFFF,
        EV_BUTTON0  = 1 << 1,
        EV_SAMPLE   = 1 << 4,
    };

private:

    /** main entry of the manager task
     *  @param arg0 pointer to sensor node manager
     *  @param arg1 unused
     */
    static void main(xdc_UArg arg0, xdc_UArg arg1);

    void initializeWork();
    void mainLoop();

    /** methods to process button press in HWI and Task context respectively */
    void initializeButton();
    static void buttonCb(PIN_Handle handle, PIN_Id pinId);
    void handleButtonPress();

    enum {
        STACK_SIZE  = 1024,
        TASK_PRIORITY = 3
    };
    Task_Params  _taskParams;
    Task_Struct  _task;
    uint8_t      _taskStack[STACK_SIZE];

    Event_Struct _managerEvent;
    Event_Handle _managerEventHandle;

    // data structures for button management
    PIN_Handle          _buttonPinHandle;
    PIN_State           _buttonPinState;
    static const PIN_Config    _buttonPinTable[];

};

SensorNodeManager   gNodeManager;

// We use btn0 for user-interaction.
const PIN_Config SensorNodeManager::_buttonPinTable[] = {
    Board_PIN_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};


SensorNodeManager::SensorNodeManager()
{
    memset(_taskStack, 0, sizeof(_taskStack));
}

void SensorNodeManager::initializeTask()
{
    uint8_t *macAddr = NodeRadioTask_getMACAddress();
    if(!macAddr) {
        System_abort("Sensor node: unable to get MAC address!\n");
    }

    System_printf("Sensor node at MAC address 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                   macAddr[0], macAddr[1], macAddr[2], macAddr[3],
                   macAddr[4], macAddr[5], macAddr[6], macAddr[7] );

    // Initialize event control structure
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&_managerEvent, &eventParam);
    _managerEventHandle = Event_handle(&_managerEvent);

    // Initialize task control structure
    Task_Params_init(&_taskParams);
    _taskParams.instance->name = "SensorNodeManager";
    _taskParams.stackSize = STACK_SIZE;
    _taskParams.priority = TASK_PRIORITY;
    _taskParams.stack = _taskStack;
    _taskParams.arg0 = (xdc_UArg) this;
    _taskParams.arg1 = (xdc_UArg) 0;
    Task_construct(&_task, SensorNodeManager::main, &_taskParams, NULL);
}

void SensorNodeManager::main(xdc_UArg arg0, xdc_UArg)
{
    SensorNodeManager *self = (SensorNodeManager *) arg0;
    self->initializeWork();
    self->mainLoop();
}

void SensorNodeManager::buttonCb(PIN_Handle, PIN_Id)
{
    Event_post(gNodeManager._managerEventHandle, EV_BUTTON0);
}

void SensorNodeManager::initializeButton()
{
    // Initialize button
    _buttonPinHandle = PIN_open(&_buttonPinState, _buttonPinTable);
    if (!_buttonPinHandle) {
        System_abort("Unable to initialize push button!\n");
    }

    if (PIN_registerIntCb(_buttonPinHandle, &SensorNodeManager::buttonCb)) {
        System_abort("Unable to register button callback!\n");
    }
}

void SensorNodeManager::handleButtonPress()
{
    System_printf("Button0 pressed\n");
}

void SensorNodeManager::initializeWork()
{
    System_printf("Initializing hardware modules...\n");
    GPIO_init();
    I2C_init();

    initializeButton();


}

void SensorNodeManager::mainLoop()
{
    System_printf("Main loop...\n");
    while (true)
    {
        // wait for events
        uint32_t events = Event_pend(_managerEventHandle, 0, (UInt)EV_ALL, BIOS_WAIT_FOREVER);

        // button 0 is pressed
        if (events & EV_BUTTON0) {
            handleButtonPress();
        }
    }
}

void NodeTask_init(void)
{
    gNodeManager.initializeTask();
    // Task will be started when BIOS starts.
}

#if 0
static I2C_Handle       i2c;    // i2c handle for BME280




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
#endif

/*
 *  ======== buttonCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */

