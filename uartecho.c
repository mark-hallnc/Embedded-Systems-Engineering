
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, output, x);

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
// Driver Handles - Global variables
I2C_Handle i2c;

// UART Global Variables
char output[64];
int bytesToSend;
// Driver Handles - Global variables
UART_Handle uart;

// Timer Global Variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

// Button Pressed Flag
volatile uint_least8_t buttonPressed = 0;

// Global variables for set-point and temperature
volatile int setPoint = 20; // Default set-point temperature
volatile int temperature = 0;
volatile int heat = 0;
volatile uint32_t seconds = 0;

// Task scheduler flags
volatile int task200msFlag = 0;
volatile int task500msFlag = 0;
volatile int task1000msFlag = 0;

/*
 *  ======== timerCallback ========
 *  Timer callback function
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    static int timerCounter = 0;
    TimerFlag = 1;

    // Increment the timer counter
    timerCounter++;

    // Set task flags based on the timer counter
    if (timerCounter % 200 == 0) {
        task200msFlag = 1;
    }
    if (timerCounter % 500 == 0) {
        task500msFlag = 1;
    }
    if (timerCounter % 1000 == 0) {
        task1000msFlag = 1;
        timerCounter = 0; // Reset the counter every second
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Set the flag to indicate button press */
    buttonPressed = 1;
}

/*
 *  ======== initUART ========
 *  Initialize the UART.
 */
void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/*
 *  ======== initI2C ========
 *  Initialize the I2C.
 */
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Try to determine which sensor we have.
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }
    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

/*
 *  ======== readTemp ========
 *  Read temperature from the sensor.
 */
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

/*
 *  ======== initTimer ========
 *  Initialize the timer.
 */
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000; // 1 millisecond period for fine-grained control
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialize timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Initialize UART before I2C */
    initUART();

    /* Initialize I2C */
    initI2C();

    /* Initialize the timer */
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // LED off initially

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    while (1) {
        if (TimerFlag) {
            TimerFlag = 0;

            // Check button flags every 200ms
            if (task200msFlag) {
                task200msFlag = 0;
                if (buttonPressed) {
                    setPoint++;
                    buttonPressed = 0;
                }
            }

            // Read temperature and update LED every 500ms
            if (task500msFlag) {
                task500msFlag = 0;
                temperature = readTemp();

                if (temperature > setPoint) {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Heat off
                    heat = 0;
                } else {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // Heat on
                    heat = 1;
                }
            }

            // Output to UART every second
            if (task1000msFlag) {
                task1000msFlag = 0;
                seconds++;
                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setPoint, heat, seconds));
            }
        }
    }

    return (NULL);
}
