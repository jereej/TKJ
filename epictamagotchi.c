#include <stdio.h>
#include <time.h>
#include <stdlib.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "wireless/comm_lib.h"
#include "sensors/mpu9250.h"
#include "buzzer.h"

enum state { WAITING=1, DATA_READY, BUZZER, BUZZER2, BUZZER3 };
enum state programState = WAITING;
enum state buzzerState = BUZZER;
enum state buzzerState2 = BUZZER2;
enum state buzzerState3 = BUZZER3;

#define STACKSIZE 2048
Char taskStack[STACKSIZE];
Char task2Stack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

//global variables for states
static int EAT = 0;
static int EXERCISE = 0;
static int PET = 0;

// Button and LED global variables
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

// MPU global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// Buzzer global variables
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;

// Buzzer config
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

//MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//MPU I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

//Button and LED config
PIN_Config buttonConfig[] = {
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config ledConfig[] = {
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

// From laboratory exercises
Void uartTaskFxn(UArg arg0, UArg arg1) {

    UART_Handle uart;
    UART_Params uartParams;


    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    while (1) {
        if (programState == DATA_READY){
            char str[80];
            char recievedStr[160];
            char input;
            sprintf(str, "id:2068,ACTIVATE:%d;%d;%d\0", EAT, EXERCISE, PET);
            System_printf(str);
            System_flush();

            UART_write(uart, str, strlen(str) + 1);
            buzzerState2 = BUZZER2;
            PET = 0;
            EAT = 0;
            EXERCISE = 0;


            //UART_read(uart, &input, sizeof(input)/sizeof(input[0]));
            UART_read(uart, &input, 1);
            sprintf(recievedStr,"%c\n",input);
            System_printf(recievedStr);
            System_flush();
            buzzerState3 = BUZZER3;
        }

        // Once per second, you can modify this
        Task_sleep(5000000 / Clock_tickPeriod);
    }
}

void activate(int eatValue, int petValue, int exerciseValue){

    PET += petValue;
    EAT += eatValue;
    EXERCISE += exerciseValue;

    System_printf("Current values, EAT: %d, PET: %d, EXERCISE: %d\n", EAT, PET, EXERCISE);
    System_flush();

}

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );

    activate(0,1,0);
    buzzerState = BUZZER;

}

void mpuFxn(UArg arg0, UArg arg1) {

    float ax, ay, az, gx, gy, gz;
    uint16_t time = 0;
    FILE *fpt;

    I2C_Handle i2cMPU;
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    //Remember to change the filepath to your own!
    fpt = fopen("D:/ti/workspace/empty_CC2650STK_TI/defaultdata1.csv", "w");

    if (!fpt) {
            System_printf("Can't open file.\n");
            System_flush();
        }

    fprintf(fpt, "time,ax,ay,az,gx,gy,gz\n");
    // Loop forever
    while (1) {

        // MPU ask data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        programState = DATA_READY;
        time++;
        fprintf(fpt,"%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", time, ax, ay, az, gx, gy, gz);
        // Sleep 100ms

        if (abs(gy) >= 70){
            activate(0,0,1);

        }

        if (abs(gz) >= 70){
            activate(1,0,0);

        }

        if (abs(gx) >= 100){
            activate(0,1,0);

        }

        Task_sleep(300000 / Clock_tickPeriod);
    }
}

Void bzrFxn(UArg arg0, UArg arg1) {
  while (1) {
    if (buzzerState == BUZZER){

    buzzerOpen(hBuzzer);
    buzzerSetFrequency(494);
    Task_sleep(207000 / Clock_tickPeriod);
    buzzerSetFrequency(622);
    Task_sleep(207000 / Clock_tickPeriod);
    buzzerSetFrequency(987);
    Task_sleep(207000 / Clock_tickPeriod);
    buzzerSetFrequency(932);
    Task_sleep(103000 / Clock_tickPeriod);
    Task_sleep(309000 / Clock_tickPeriod);
    buzzerSetFrequency(740);
    Task_sleep(207000 / Clock_tickPeriod);


    buzzerClose();
    Task_sleep(950000 / Clock_tickPeriod);
    buzzerState = WAITING;
    }

    else if (buzzerState2 == BUZZER2){

        buzzerOpen(hBuzzer);
        buzzerSetFrequency(1000);
        Task_sleep(103000 / Clock_tickPeriod);
        buzzerSetFrequency(1500);
        Task_sleep(103000 / Clock_tickPeriod);
        buzzerSetFrequency(500);
        Task_sleep(103000 / Clock_tickPeriod);
        buzzerClose();
        Task_sleep(950000 / Clock_tickPeriod);
        buzzerState2 = WAITING;
        }

    else if (buzzerState3 == BUZZER3){

            buzzerOpen(hBuzzer);
            buzzerSetFrequency(420);
            Task_sleep(103000 / Clock_tickPeriod);
            buzzerSetFrequency(666);
            Task_sleep(103000 / Clock_tickPeriod);
            buzzerClose();
            Task_sleep(950000 / Clock_tickPeriod);
            buzzerState3 = WAITING;
            }
  }
}

int main (void){

    Task_Handle task, task2;
    Task_Params taskParams, task2Params;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    Board_initGeneral();
    Board_initI2C();
    Init6LoWPAN();
    Board_initUART();

    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
        System_abort("Error initializing button pins\n");
    }
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
        System_abort("Error initializing LED pins\n");
    }

    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }

    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
        System_abort("Pin open failed!");
    }

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    task = Task_create((Task_FuncPtr)mpuFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&task2Params);
    task2Params.stackSize = STACKSIZE;
    task2Params.stack = &task2Stack;
    task2 = Task_create((Task_FuncPtr)bzrFxn, &task2Params, NULL);
    if (task2 == NULL) {
      System_abort("Task2 create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }


    /*Sanity check*/
    System_printf("Working!\n");
    System_flush();
    /* BIOS start */
    BIOS_start();
    return (0);
}
