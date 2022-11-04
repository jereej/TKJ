/* EAT-arvon nostaminen tapahtuu liikutettaessa sensortagia vaakatasossa (jokin kynnysehto)
 * EXERCISE-arvon nostaminen tapahtuu liikutettaessa sensortagia pystytasossa (jokin kynnysehto)
 * PET-arvon nostaminen tapahtuu nappia painamalla
 * Arvon nostaminen tapahtuu activate-komennolla activate(n,n,n)
 *
 * buzzer: esim.
 * jos PET-arvo on alle x, niin sensortag piippaa 3 lyhytt‰ piippausta
 * jos EXERCISE-arvo on alle x, niin sensortag piippaa pidemp‰‰n
 * kun kaikki arvot ovat 10, niin buzzer "soittaa" jonkin lyhyen melodian
 *
 *
 * Authors: Jere Jacklin, Tommi Jokinen, Jeremias Nevalainen
 */



#include <stdio.h>

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

#define STACKSIZE 2048
Char taskStack[STACKSIZE];

static int EAT = 10;
static int EXERCISE = 10;
static int PET = 10;

// Button and LED global variables
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;

// MPU global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

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

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {


    uint_t pinValue = PIN_getOutputValue( Board_LED1 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED1, pinValue );
}

Void mpuFxn(UArg arg0, UArg arg1) {

    float ax, ay, az, gx, gy, gz;
    uint16_t t = 0;
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


    fpt = fopen("D:/ti/workspace/empty_CC2650STK_TI/yeet.csv", "w");

    if (!fpt) {
            System_printf("Can't open file.\n");
            System_flush();
        }

    fprintf(fpt, "time,ax,ay,az,gx,gy,gz\n");
    // Loop forever
    while (1) {

        // MPU ask data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        t++;
        fprintf(fpt,"%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t, ax, ay, az, gx, gy, gz);
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
}



Int main (void){

    Task_Handle task;
    Task_Params taskParams;

    Board_initGeneral();
    Board_initI2C();

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

    Task_Params_init(&taskParams);
    taskParams.stackSize = STACKSIZE;
    taskParams.stack = &taskStack;
    task = Task_create((Task_FuncPtr)mpuFxn, &taskParams, NULL);
    if (task == NULL) {
        System_abort("Task create failed!");
    }


    /*Sanity check*/
    System_printf("Working!\n");
    System_flush();
    /* BIOS start */
    BIOS_start();
    return (0);
}
