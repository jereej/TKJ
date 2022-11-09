/* EAT-arvon nostaminen tapahtuu liikutettaessa sensortagia vaakatasossa (jokin kynnysehto)
 * EXERCISE-arvon nostaminen tapahtuu liikutettaessa sensortagia pystytasossa (jokin kynnysehto)
 * PET-arvon nostaminen tapahtuu nappia painamalla
 * Arvon nostaminen tapahtuu activate-komennolla activate(n,n,n)
 *
 * buzzer: esim.
 * jos PET-arvo on alle x, niin sensortag piippaa 3 lyhyttä piippausta
 * jos EXERCISE-arvo on alle x, niin sensortag piippaa pidempään
 * kun kaikki arvot ovat 10, niin buzzer "soittaa" jonkin lyhyen melodian
 *
 *
 * Authors: Jere Jacklin, Tommi Jokinen, Jeremias Nevalainen
 */



#include <stdio.h>
#include <time.h>

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

#define STACKSIZE 2048
Char taskStack[STACKSIZE];

//global variables for states
static int EAT = 10;
static int EXERCISE = 10;
static int PET = 1;

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

    if (PET < 10){
        PET++;
        System_printf("Pet value increased to %d\n", PET);
        System_flush();
    } else if (PET >= 10){
        PET = 10;
        System_printf("Pet doesn't like u anymore\n");
        System_flush();
    } else {
        PET = 1;
        }
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


    fpt = fopen("D:/ti/workspace/empty_CC2650STK_TI/collectdata.csv", "w");

    if (!fpt) {
            System_printf("Can't open file.\n");
            System_flush();
        }

    fprintf(fpt, "time,ax,ay,az,gx,gy,gz\n");
    // Loop forever
    while (1) {

        // MPU ask data
        mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
        time++;
        fprintf(fpt,"%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", time, ax, ay, az, gx, gy, gz);
        // Sleep 100ms
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

void valueReduce(UArg arg0, UArg arg1){
    while (1){
        Task_sleep(5000000 / Clock_tickPeriod);
        PET--;
        System_printf("PET value reduced to %d\n", PET);
        System_flush();
        Task_sleep(3000000 / Clock_tickPeriod);
        EAT--;
        System_printf("EAT value reduced to %d\n", EAT);
        System_flush();
        Task_sleep(2000000 / Clock_tickPeriod);
        EXERCISE--;
        System_printf("EXERCISE value reduced to %d\n", EXERCISE);
        System_flush();
        Task_sleep(5000000 / Clock_tickPeriod);
    }
}
int main (void){

    Task_Handle task, task1;
    Task_Params taskParams, task1Params;

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

    Task_Params_init(&task1Params);
    task1 = Task_create((Task_FuncPtr)valueReduce, &task1Params, NULL);
    if (task1 == NULL) {
        System_abort("Task create failed!");
    }


    /*Sanity check*/
    System_printf("Working!\n");
    System_flush();
    /* BIOS start */
    BIOS_start();
    return (0);
}
