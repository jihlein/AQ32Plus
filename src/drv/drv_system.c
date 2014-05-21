/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

// Cycle counter stuff - these should be defined by CMSIS, but they aren't
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

///////////////////////////////////////////////////////////////////////////////

// Cycles per microsecond
static volatile uint32_t usTicks = 0;

///////////////////////////////////////////////////////////////////////////////

// Current uptime for 1kHz systick timer. will rollover after 49 days.
// Hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;

///////////////////////////////////////////////////////////////////////////////
// Cycle Counter
///////////////////////////////////////////////////////////////////////////////

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;
}

///////////////////////////////////////
// Frame Timing Variables
///////////////////////////////////////

uint16_t frameCounter = 0;

semaphore_t frame_500Hz = false;
semaphore_t frame_100Hz = false;
semaphore_t frame_50Hz  = false;
semaphore_t frame_10Hz  = false;
semaphore_t frame_5Hz   = false;
semaphore_t frame_1Hz   = false;

uint32_t deltaTime1000Hz, executionTime1000Hz, previous1000HzTime;
uint32_t deltaTime500Hz,  executionTime500Hz,  previous500HzTime;
uint32_t deltaTime100Hz,  executionTime100Hz,  previous100HzTime;
uint32_t deltaTime50Hz,   executionTime50Hz,   previous50HzTime;
uint32_t deltaTime10Hz,   executionTime10Hz,   previous10HzTime;
uint32_t deltaTime5Hz,    executionTime5Hz,    previous5HzTime;
uint32_t deltaTime1Hz,    executionTime1Hz,    previous1HzTime;

float dt500Hz, dt100Hz;

semaphore_t systemReady = false;

semaphore_t execUp = false;

#ifdef _DTIMING
	#define LA2_ENABLE       GPIO_SetBits(GPIOC,   GPIO_Pin_2)
	#define LA2_DISABLE      GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#endif


///////////////////////////////////////////////////////////////////////////////
// SysTick
///////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
    uint8_t index;
    uint32_t currentTime;

    sysTickCycleCounter = *DWT_CYCCNT;
    sysTickUptime++;

    watchDogsTick();

    if ((systemReady         == true)  &&
        (cliBusy             == false) &&
        (accelCalibrating    == false) &&
        (escCalibrating      == false) &&
        (magCalibrating      == false) &&
        (mpu6000Calibrating  == false))

    {
        #ifdef _DTIMING
//    	    LA2_ENABLE;
        #endif

        frameCounter++;
        if (frameCounter > FRAME_COUNT)
            frameCounter = 1;

        ///////////////////////////////

        currentTime = micros();
        deltaTime1000Hz = currentTime - previous1000HzTime;
        previous1000HzTime = currentTime;

        readMPU6000();

        accelSum500Hz[XAXIS] += rawAccel[XAXIS].value;
        accelSum500Hz[YAXIS] += rawAccel[YAXIS].value;
        accelSum500Hz[ZAXIS] += rawAccel[ZAXIS].value;

        accelSum100Hz[XAXIS] += rawAccel[XAXIS].value;
        accelSum100Hz[YAXIS] += rawAccel[YAXIS].value;
        accelSum100Hz[ZAXIS] += rawAccel[ZAXIS].value;

        gyroSum500Hz[ROLL ] += rawGyro[ROLL ].value;
        gyroSum500Hz[PITCH] += rawGyro[PITCH].value;
        gyroSum500Hz[YAW  ] += rawGyro[YAW  ].value;

        ///////////////////////////////

        if ((frameCounter % COUNT_500HZ) == 0)
        {
            frame_500Hz = true;

            for (index = 0; index < 3; index++)
            {
            	accelSummedSamples500Hz[index] = accelSum500Hz[index];
            	accelSum500Hz[index] = 0;

            	gyroSummedSamples500Hz[index] = gyroSum500Hz[index];
                gyroSum500Hz[index] = 0;
            }
        }

        ///////////////////////////////

        if ((frameCounter % COUNT_100HZ) == 0)
        {
            frame_100Hz = true;

            for (index = 0; index < 3; index++)
            {
                accelSummedSamples100Hz[index] = accelSum100Hz[index];
                accelSum100Hz[index] = 0;
            }

            if (!newTemperatureReading)
			{
				readTemperatureRequestPressure();
			    newTemperatureReading = true;
			}
			else
			{
			    readPressureRequestTemperature();
			    newPressureReading = true;
			}

			sdCardCountDown();
        }

        ///////////////////////////////

        if ((frameCounter % COUNT_50HZ) == 0)
            frame_50Hz = true;

        ///////////////////////////////

        if (((frameCounter + 1) % COUNT_10HZ) == 0)
            newMagData = readMag();

        if ((frameCounter % COUNT_10HZ) == 0)
            frame_10Hz = true;

        ///////////////////////////////

        if ((frameCounter % COUNT_5HZ) == 0)
            frame_5Hz = true;

        ///////////////////////////////

        if ((frameCounter % COUNT_1HZ) == 0)
            frame_1Hz = true;

        ///////////////////////////////////

        executionTime1000Hz = micros() - currentTime;

        ///////////////////////////////

        #ifdef _DTIMING
//            LA2_DISABLE;
        #endif
    }
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Microseconds
//
// Note: This can be called from within IRQ Handlers, so uses LDREX/STREX.
// If a higher priority IRQ or DMA or anything happens the STREX will fail
// and restart the loop. Otherwise the same number that was read is harmlessly
// written back.
///////////////////////////////////////////////////////////////////////////////

uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;

    do
    {
        timeMs = __LDREXW(&sysTickUptime);
        cycle = *DWT_CYCCNT;
        oldCycle = sysTickCycleCounter;
    }
    while ( __STREXW( timeMs , &sysTickUptime ) );

    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Milliseconds
///////////////////////////////////////////////////////////////////////////////

uint32_t millis(void)
{
    return sysTickUptime;
}

///////////////////////////////////////////////////////////////////////////////
// System Initialization
///////////////////////////////////////////////////////////////////////////////

void checkResetType()
{
    uint32_t rst = RCC->CSR;

    evrPush(( rst & (RCC_CSR_PORRSTF | RCC_CSR_PADRSTF | RCC_CSR_SFTRSTF) ) ? EVR_NormalReset : EVR_AbnormalReset , rst >> 24 );

    RCC_ClearFlag();
}

///////////////////////////////////////

void systemInit(void)
{
    RCC_ClocksTypeDef rccClocks;

	///////////////////////////////////

	// Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    // Turn on peripherial clcoks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,   ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,   ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,   ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,  ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,  ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,   ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,   ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,   ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,   ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,  ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    ///////////////////////////////////

    checkFirstTime(false);
	readEEPROM();

	if (eepromConfig.receiverType == SPEKTRUM)
		checkSpektrumBind();

	checkResetType();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 2 bits for pre-emption priority, 2 bits for subpriority

	///////////////////////////////////

	gpsPortClearBuffer       = &uart2ClearBuffer;
    gpsPortNumCharsAvailable = &uart2NumCharsAvailable;
    gpsPortPrintBinary       = &uart2PrintBinary;
    gpsPortRead              = &uart2Read;

    openLogPortPrintF        = &uart3PrintF;

	///////////////////////////////////

	initMixer();

    ledInit();

    usbInit();

    uart1Init();
    uart2Init();
    uart3Init();

    ///////////////////////////////////

    BLUE_LED_ON;

    delay(10000);  // 10 seconds of 20 second delay for sensor stabilization

    ///////////////////////////////////

    checkUsbActive();

    #ifdef __VERSION__
        cliPortPrintF("\ngcc version " __VERSION__ "\n");
    #endif

    cliPortPrintF("\nAQ32Plus Firmware V%s, Build Date " __DATE__ " "__TIME__" \n", __AQ32PLUS_VERSION);

    if ((RCC->CR & RCC_CR_HSERDY) != RESET)
    {
        cliPortPrint("\nRunning on external HSE clock....\n");
    }
    else
    {
        cliPortPrint("\nERROR: Running on internal HSI clock....\n");
    }

    RCC_GetClocksFreq(&rccClocks);

    cliPortPrintF("\nHCLK->   %3d MHz\n",   rccClocks.HCLK_Frequency   / 1000000);
    cliPortPrintF(  "PCLK1->  %3d MHz\n",   rccClocks.PCLK1_Frequency  / 1000000);
    cliPortPrintF(  "PCLK2->  %3d MHz\n",   rccClocks.PCLK2_Frequency  / 1000000);
    cliPortPrintF(  "SYSCLK-> %3d MHz\n\n", rccClocks.SYSCLK_Frequency / 1000000);

    if (eepromConfig.receiverType == PPM)
    	cliPortPrint("Using PPM Receiver....\n\n");
    else if (eepromConfig.receiverType == PWM)
        cliPortPrint("Using PWM Receiver....\n\n");
    else if (eepromConfig.receiverType == SPEKTRUM)
    	cliPortPrint("Using Spektrum Satellite Receiver....\n\n");
    else
    	cliPortPrint("Error....\n\n");

    initUBLOX();

    delay(10000);  // Remaining 10 seconds of 20 second delay for sensor stabilization - probably not long enough..

    ///////////////////////////////////

    adcInit();
    i2cInit(I2C1);
    i2cInit(I2C2);
    pwmServoInit();

    if (eepromConfig.receiverType == SPEKTRUM)
        spektrumInit();
    else
        rxInit();

    spiInit(SPI1);
    spiInit(SPI2);
    spiInit(SPI3);
    timingFunctionsInit();

    batteryInit();

    initFirstOrderFilter();
    initMavlink();
    initMax7456();
    initPID();

    switch (initSDCard())
    {
        case  0:
        	cliPortPrint("SD Card Initialization Failed....\n\n");
        	break;

        case  1:
        	cliPortPrint("SD Card Initialized, MMC Version 3....\n\n");
        	break;

        case  2:
        	cliPortPrint("SD Card Initialized, SD Version 1....\n\n");
        	break;

        case  4:
        case 12:
        	cliPortPrint("SD Card Initialized, SD Version 2....\n\n");
        	break;
    }

    GREEN_LED_ON;

    initMPU6000();
    initMag();
    initPressure();
}

///////////////////////////////////////////////////////////////////////////////
// Delay Microseconds
///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = *DWT_CYCCNT;

    for (;;) {
        register uint32_t current_count = *DWT_CYCCNT;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////
// System Reset
///////////////////////////////////////////////////////////////////////////////

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader)
    {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F407
    }

    GPIO_ResetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

    delay(200);

    GPIO_SetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t) 0x04;
}

///////////////////////////////////////////////////////////////////////////////
