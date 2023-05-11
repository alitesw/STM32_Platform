## <b>PWR_SLEEP Example Description</b>

How to enter the Sleep mode and wake up from this mode by using an interrupt.

In the associated software, the system clock is set to 48 MHz.
an EXTI line is connected to the Joystick Selection push-button through PA.08 and configured
to generate an interrupt on falling edge upon key press.
The SysTick is programmed to generate an interrupt each 1 ms and in the SysTick
interrupt handler, LED3 is toggled in order to indicate whether the MCU is in SLEEP mode
or RUN mode.

5 seconds after start-up, the system automatically enters SLEEP mode and LED3 stops toggling.

-   The Joystick Selection push-button can be pressed at any time to wake-up the system.
-   The software then comes back in RUN mode for 5 sec. before automatically entering SLEEP mode again.

LED3 is used to monitor the system state as follows:

 - **LED3 toggling :** system in RUN mode
 - **LED3 off :** system in SLEEP mode

These steps are repeated in an infinite loop.


#### <b>Notes</b>

 1. This example can not be used in DEBUG mode due to the fact
    that the Cortex-M0+ core is no longer clocked during low power mode
    so debugging features are disabled.

 2. Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
    based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
    a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
    than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
    To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

 3. The application needs to ensure that the SysTick time base is always set to 1 millisecond
    to have correct HAL operation.

### <b>Keywords</b>

Power, PWR, EXTI, Sleep mode, Interrupt, Wakeup, External reset

### <b>Directory contents</b>

  - PWR/PWR_SLEEP/Inc/stm32c0xx_conf.h         HAL Configuration file
  - PWR/PWR_SLEEP/Inc/stm32c0xx_it.h           Header for stm32c0xx_it.c
  - PWR/PWR_SLEEP/Inc/main.h                         Header file for main.c
  - PWR/PWR_SLEEP/Src/system_stm32c0xx.c       STM32C0xx system clock configuration file
  - PWR/PWR_SLEEP/Src/stm32c0xx_it.c           Interrupt handlers
  - PWR/PWR_SLEEP/Src/stm32c0xx_hal_msp.c      HAL MSP module
  - PWR/PWR_SLEEP/Src/main.c                         Main program

### <b>Hardware and Software environment</b>

  - This example runs on STM32C0xx devices

  - This example has been tested with STMicroelectronics STM32C0116-DISCO
    board and can be easily tailored to any other supported device
    and development board.

  - STM32C0116-DISCO set-up:

    - LED3 connected to PB.6 pin
    - Use the Joystick Selection push-button connected to pin PA.08 (EXTI_Line8)

### <b>How to use it ?</b>

In order to make the program work, you must do the following :

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example