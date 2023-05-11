﻿## <b>IWDG_RefreshUntilUserEvent_Init Example Description</b>

How to configure the IWDG peripheral to ensure periodical counter update and
generate an MCU IWDG reset when a User push-button is pressed. The peripheral
is initialized with LL unitary service functions to optimize for performance and size.

**Example Configuration**:

Configure the IWDG (prescaler, counter) and enable it.
Infinite refresh of the IWDG down-counter done in the main loop.

**LED4** is blinking fast & continuously.

**Example Execution**:

When User push-button is pressed, the down-counter automatic refresh mechanism is
disable and thus, reset will occur. After a reset, when re-entering in the main,
RCC IWDG Reset Flag will be checked and if we are back from a IWDG reset the **LED4** will be switch ON.

Waiting a new User push-button pressed to re-activate the IWDG

### <b>Keywords</b>

System, IWDG, reload counter, MCU Reset

### <b>Directory contents</b>

  - IWDG/IWDG_RefreshUntilUserEvent_Init/Inc/stm32c0xx_it.h          Interrupt handlers header file
  - IWDG/IWDG_RefreshUntilUserEvent_Init/Inc/main.h                        Header for main.c module
  - IWDG/IWDG_RefreshUntilUserEvent_Init/Inc/stm32_assert.h                Template file to include assert_failed function
  - IWDG/IWDG_RefreshUntilUserEvent_Init/Src/stm32c0xx_it.c          Interrupt handlers
  - IWDG/IWDG_RefreshUntilUserEvent_Init/Src/main.c                        Main program
  - IWDG/IWDG_RefreshUntilUserEvent_Init/Src/system_stm32c0xx.c      STM32C0xx system source file

### <b>Hardware and Software environment</b>

  - This example runs on STM32C031C6Tx devices.

  - This example has been tested with NUCLEO-C031C6 board and can be 
    easily tailored to any other supported device and development board.

### <b>How to use it ?</b>

In order to make the program work, you must do the following :

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

