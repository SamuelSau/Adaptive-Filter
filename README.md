# Adaptive-Filter

## Purpose

Implement an adaptive filter in real-time with the STM32446RE-Nucleo board using the STM32CubeIDE

## Features
* 180mHZ max clock speed for SYSCLCK using HSI RC (eventually may port to HSE for stability).
* Enabled USART2 using CubeIDE's serial terminal for COM5 port to debug printf and debugging statements.
* Configured TIM2 interrupts in the .ioc to display on LD2 (PA5) and within a logic analyzer based on the ARR and prescaler configs.
