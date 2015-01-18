# Intro
  This is my template to work with freeRTOS on STM32F429 just with a **Makefile** and **ARM gcc** on a
  **linux environment** (thanks to Winfred Lu, take a look at [references](https://github.com/sousapedro596/stm32f429-freertos800#references)).





## Updates

##### **USART Implementation**
  In the folder `./custom_libs` there's a implementation (somehow hardcoded ...) of USART comunication using only ISR (**no polling**) and freeRTOS API.



### References

  This git repo was initial a copy from https://github.com/winfred-lu/stm32f429-freertos800, and from now let
  me say thanks for their great work! On their repo theres a great tuturial how to get freeRTOS and STM32F429 Discovery Firmware ready.
  Please take a [look](https://github.com/winfred-lu/stm32f429-freertos800) if you have some issue.
