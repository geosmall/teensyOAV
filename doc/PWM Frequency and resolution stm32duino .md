https://www.stm32duino.com/viewtopic.php?p=3273#p3273

Re: how to PWM Frequency stm32duino and resolution
Unread post by fpiSTM » Thu Jun 11, 2020 12:30 pm

You can try this instead of analog API:

HardwareTimer examples for PWM:
https://github.com/stm32duino/STM32Examples/blob/master/examples/Peripherals/HardwareTimer/All-in-one_setPWM/All-in-one_setPWM.ino
https://github.com/stm32duino/STM32Examples/blob/master/examples/Peripherals/HardwareTimer/PWM_FullConfiguration/PWM_FullConfiguration.ino


Wiki:
https://github.com/stm32duino/wiki/wiki/HardwareTimer-library