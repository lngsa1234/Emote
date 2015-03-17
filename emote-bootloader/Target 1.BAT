SET PATH=d:\Keil\ARM\ARMCC\bin\;C:\ProgramData\Oracle\Java\javapath;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Windows Kits\8.1\Windows Performance Toolkit\;C:\Program Files\Microsoft SQL Server\110\Tools\Binn\;D:\MATLAB7\bin\win32;D:\Program Files\IDM Computer Solutions\UltraEdit\;D:\Program Files\IDM Computer Solutions\UltraCompare\;D:\Program Files\GNU Tools ARM Embedded\4.9 2014q4\bin;D:\Program Files\GNU Tools ARM Embedded\4.9 2014q4
SET CPU_TYPE=STM32F103ZG
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x007A1200
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\core_cm3.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\system_stm32f10x.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\misc.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\stm32f10x_flash.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\stm32f10x_gpio.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\stm32f10x_rcc.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\stm32f10x_usart.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\retarget.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\serial.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\stm32f10x_it.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\usart1_conf.__i"
"d:\Keil\ARM\ARMCC\bin\ArmAsm" --Via ".\debug\startup_stm32f10x_xl._ia"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\common.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\download.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\main.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\upload.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\ymodem.__i"
"d:\Keil\ARM\ARMCC\bin\ArmCC" --Via ".\debug\usart2_conf.__i"
"d:\Keil\ARM\ARMCC\bin\ArmLink" --Via ".\debug\usart.lnp"
"d:\Keil\ARM\ARMCC\bin\fromelf.exe" ".\debug\usart.axf" --i32combined --output ".\debug\usart.hex"
fromelf --bin --output ./debug/usart.bin ./debug/usart.axf
