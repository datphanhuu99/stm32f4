SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;C:\Program Files\nodejs\;C:\Program Files\Git\cmd;C:\Users\ADMIN\AppData\Local\Microsoft\WindowsApps;C:\Program Files\heroku\bin;C:\Users\ADMIN\AppData\Roaming\npm;;C:\altera\13.0sp1\modelsim_ase\win32aloem
SET CPU_TYPE=STM32F429ZITx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=dkdc
SET CPU_CLOCK=0x0A037A00
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\system_stm32f4xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\dkdc\startup_stm32f429xx._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_pwr_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_flash_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_dma_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_flash.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_flash_ramfunc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_pwr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\dkdc\stm32f4xx_hal.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\dkdc\dkdc.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\dkdc\dkdc.axf" --i32combined --output ".\dkdc\dkdc.hex"
