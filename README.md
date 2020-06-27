# 麦克风定位，利用FFT加速互相关运算

### 分支说明

master 分支用于存放稳定版本，目前较旧

dev 分支用于存放测试版本，一般更新都更新到此分支。若要下载先下载此分支下的文件。

### 文件目录（包含.c .h 文件、编译后的 .o 文件、IAR生成的配置文件）

实际需要添加进工程的文件如下

APP/soc/**IRQ_Handler.c**
				**main.c**

Board/soc/**ANO_DT.c** 
				    **FFT.c** 
			   	 **LG_EncTPM.c**
		    		**Longer_motor.c**
					**LQ_LED.c**

Libraries/Drive/soc/**ALL**


<<<<<<< HEAD
            
=======
D:.
│  clear.bat
│  README.md
│  文件目录.txt
│  生成文件目录.bat
│  
├─APP
│  ├─inc
│  │      include.h
│  │      
│  └─soc
│          **IRQ_Handler.c**
│          **main.c**
│          
├─Board
│  ├─inc
│  │      ANO_DT.h
│  │      FFT.h
│  │      LG_EncTPM.h
│  │      Longer_motor.h
│  │      LQ_12864.h
│  │      LQ_9AX.h
│  │      LQ_ADC.h
│  │      LQ_CAMERA_MT9V034.h
│  │      LQ_CAMERA_OV7725.h
│  │      LQ_CCD.h
│  │      LQ_FLASH.h
│  │      LQ_KEY.h
│  │      LQ_LED.h
│  │      LQ_LPTMR.h
│  │      LQ_MOTOR.h
│  │      LQ_MPU6050.h
│  │      LQ_MT9V034.h
│  │      LQ_OV7725.h
│  │      LQ_PIT.h
│  │      LQ_SD5.h
│  │      LQ_SGP18T.h
│  │      LQ_SYSTICK.h
│  │      LQ_UART.h
│  │      LQ_Ultrasonic.h
│  │      LQ_WDOG.h
│  │      picture.h
│  │      Slide_average_filter.h
│  │      
│  └─soc
│          **ANO_DT.c**
│          **FFT.c**
│          **LG_EncTPM.c**
│          **Longer_motor.c**
│          LQ_12864.c
│          LQ_9AX.c
│          LQ_ADC.c
│          LQ_CAMERA_MT9V034.c
│          LQ_CAMERA_OV7725.c
│          LQ_CCD.c
│          LQ_FLASH.c
│          LQ_KEY.c
│          **LQ_LED.c**
│          LQ_LPTMR.c
│          LQ_MOTOR.c
│          LQ_MPU6050.c
│          LQ_MT9V034.c
│          LQ_OV7725.c
│          LQ_PIT.c
│          LQ_SD5.c
│          LQ_SGP18T.c
│          LQ_SYSTICK.c
│          LQ_UART.c
│          LQ_Ultrasonic.c
│          LQ_WDOG.c
│          Slide_average_filter.c
│          
├─CMSIS
│  ├─Libs
│  │      arm_common_tables.h
│  │      arm_const_structs.h
│  │      arm_math.h
│  │      core_cm4.h
│  │      core_cm4_simd.h
│  │      core_cmFunc.h
│  │      core_cmInstr.h
│  │      fsl_device_registers.h
│  │      MK66F18.h
│  │      MK66F18_features.h
│  │      system_MK66F18.c
│  │      system_MK66F18.h
│  │      
│  ├─Linker
│  │      FlashK66Fxxx256K.board
│  │      LQK66FLASH1M.icf
│  │      LQK66RAM256K.icf
│  │      MK66FX1M0xxx18.ddf
│  │      MK66FX1M0xxx18_flash.scf
│  │      Pflash.icf
│  │      
│  └─Startup
│          start.c
│          startup_MK66F18.s
│          
├─DOC
│      修改之处.txt
│      文件目录.txt
│      
├─Libraries
│  └─Drive
│      ├─inc
│      │      common.h
│      │      fsl_clock.h
│      │      fsl_common.h
│      │      fsl_port.h
│      │      MK60_ADC.h
│      │      MK60_CMT.h
│      │      MK60_DMA.h
│      │      MK60_FLASH.h
│      │      MK60_FTM.h
│      │      MK60_GPIO.h
│      │      MK60_GPIO_Cfg.h
│      │      MK60_IIC.h
│      │      MK60_LPTMR.h
│      │      MK60_PIT.h
│      │      MK60_PLL.h
│      │      MK60_SYSTICK.h
│      │      MK60_UART.h
│      │      MK60_WDOG.h
│      │      MK66_TPM.h
│      │      
│      └─soc
│              **fsl_clock.c**
**│              fsl_common.c**
**│              MK60_ADC.c**
**│              MK60_CMT.c**
**│              MK60_DMA.c**
**│              MK60_FLASH.c**
**│              MK60_FTM.c**
**│              MK60_GPIO.c**
**│              MK60_IIC.c**
**│              MK60_LPTMR.c**
**│              MK60_PIT.c**
**│              MK60_PLL.c**
**│              MK60_SYSTICK.c**
**│              MK60_UART.c**
**│              MK60_WDOG.c**
**│              MK66_TPM.c**
│              
└─project
    │  MAX&FFT_K66.eww
    │  template.dep
    │  template.ewd
    │  template.ewp
    │  template.ewt
    │  
    ├─Debug
    │  ├─Exe
    │  │      template.out
    │  │      template.sim
    │  │      
    │  ├─List
    │  │      template.map
    │  │      
    │  └─Obj
    │          ANO_DT.o
    │          ANO_DT.pbi
    │          ANO_DT.pbi.xcl
    │          FFT.o
    │          FFT.pbi
    │          FFT.pbi.xcl
    │          fsl_clock.o
    │          fsl_clock.pbi
    │          fsl_clock.pbi.xcl
    │          fsl_common.o
    │          fsl_common.pbi
    │          fsl_common.pbi.xcl
    │          IRQ_Handler.o
    │          IRQ_Handler.pbi
    │          IRQ_Handler.pbi.xcl
    │          LG_EncTPM.o
    │          LG_EncTPM.pbi
    │          LG_EncTPM.pbi.xcl
    │          Longer_motor.o
    │          Longer_motor.pbi
    │          Longer_motor.pbi.xcl
    │          LQ_12864.o
    │          LQ_12864.pbi.xcl
    │          LQ_9AX.o
    │          LQ_9AX.pbi.xcl
    │          LQ_ADC.o
    │          LQ_ADC.pbi.xcl
    │          LQ_CAMERA_MT9V034.pbi.xcl
    │          LQ_CAMERA_OV7725.o
    │          LQ_CAMERA_OV7725.pbi.xcl
    │          LQ_CCD.o
    │          LQ_CCD.pbi.xcl
    │          LQ_FLASH.o
    │          LQ_FLASH.pbi.xcl
    │          LQ_KEY.o
    │          LQ_KEY.pbi.xcl
    │          LQ_LED.o
    │          LQ_LED.pbi
    │          LQ_LED.pbi.xcl
    │          LQ_LPTMR.o
    │          LQ_LPTMR.pbi.xcl
    │          LQ_MOTOR.o
    │          LQ_MOTOR.pbi.xcl
    │          LQ_MPU6050.o
    │          LQ_MPU6050.pbi.xcl
    │          LQ_MT9V034.pbi.xcl
    │          LQ_OV7725.o
    │          LQ_OV7725.pbi.xcl
    │          LQ_PIT.o
    │          LQ_PIT.pbi.xcl
    │          LQ_SD5.o
    │          LQ_SD5.pbi.xcl
    │          LQ_SGP18T.o
    │          LQ_SGP18T.pbi.xcl
    │          LQ_SYSTICK.o
    │          LQ_SYSTICK.pbi.xcl
    │          LQ_UART.o
    │          LQ_UART.pbi.xcl
    │          LQ_Ultrasonic.o
    │          LQ_Ultrasonic.pbi.xcl
    │          LQ_WDOG.o
    │          LQ_WDOG.pbi.xcl
    │          main.o
    │          main.pbi
    │          main.pbi.xcl
    │          MK60_ADC.o
    │          MK60_ADC.pbi
    │          MK60_ADC.pbi.xcl
    │          MK60_CMT.o
    │          MK60_CMT.pbi
    │          MK60_CMT.pbi.xcl
    │          MK60_DMA.o
    │          MK60_DMA.pbi
    │          MK60_DMA.pbi.xcl
    │          MK60_FLASH.o
    │          MK60_FLASH.pbi
    │          MK60_FLASH.pbi.xcl
    │          MK60_FTM.o
    │          MK60_FTM.pbi
    │          MK60_FTM.pbi.xcl
    │          MK60_GPIO.o
    │          MK60_GPIO.pbi
    │          MK60_GPIO.pbi.xcl
    │          MK60_IIC.o
    │          MK60_IIC.pbi
    │          MK60_IIC.pbi.xcl
    │          MK60_LPTMR.o
    │          MK60_LPTMR.pbi
    │          MK60_LPTMR.pbi.xcl
    │          MK60_PIT.o
    │          MK60_PIT.pbi
    │          MK60_PIT.pbi.xcl
    │          MK60_PLL.o
    │          MK60_PLL.pbi
    │          MK60_PLL.pbi.xcl
    │          MK60_SYSTICK.o
    │          MK60_SYSTICK.pbi
    │          MK60_SYSTICK.pbi.xcl
    │          MK60_UART.o
    │          MK60_UART.pbi
    │          MK60_UART.pbi.xcl
    │          MK60_WDOG.o
    │          MK60_WDOG.pbi
    │          MK60_WDOG.pbi.xcl
    │          MK66_TPM.o
    │          MK66_TPM.pbi
    │          MK66_TPM.pbi.xcl
    │          Slide_average_filter.o
    │          Slide_average_filter.pbi
    │          Slide_average_filter.pbi.xcl
    │          start.o
    │          start.pbi
    │          start.pbi.xcl
    │          startup_MK66F18.o
    │          template.pbd
    │          template.pbd.browse
    │          template.pbd.linf
    │          template.pbd~RF165aab6b.TMP
    │          template.pbw
    │          
    └─settings
            ICM20602.wsdt
            MAX&FFT_K66.wsdt
            MAX&FFT_K66_EncodingOverride.xml
            template.crun
            template.dbgdt
            template.Debug.cspy.bat
            template.Debug.cspy.ps1
            template.Debug.driver.xcl
            template.Debug.general.xcl
            template.dnx
            template.reggroups
            template.wsdt
>>>>>>> 82c3561f27770593052216cd773e85f9e382dce2
