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


            