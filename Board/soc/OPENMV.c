/**
 * @brief       摄像头相关
 * @date        2020/8/12
 *
 */

#include "include.h"
   

/**
 * @brief       接收OPENMV的协议接收端实现
 * @example     recieve_decouder(UART_GetChar(UART2));
 */
void recieve_decouder(uint8_t data)
{
  static uint8_t head_buff[3] = {0}, i = 0;
  static uint8_t *data_to_receive = NULL;
  
  //帧头接收 
  if (i < 4)
    head_buff[i++] = data;
  
  //辨别帧头 
  if (head_buff[0] != 0xaa)
    i = 0;
  
  else if (i == 2 && head_buff[1] != 0xaa)
    i = 0;
  
  else if (i == 3) {
    data_to_receive = (uint8_t *)malloc(head_buff[i++]);
  }
  
  //接收数据 
  else if (i > 3) 
    if ((i - 4) < head_buff[2]){
      data_to_receive[i - 4] = data;
      i++;
    }
    else if (data == 0x11) {
      for (int j = 0; j < head_buff[2]; j++)
        printf("%d ", data_to_receive[j]);
      
      printf("\n\n");
      free(data_to_receive);
      i = 0;
    }
  
    else {
      printf("failed\n\n");
      free(data_to_receive);
      i = 0;	
    }
  
} 
