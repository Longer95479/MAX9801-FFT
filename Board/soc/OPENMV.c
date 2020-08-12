/**
 * @brief       ����ͷ���
 * @date        2020/8/12
 *
 */

#include "include.h"
   

/**
 * @brief       ����OPENMV��Э����ն�ʵ��
 * @example     recieve_decouder(UART_GetChar(UART2));
 */
void recieve_decouder(uint8_t data)
{
  static uint8_t head_buff[3] = {0}, i = 0;
  static uint8_t *data_to_receive = NULL;
  
  //֡ͷ���� 
  if (i < 4)
    head_buff[i++] = data;
  
  //���֡ͷ 
  if (head_buff[0] != 0xaa)
    i = 0;
  
  else if (i == 2 && head_buff[1] != 0xaa)
    i = 0;
  
  else if (i == 3) {
    data_to_receive = (uint8_t *)malloc(head_buff[i++]);
  }
  
  //�������� 
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
