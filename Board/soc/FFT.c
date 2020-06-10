#include "include.h"

type_complex complex_build(float re0, float im0)
{
	type_complex cx;
	cx.re = re0;
	cx.im = im0;
	return cx;
}

type_complex complex_add(type_complex cx1, type_complex cx2)
{
	type_complex cx_add;
	cx_add.re = cx1.re + cx2.re;
	cx_add.im = cx1.im + cx2.im;
	return cx_add;
}
type_complex complex_minus(type_complex cx1, type_complex cx2)
{
	type_complex cx_minus;
	cx_minus.re = cx1.re - cx2.re;
	cx_minus.im = cx1.im - cx2.im;
	return cx_minus;
}

type_complex complex_mult(type_complex cx1, type_complex cx2)
{
	type_complex cx_mult;
	cx_mult.re = cx1.re * cx2.re - cx1.im * cx2.im;
	cx_mult.im = cx1.re * cx2.im + cx1.im * cx2.re;
	return cx_mult;
}

static void inver(type_complex x[],int N)
{
	int n = (int)log2(N);
	for (int index = 0; index < N; index++) {
		int index_inver = 0;
		
		for(int i = 0; i < n; i++)
			if (index & (1<<(n - i - 1)))
				index_inver = index_inver | (1<<i);
		if (index < index_inver) {
			type_complex temp = x[index];
			x[index] = x[index_inver];
			x[index_inver] = temp;
		}
		
	}
}
        
#ifdef GOBAL

//const type_complex Wnk_fft_G[_N], Wnk_ifft_G[_N];
void init_Wnk(uint8 model, type_complex *Wnk, int N)
{
    if (model) {
      Wnk = (type_complex *)0x410;//Wnk_fft_G;
      for(int i=0;i < N;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //��ŷ����ʽ����fft��ת����  
          Wnk[i].im = -1*sin(2*PI/N*i);  
      }
    }
    else {
      Wnk = (type_complex *)0x10410;//Wnk_ifft_G;
      for(int i=0;i < N;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //��ŷ����ʽ����ifft��ת����  
          Wnk[i].im = sin(2*PI/N*i);  
      }
    }
}
        
#else

// �˺����� Wnk���ڶѣ����ֻ�ܴ�64k���ݡ������Ҫ��������const�ⲿ�����洢��flash
type_complex *init_Wnk(uint8 model, int N)   
{  
    int i;  

    type_complex *Wnk=(type_complex *)malloc(sizeof(type_complex) * N);  //���ɱ任��  

    if (model)
      for(i=0;i < N/2;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //��ŷ����ʽ����fft��ת����  
          Wnk[i].im = -1*sin(2*PI/N*i);  
      }
    else
      for(i=0;i < N;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //��ŷ����ʽ����ifft��ת����  
          Wnk[i].im = sin(2*PI/N*i);  
      }
    
    return Wnk;
}

#endif


void FFT(type_complex x[], type_complex *Wnk, int N)
{
	type_complex product;
	int n = (int)log2(N); //���� 
	
	inver(x, N);
	
	// ��M�� 
	for (int M = 0; M < n; M++) {
		int l = 1<<M; //һ�ε����������������� 
		
		//��j�� 
		for(int j = 0; j < N; j += 2 * l) {
			
			//���ڵ�k���������� 
			for (int k = 0; k < l; k++) {
				int index1 = j + k;
				int index2 = index1 + l;
				
				product = complex_mult(x[index2], Wnk[N*k/(2*l)]);
				
				x[index2] = complex_minus(x[index1], product);
				x[index1] = complex_add(x[index1], product);
		 						 		 		
			}
		}
	} 
}

void NIFFT(type_complex x[], type_complex *Wnk, int N)
{
	type_complex product;
	int n = (int)log2(N); //���� 
	
	inver(x, N);
	
	// ��M�� 
	for (int M = 0; M < n; M++) {
		int l = 1<<M; //һ�ε����������������ͬʱҲ����һ����ε�������� 
		
		//��j�� 
		for(int j = 0; j < N; j += 2 * l) {
			
			//���ڵ�k���������� 
			for (int k = 0; k < l; k++) {
				int index1 = j + k;
				int index2 = index1 + l;
				
				product = complex_mult(x[index2], Wnk[N*k/(2*l)]);
				
				x[index2] = complex_minus(x[index1], product);
				x[index1] = complex_add(x[index1], product);
		 						 		 		
			}
		}
	} 
}

void IFFT(type_complex x[], type_complex *Wnk, int N){
	NIFFT(x, Wnk, N);	
	for(int i = 0; i < N; i++)
		x[i] = complex_build(x[i].re/N, x[i].im/N);
}


////�Բ��α�׼����Ѱ�Ҿ�ֵ������ֵ�����ݷֳ�32��ֱ���
void amplitude_and_mean_process(type_complex sample[])
{
   static int16 max[32] = {0}, min[32] = {0};
      for (int i = 0; i < 32; i++) {
        for (int j = i * _N/2/32; j < (i+1) * _N/2/32; j++) {
          
          if (sample[j].re > sample[max[i]].re)
            max[i] = j;
          else if (sample[j].re < sample[min[i]].re)
            min[i] = j;
          
        }            
      }
      
      static float mean_max = 0, mean_min = 0;
      mean_max = 0, mean_min = 0;
      for (int i = 0; i < 32; i++) {
        mean_max = mean_max + sample[max[i]].re;
        mean_min = mean_min + sample[min[i]].re;    
      }
      
      mean_max = mean_max/32;
      mean_min = mean_min/32;
      
      static float amplitude, mid;
      amplitude = (mean_max - mean_min) / 2;
      mid = (mean_max + mean_min) / 2;
      
      
      for (int i = 0; i < _N/2; i++) {
        sample[i].re = (sample[i].re - mid)/amplitude;        
      }
}


static void low_pass_filter(type_complex sample[])
{
  for (int i = 0; i < _N; i++)
    if (i < 50 || (i > 204 && i < 1843) || i > 1997 || (i > 110 && i < 144) || (i > 1903 && i < 1937)) {
      sample[i].re = 0.0001 * sample[i].re;//sqrt(pow(sample[i].re, 2) + pow(sample[i].im, 2));
      sample[i].im = 0.0001 * sample[i].im;//sqrt(pow(sample[i].re, 2) + pow(sample[i].im, 2));
    }
}

//r_d - r_s
void xcorr(type_complex sample_d[], type_complex sample_s[], type_complex z[], type_complex *Wnk_fft, type_complex *Wnk_ifft, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type * ADC_d, ADC_Type * ADC_s)
{
  for(int i = 0; i < _N; i++) {
        sample_s[i].re = 0;
        sample_s[i].im = 0;
        sample_d[i].re = 0;
        sample_d[i].im = 0;
      }
  
      //int time;
      //LPTMR_TimeStartms();
      for(int i = 0; i < _N/2; i++) {
        sample_s[i].re = (uint16_t)(ADC_Mid(ADC_s, ADC_CH_s, ADC_12bit)/**0.806*/);  //PTE0, ADC�ɼ�����λ�� mv�����д���ִ����Ҫ 19us
        sample_d[_N/2-i-1].re = (uint16_t)(ADC_Mid(ADC_d, ADC_CH_d, ADC_12bit)/**0.806*/);  //PTE1
        systime.delay_us(62);      //��ʱ�Կ��Ʋ���Ƶ�ʣ�Ŀǰ�� DELTA_TIME = 100us �ɼ�һ�����ݣ����㹫ʽΪ DELTA_TIME * 10E6 - 38
      }
      //time = LPTMR_TimeGetms();
      //printf("%d\n", time);
      
      amplitude_and_mean_process(sample_s);
      amplitude_and_mean_process(sample_d);
      
      //slide_average_filter(sample_s, 17);
      //slide_average_filter(sample_d, 17);
      
      FFT(sample_s, Wnk_fft, _N); 
      FFT(sample_d, Wnk_fft, _N);
      
      
      //low_pass_filter(sample_s);
      //low_pass_filter(sample_d);
      
      
      for (int i = 0; i < _N; i++) {
        z[i] = complex_mult(sample_s[i], sample_d[i]);
        
        static float A = 1;
        A =  0.01 * sqrt(pow(z[i].re, 2) + pow(z[i].im, 2));
        z[i].re = z[i].re / A;
        z[i].im = z[i].im / A;
      }
      
      
      IFFT(z, Wnk_ifft, _N);
}


static int midst_filter(int16 max_now, int16 max_queue_ori[], int8 N)
{
  int max_queue[N], flag;
  
  //���и���
  for (int i = N - 1; i >0; i--)
  	max_queue_ori[i] = max_queue_ori[i - 1];
  max_queue_ori[0] = max_now;
  
  //���ɸ��� 
  for (int i = 0; i < N; i++)
  	max_queue[i] = max_queue_ori[i];
  
  for (int i = 0; i < N - 1; i++) { 
	for (int j = 0; j < N - 1 -i; j++)
            if (max_queue[j] > max_queue[j + 1]) {
		int temp = max_queue[j + 1];
                    max_queue[j + 1] = max_queue[j];
			max_queue[j] = temp;
			flag = 0;
            }
            if (flag)
		break; 
  }
  
  if (max_queue[(N - 1)/2] > 1034)
    return 1034;
  else if (max_queue[(N - 1)/2] < 1014)
    return 1014;
  else
    return max_queue[(N - 1)/2]; 
  
}


//r_d - r_s
float distance_difference(float V_sound, type_complex sample_d[], type_complex sample_s[], type_complex z[], type_complex *Wnk_fft, type_complex *Wnk_ifft, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type *ADC_d, ADC_Type *ADC_s)
{
  static int16 max_now, max_chosen, max_queue[9] = {0}; //�������飬0��
   
  xcorr(sample_d, sample_s, z, Wnk_fft, Wnk_ifft, ADC_CH_d, ADC_CH_s, ADC_d, ADC_s);
       
  max_now = 0;
  for (int i = 0; i < _N; i++)
  if (z[i].re > z[max_now].re)
      max_now = i;
  
  max_chosen = midst_filter(max_now, max_queue, 9);
  
   return V_sound * ((max_chosen - _N/2 + 1)  * DELTA_TIME +  19e-6);
}


//���ٱ�ʶ������ʮ�Ρ�
float V_sound_Identification(type_complex sample_d[], type_complex sample_s[], type_complex z[], type_complex *Wnk_fft, type_complex *Wnk_ifft, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type *ADC_d, ADC_Type *ADC_s)
{  
  static int max[TIMES];
  
  for (int i = 0; i < TIMES; i++) {
    xcorr(sample_d, sample_s, z, Wnk_fft, Wnk_ifft, ADC_CH_d, ADC_CH_s, ADC_d, ADC_s);
    
    max[i] = 0;
    for (int j = 0; j < _N; j++)
      if (z[j].re > z[max[i]].re)
        max[i] = j;
  }
  
  int sum_of_max = 0;
  for (int i = 0; i < TIMES; i++)
    sum_of_max += max[i];
  
  sum_of_max /= TIMES;
  
  float V_sound = DISTANCE / ((sum_of_max - _N/2 + 1)  * DELTA_TIME +  19e-6);
  
  return V_sound;
  
  
}





