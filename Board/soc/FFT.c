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


void filter(type_complex sample[])
{
  for (int i = 0; i < _N; i++)
    if (i < 25 || (i > 130 && i < 1917) || i > 2022)
      sample[i].re = 0;
}


void xcorr(type_complex sample_d[], type_complex sample_s[], type_complex z[], type_complex *Wnk_fft, type_complex *Wnk_ifft)
{
  for(int i = 0; i < _N; i++) {
        sample_s[i].re = 0;
        sample_s[i].im = 0;
        sample_d[i].re = 0;
        sample_d[i].im = 0;
      }
      
      //LPTMR_TimeStartms();
      for(int i = 0; i < _N/2; i++) {
        sample_s[i].re = (uint16_t)(ADC_Get(0)*0.806);  //PTB4, ADC�ɼ�����λ�� mv�����д���ִ����Ҫ 19us
        sample_d[_N/2-i-1].re = (uint16_t)(ADC_Get(1)*0.806);  //PTB5
        systime.delay_us(12); //����⣬�����ʵ��ms��ʱ      //��ʱ�Կ��Ʋ���Ƶ�ʣ�Ŀǰ��50us�ɼ�һ������
      }
      //time = LPTMR_TimeGetms();

      
      amplitude_and_mean_process(sample_s);
      amplitude_and_mean_process(sample_d);
    
      
      FFT(sample_s, Wnk_fft, _N);      
      FFT(sample_d, Wnk_fft, _N);
      
      filter(sample_s);
      filter(sample_d);
      
      for (int i = 0; i < _N; i++) {
        z[i] = complex_mult(sample_s[i], sample_d[i]);
      }
      
      IFFT(z, Wnk_ifft, _N);
}


