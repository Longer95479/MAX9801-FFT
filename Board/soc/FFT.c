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

void inver(type_complex x[],int N)
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
          Wnk[i].re = cos(2*PI/N*i);   //用欧拉公式计算fft旋转因子  
          Wnk[i].im = -1*sin(2*PI/N*i);  
      }
    }
    else {
      Wnk = (type_complex *)0x10410;//Wnk_ifft_G;
      for(int i=0;i < N;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //用欧拉公式计算ifft旋转因子  
          Wnk[i].im = sin(2*PI/N*i);  
      }
    }
}
        
#else

// 此函数把 Wnk放在堆，最多只能存64k数据。因此我要把声明成const外部变量存储在flash
type_complex *init_Wnk(uint8 model, int N)   
{  
    int i;  

    type_complex *Wnk=(type_complex *)malloc(sizeof(type_complex) * N);  //生成变换核  

    if (model)
      for(i=0;i < N/2;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //用欧拉公式计算fft旋转因子  
          Wnk[i].im = -1*sin(2*PI/N*i);  
      }
    else
      for(i=0;i < N;i++) {  
          Wnk[i].re = cos(2*PI/N*i);   //用欧拉公式计算ifft旋转因子  
          Wnk[i].im = sin(2*PI/N*i);  
      }
    
    return Wnk;
}

#endif


void FFT(type_complex x[], type_complex *Wnk, int N)
{
	type_complex product;
	int n = (int)log2(N); //级数 
	
	inver(x, N);
	
	// 第M级 
	for (int M = 0; M < n; M++) {
		int l = 1<<M; //一次蝶形运算数组索引差 
		
		//第j组 
		for(int j = 0; j < N; j += 2 * l) {
			
			//组内第k个蝶形运算 
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
	int n = (int)log2(N); //级数 
	
	inver(x, N);
	
	// 第M级 
	for (int M = 0; M < n; M++) {
		int l = 1<<M; //一次蝶形运算数组索引差，同时也等于一组蝶形的运算次数 
		
		//第j组 
		for(int j = 0; j < N; j += 2 * l) {
			
			//组内第k个蝶形运算 
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
