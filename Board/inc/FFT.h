#ifndef _FFT_H
#define _FFT_H

#define PI 3.1415926
#define fft 1
#define ifft 0
#define _N 2048

//#define GOBAL

typedef struct {
	float re; //really
	float im; //imaginary
} type_complex;

type_complex complex_build(float re0, float im0);
type_complex complex_add(type_complex cx1, type_complex cx2);
type_complex complex_minus(type_complex cx1, type_complex cx2);
type_complex complex_mult(type_complex cx1, type_complex cx2);

void inver(type_complex x[],int N);
void FFT(type_complex x[], type_complex *Wnk, int N);
void NIFFT(type_complex x[], type_complex *Wnk, int N);
void IFFT(type_complex x[], type_complex *Wnk, int N);

#ifdef GOBAL
void init_Wnk(uint8 model, type_complex *Wnk, int N);
#else
type_complex *init_Wnk(uint8 model, int N);
#endif //GOBAL

void amplitude_and_mean_process(type_complex sample[]);


#endif
