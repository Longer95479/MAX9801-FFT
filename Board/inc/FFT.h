#ifndef _FFT_H
#define _FFT_H

#define PI 3.141592f
#define fft 1
#define ifft 0
#define _N 4096
#define _L 0.27f    //�������ٱ�ʶ��������˷����
#define TIMES 50        //���ٱ�ʶʱ�ظ���������
#define DELTA_TIME 1e-4f

//#define GOBAL

typedef struct {
	float re; //really
	float im; //imaginary
} type_complex;

/*typedef struct {
  type_complex *sample_s1;
  type_complex *sample_d1;
  type_complex *sample_s2;
  type_complex *sample_d2;
  
}*/

type_complex complex_build(float re0, float im0);
type_complex complex_add(type_complex cx1, type_complex cx2);
type_complex complex_minus(type_complex cx1, type_complex cx2);
type_complex complex_mult(type_complex cx1, type_complex cx2);

/**
 * @brief       ����Ŀ������㣬�ȿ⿪����4��
 * @param       x����������
 * @return      1/x^0.5
 * @example
 * @note
 *
 */
float InvSqrt(float x);
/**
 * @brief       ��λ���˲�
 * @param        max_now: �¼�����Ļ�������ֵ����
 *                max_queue_ori[]: ��ʷ����������ڴ˶�����
 *                N: ���г���
 *
 * @return      max_queue[(N - 1)/2]: 
 * @example     
 * @note        ���ڶԻ���ؽ�������ֵ���� max  �����˲�
 *
 */
int midst_filter(int16 max_now, int16 max_queue_ori[], int8 N);

static void inver(type_complex x[],int N);
void FFT(type_complex x[], const type_complex *Wnk, int N);
void NIFFT(type_complex x[], const type_complex *Wnk, int N);
void IFFT(type_complex x[], const type_complex *Wnk, int N);



/**
 * @brief      ��ͨ�˲�
 * @param       sample[]: ���˲���Ƶ������
 * @return
 * @example     
 * @note        �˺���ʵ��Ƶ���˲���ֱ�ӽ�����Ҫ��Ƶ�ʳɷֳ���һ����С��ֵ
 */
void low_pass_filter(type_complex sample[]);
void sample_get(void);
void amplitude_and_mean_process(type_complex sample[]);
void xcorr(type_complex sample_d[], type_complex sample_s[], type_complex z[], const type_complex *Wnk_fft, const type_complex *Wnk_ifft/*, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type * ADC_d, ADC_Type * ADC_s*/);
float V_sound_Identification(type_complex sample_d[], type_complex sample_s[], type_complex z[], const type_complex *Wnk_fft, const type_complex *Wnk_ifft/*, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type * ADC_d, ADC_Type * ADC_s*/);
float distance_difference(float V_sound, type_complex sample_d[], type_complex sample_s[], type_complex z[], const type_complex *Wnk_fft, const type_complex *Wnk_ifft/*, ADCn_Ch_e ADC_CH_d, ADCn_Ch_e ADC_CH_s, ADC_Type * ADC_d, ADC_Type * ADC_s*/);

#endif
