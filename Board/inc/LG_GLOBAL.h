/**
 * @brief       ȫ�ֱ��������ļ�
 * @note        ��Ӧ�ļ��ĺ궨��һ��Ҫ���ڴ�ͷ�ļ��İ���(#include)֮ǰ
 *
 */


#ifdef FFT_GLOBALS
#define FFT_EXT
#else
#define FFT_EXT extern
#endif  //FFT_GLOBALS

/**
 * @brief       ǰ�ĸ�Ϊ��˷����ݣ����һ�������洢����ؽ��
 */
FFT_EXT type_complex sample_sx[_N], sample_dx[_N], sample_sy[_N], sample_dy[_N], z_x[_N], z_y[_N];


/**
 * @brief       fft �� ifft ��Ҫ�õ�����ת����
 * @note        �����������ǳ�ֵ��������Ҫ���̳�ʼ������˶����� FFT.c �У������� FFT_EXT ǰ׺
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];




#ifdef  ADCDMA_GLOBALS
#define ADCDMA_EXT
#else
#define ADCDMA_EXT extern
#endif  //ADCDMA_GLOBALS

/**
 * @brief       ���Ÿ��ô洢���������� DMA  ���䣬ʵ���Զ�����ADC ͨ���Ĺ���
 */
extern uint8_t g_ADC0_mux[2];
extern uint8_t g_ADC1_mux[2];

/**
 * @brief       x���ADC�Ѳɼ�2048���㣬���Խ������ݷ�ɢ��FFT�ˣ��� task1 ��ʹ��
 */
ADCDMA_EXT uint8_t x_ready_for_fft;
ADCDMA_EXT uint8_t y_ready_for_fft;




#ifdef  PID_GLOBALS
#define PID_EXT
#else
#define PID_EXT extern
#endif  //PID_GLOBALS
/**
 * @brief       pid ���ƽṹ��
 */
typedef struct { 
    float SetSpeed; //�����趨ֵ 
    float ActualSpeed; //����ʵ��ֵ 
    float err; //����ƫ��ֵ 
    float err_next; //������һ��ƫ��ֵ 
    float err_last; //������һ��ƫ��ֵ 
    float Kp,Ki,Kd; //������������֡�΢��ϵ��
    float epsilon; //ƫ������ֵ
} pid_t;

/**
 * @brief       ����pid����ʵ��
 */
PID_EXT pid_t pid[4];



