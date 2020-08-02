/**
 * @brief       全局变量管理文件
 * @note        对应文件的宏定义一定要放在此头文件的包含(#include)之前
 *
 */


#ifdef FFT_GLOBALS
#define FFT_EXT
#else
#define FFT_EXT extern
#endif  //FFT_GLOBALS

/**
 * @brief       前四个为麦克风数据，最后一个用来存储互相关结果
 */
FFT_EXT type_complex sample_sx[_N], sample_dx[_N], sample_sy[_N], sample_dy[_N], z_x[_N], z_y[_N];


/**
 * @brief       fft 和 ifft 需要用到的旋转因子
 * @note        由于这两个是常值变量，需要立刻初始化，因此定义在 FFT.c 中，不适用 FFT_EXT 前缀
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];




#ifdef  ADCDMA_GLOBALS
#define ADCDMA_EXT
#else
#define ADCDMA_EXT extern
#endif  //ADCDMA_GLOBALS

/**
 * @brief       引脚复用存储函数，用于 DMA  传输，实现自动更换ADC 通道的功能
 */
extern uint8_t g_ADC0_mux[2];
extern uint8_t g_ADC1_mux[2];

/**
 * @brief       x轴的ADC已采集2048个点，可以进行数据分散和FFT了，在 task1 中使用
 */
ADCDMA_EXT uint8_t x_ready_for_fft;
ADCDMA_EXT uint8_t y_ready_for_fft;




#ifdef  PID_GLOBALS
#define PID_EXT
#else
#define PID_EXT extern
#endif  //PID_GLOBALS
/**
 * @brief       pid 控制结构体
 */
typedef struct { 
    float SetSpeed; //定义设定值 
    float ActualSpeed; //定义实际值 
    float err; //定义偏差值 
    float err_next; //定义上一个偏差值 
    float err_last; //定义上一个偏差值 
    float Kp,Ki,Kd; //定义比例、积分、微分系数
    float epsilon; //偏差检测阈值
} pid_t;

/**
 * @brief       四轮pid控制实例
 */
PID_EXT pid_t pid[4];



