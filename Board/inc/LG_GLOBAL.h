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
 * @brief       前四个元素为四个麦克风信号的幅值， 第五元素为幅值平均值
 */
FFT_EXT float amplitude[5]; 


/**
 * @brief       fft 和 ifft 需要用到的旋转因子
 * @note        由于这两个是常值变量，需要立刻初始化，因此定义在 FFT.c 中，不适用 FFT_EXT 前缀
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];

/**
 * @brief        chirp 信号翻转补零FFT后的结果
 * @note        由于这两个是常值变量，需要立刻初始化，因此定义在 FFT.c 中，不适用 FFT_EXT 前缀
 */
extern const type_complex ref_chirp[];



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

/**
 * @brief       角速度pid控制实例
 */
PID_EXT pid_t pid_omega;

/**
 * @brief       偏航角pid控制实例
 */
PID_EXT pid_t pid_yaw;




#ifdef  MOTOR_GLOBALS
#define MOTOR_EXT
#else
#define MOTOR_EXT extern
#endif  //MOTOR_GLOBALS
/**
 * @brief       车体运动状态
 */
typedef enum {
  CAR_STOP,             // 车停
  CAR_RUN,              // 车在校准模式下行驶
  CAR_NORMAL_RUN,       // 车正常行驶，以单次计算获得的角度为方位
  CAR_BACK,        //车后退，以离开障碍物
  CAR_OBLIQUE           //车斜着前进，避开障碍物
} car_status_t;

/**
 * @brief       车体运动状态变量
 */
MOTOR_EXT car_status_t car_status;

/**
 * @brief       上次的车状态
 */
MOTOR_EXT car_status_t last_car_status;

#ifdef  ENC_GLOBALS
#define ENC_EXT 
#else
#define ENC_EXT extern
#endif  //ENC_GLOBALS
/**
 * @brief       四轮速度
 */
ENC_EXT float rps_1;
ENC_EXT float rps_2;
ENC_EXT float rps_3;
ENC_EXT float rps_4;

/**
 * @brief       四轮的目标速度
 * @note        合速度应包含平动部分和旋转部分
 */
ENC_EXT float rps1_set;
ENC_EXT float rps2_set;
ENC_EXT float rps3_set;
ENC_EXT float rps4_set;



#ifdef  AHRS_GLOBALS
#define AHRS_EXT        
#else
#define AHRS_EXT extern
#endif
/**
 * @brief       欧拉角结构体
 */
typedef struct {
  float pitch;
  float yaw;
  float roll;
} eulerianAngles_t;

/**
 * @brief       欧拉角实例
 */
AHRS_EXT eulerianAngles_t g_eulerAngle;

/**
 * @brief       处理后的加速度和角速度数据，单位为 LSB 和 弧度每秒
 * @note        数据内容依次为 ACC_X   ACC_Y   ACC_Z   GYRO_X  GYRO_Y  GYRO_Z
 */
AHRS_EXT float values[6];



#ifdef  MAIN_GLOBALS
#define MAIN_EXT
#else
#define MAIN_EXT extern
#endif
/**
 * @brief       模式选择标志位
 */
MAIN_EXT int8_t model[6];



