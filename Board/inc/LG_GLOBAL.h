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
 * @brief       ǰ�ĸ�Ԫ��Ϊ�ĸ���˷��źŵķ�ֵ�� ����Ԫ��Ϊ��ֵƽ��ֵ
 */
FFT_EXT float amplitude[5]; 


/**
 * @brief       fft �� ifft ��Ҫ�õ�����ת����
 * @note        �����������ǳ�ֵ��������Ҫ���̳�ʼ������˶����� FFT.c �У������� FFT_EXT ǰ׺
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];

/**
 * @brief        chirp �źŷ�ת����FFT��Ľ��
 * @note        �����������ǳ�ֵ��������Ҫ���̳�ʼ������˶����� FFT.c �У������� FFT_EXT ǰ׺
 */
extern const type_complex ref_chirp[];



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

/**
 * @brief       ���ٶ�pid����ʵ��
 */
PID_EXT pid_t pid_omega;

/**
 * @brief       ƫ����pid����ʵ��
 */
PID_EXT pid_t pid_yaw;




#ifdef  MOTOR_GLOBALS
#define MOTOR_EXT
#else
#define MOTOR_EXT extern
#endif  //MOTOR_GLOBALS
/**
 * @brief       �����˶�״̬
 */
typedef enum {
  CAR_STOP,             // ��ͣ
  CAR_RUN,              // ����У׼ģʽ����ʻ
  CAR_NORMAL_RUN,       // ��������ʻ���Ե��μ����õĽǶ�Ϊ��λ
  CAR_BACK,        //�����ˣ����뿪�ϰ���
  CAR_OBLIQUE           //��б��ǰ�����ܿ��ϰ���
} car_status_t;

/**
 * @brief       �����˶�״̬����
 */
MOTOR_EXT car_status_t car_status;

/**
 * @brief       �ϴεĳ�״̬
 */
MOTOR_EXT car_status_t last_car_status;

#ifdef  ENC_GLOBALS
#define ENC_EXT 
#else
#define ENC_EXT extern
#endif  //ENC_GLOBALS
/**
 * @brief       �����ٶ�
 */
ENC_EXT float rps_1;
ENC_EXT float rps_2;
ENC_EXT float rps_3;
ENC_EXT float rps_4;

/**
 * @brief       ���ֵ�Ŀ���ٶ�
 * @note        ���ٶ�Ӧ����ƽ�����ֺ���ת����
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
 * @brief       ŷ���ǽṹ��
 */
typedef struct {
  float pitch;
  float yaw;
  float roll;
} eulerianAngles_t;

/**
 * @brief       ŷ����ʵ��
 */
AHRS_EXT eulerianAngles_t g_eulerAngle;

/**
 * @brief       �����ļ��ٶȺͽ��ٶ����ݣ���λΪ LSB �� ����ÿ��
 * @note        ������������Ϊ ACC_X   ACC_Y   ACC_Z   GYRO_X  GYRO_Y  GYRO_Z
 */
AHRS_EXT float values[6];



#ifdef  MAIN_GLOBALS
#define MAIN_EXT
#else
#define MAIN_EXT extern
#endif
/**
 * @brief       ģʽѡ���־λ
 */
MAIN_EXT int8_t model[6];



