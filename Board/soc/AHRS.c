#define AHRS_GLOBALS

#include "include.h"
#include "AHRS.h"

/**
 * @File name    AHRS.c
 * @Brief        姿态解算，航姿参考系统，对外输出RPY
 * @Date        2020/8/6
 *
 */


/**
 * @breif       内部函数、变量及所用到的宏定义
 */
#define new_weight           0.35f
#define old_weight           0.65f
#define M_PI                 3.1415926f
static void IMU_getValues(float * values, accel_t * accelval, gyro_t * gyroval) ;

#define delta_T      0.005f  //5ms计算一次
static float I_ex, I_ey, I_ez;  // 误差积分
static quaterInfo_t Q_info = {1, 0, 0, 0};  // 全局四元数
static const float param_Kp = 50.0;   // 加速度计(磁力计)的收敛速率比例增益50 
static const float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2
static void IMU_AHRSupdate_noMagnetic(float ax, float ay, float az, float gx, float gy, float gz);

static void IMU_quaterToEulerianAngles(void);




/**
 * @brief       更新欧拉角
 * @param
 * @return
 * @example
 * @note        对外接口函数，更新全局变量欧拉角结构体变量
 *
 */
void EulerianAngles_update(void)
{
  static accel_t accelval;
  static gyro_t gyroval;
  
  MPU_Get_Raw_data(&(accelval.x), &(accelval.y), &(accelval.z), &(gyroval.x), &(gyroval.y), &(gyroval.z));	//得到加速度传感器原始数据  
  
  //**ANO_DT_send_senser_int16(accelval.x, accelval.y, accelval.z, gyroval.x, gyroval.y, gyroval.z, 0, 0, 0);
  IMU_getValues(values, &accelval, &gyroval);   //对原始数据进行处理
  IMU_AHRSupdate_noMagnetic(values[0], values[1], values[2], values[3], values[4], values[5]);  //四元数更新
  IMU_quaterToEulerianAngles();         //四元数转换成欧拉角
  
#if 1
  NVIC_DisableIRQ(UART4_RX_TX_IRQn);
  
  ANO_DT_send_status_int16((short)(g_eulerAngle.roll * 100), (short)(g_eulerAngle.pitch * 100), (short)(g_eulerAngle.yaw * 100));             //仅用于测试
  
  NVIC_EnableIRQ(UART4_RX_TX_IRQn);
#endif
}


/**
 * @brief       角度环控制（YAW环控制）
 * @note        此函数会修改 pid_omega.SetSpeed
 */
void get_yaw_and_set_omega(void)
{ 
  EulerianAngles_update();
#if 0
  if(g_eulerAngle.yaw > 10)
    pid_omega.SetSpeed = -1.5;
  else if (g_eulerAngle.yaw < -10)
    pid_omega.SetSpeed = 1.5;
  else
    pid_omega.SetSpeed = pid_yaw.Kp * g_eulerAngle.yaw;         //角度误差与角速度的线性系数，相当于构造一个函数
#endif
  
}



/**
 * @brief       角速度环控制
 * @note        此函数会修改 rpsx_set
 */
void get_omega_and_set_rotation_speed(void)
{
  static float omega_set;      // 角速度更新值
  
  pid_omega.ActualSpeed = values[5];
  omega_set = PID_realize(pid_omega.SetSpeed, &(pid_omega));    
  
  rps1_set += -omega_set /* * 0.18f / 0.19f*/;       // 0.18/0.19近似为1，a + b = 0.18m   PI * d = 0.19m
  rps2_set += -omega_set /* * 0.18f / 0.19f*/;
  rps3_set += omega_set /* * 0.18f / 0.19f*/;
  rps4_set += omega_set /* * 0.18f / 0.19f*/;
}



/****************************************************************************************************************/

//get accel and gyro from iam20609 
// 对accel一阶低通滤波(参考匿名)，对gyro转成弧度每秒(2000dps)

static void IMU_getValues(float * values, accel_t * accelval, gyro_t * gyroval) 
{  
    static float lastaccel[3]= {0,0,0};
    int i;

    values[0] = ((float)accelval->x) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)accelval->y) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)accelval->z) * new_weight + lastaccel[2] * old_weight;

    for(i=0; i<3; i++) {
        lastaccel[i] = values[i];
    }

    values[3] = ((float)gyroval->x) * M_PI / 180 / 16.4f;
    values[4] = ((float)gyroval->y) * M_PI / 180 / 16.4f;
    values[5] = ((float)gyroval->z) * M_PI / 180 / 16.4f;   
}


static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}




/**
  * brief IMU_AHRSupdate_noMagnetic  姿态解算融合，是Crazepony和核心算法
  * 使用的是互补滤波算法，没有使用Kalman滤波算法
  * param float gx, float gy, float gz, float ax, float ay, float az
  *
  * return None
  */
static void IMU_AHRSupdate_noMagnetic(float ax, float ay, float az, float gx, float gy, float gz)
{
    float halfT = 0.5 * delta_T;
    
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    float delta_2 = 0;
    
    
    //对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);       

    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
      
    vx = -2*(q1q3 - q0q2);
    vy = -2*(q0q1 + q2q3);
    vz = -(q0q0 - q1q1 - q2q2 + q3q3);
    
    
    //用叉乘误差来做PI修正陀螺零偏，
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += delta_T * ex;   // integral error scaled by Ki
    I_ey += delta_T * ey;
    I_ez += delta_T * ez;
    
    gx = gx+ param_Kp*ex + param_Ki*I_ex;
    gy = gy+ param_Kp*ey + param_Ki*I_ey;
    gz = gz+ param_Kp*ez + param_Ki*I_ez;

    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;

//    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
//    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			
//    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;	

    // normalise quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}


/**
 * @brief       四元数转换成欧拉角
 * @param
 * @return
 * @example
 * @note        欧拉角单位为 °
 *
 */
static void IMU_quaterToEulerianAngles(void)
{
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    
    g_eulerAngle.pitch = -asin(2*q2*q3 + 2*q0*q1) * 180/M_PI; // pitch
    g_eulerAngle.roll = atan2(-2*q1*q3 + 2*q0*q2, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
    g_eulerAngle.yaw = -atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1) * 180/M_PI; // yaw 
}
