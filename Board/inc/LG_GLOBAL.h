#ifdef FFT_GLOBALS
#define FFT_EXT
#else
#define FFT_EXT extern
#endif  //FFT_GLOBALS

/**
 * @brief       前四个为麦克风数据，最后一个用来存储互相关结果
 */
FFT_EXT type_complex sample_sx[_N], sample_dx[_N], sample_sy[_N], sample_dy[_N], z[_N];


/**
 * @brief       fft 和 ifft 需要用到的旋转因子
 * @note        由于这两个是常值变量，需要立刻初始化，因此定义在 FFT.c 中，不适用 FFT_EXT 前缀
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];
