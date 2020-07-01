#ifdef FFT_GLOBALS
#define FFT_EXT
#else
#define FFT_EXT extern
#endif  //FFT_GLOBALS

/**
 * @brief       ǰ�ĸ�Ϊ��˷����ݣ����һ�������洢����ؽ��
 */
FFT_EXT type_complex sample_sx[_N], sample_dx[_N], sample_sy[_N], sample_dy[_N], z[_N];


/**
 * @brief       fft �� ifft ��Ҫ�õ�����ת����
 * @note        �����������ǳ�ֵ��������Ҫ���̳�ʼ������˶����� FFT.c �У������� FFT_EXT ǰ׺
 */
extern const type_complex kWnk_fft[];
extern const type_complex kWnk_ifft[];
