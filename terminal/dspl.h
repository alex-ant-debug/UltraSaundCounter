/*
* Copyright (c) 2015-2019 Sergey Bakhurin
* Digital Signal Processing Library [http://dsplib.org]
*
* This file is part of libdspl-2.0.
*
* is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser  General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSPL is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef DSPL_H
#define DSPL_H



#include <math.h>


/* math const definition */
#ifndef M_PI
  #define M_PI        3.1415926535897932384626433832795
#endif

#ifndef M_2PI
  #define M_2PI        6.283185307179586476925286766559
#endif



typedef double complex_t[2];




typedef struct
{
   complex_t*  w;
   complex_t*  t0;
   complex_t*  t1;
   int         n;
} fft_t;


#define RAND_TYPE_MRG32K3A 0x00000001
#define RAND_TYPE_MT19937  0x00000002

#define RAND_MT19937_NN    312
typedef struct
{

  double mrg32k3a_seed;
  double mrg32k3a_x[3];
  double mrg32k3a_y[3];

  /* The array for the MT19937 state vector */
  unsigned long long mt19937_mt[RAND_MT19937_NN];
  int                mt19937_mti;

  int type;

}random_t;




#define RE(x)           (x[0])
#define IM(x)           (x[1])


#define SQR(x)          ((x) * (x))
#define ABSSQR(x)       ((SQR(RE(x))) + (SQR(IM(x))))
#define ABS(x)          sqrt((ABSSQR(x)))
#define ARG(x)          atan2(IM(x), RE(x))

#define CMRE(a,b)       ((RE(a)) * (RE(b)) - (IM(a)) * (IM(b)))
#define CMIM(a,b)       ((RE(a)) * (IM(b)) + (IM(a)) * (RE(b)))

#define CMCONJRE(a, b)  ((RE(a)) * (RE(b)) + (IM(a)) * (IM(b)))
#define CMCONJIM(a, b)  ((IM(a)) * (RE(b)) - (RE(a)) * (IM(b)))



#define RES_OK                                0

/* Error codes                                          */
/* A                                          0x01xxxxxx*/
/* B                                          0x02xxxxxx*/
/* C                                          0x03xxxxxx*/
/* D                                          0x04xxxxxx*/
#define ERROR_DAT_TYPE                        0x04012020
#define ERROR_DIV_ZERO                        0x04102226
/* E                                          0x05xxxxxx*/
#define ERROR_ELLIP_MODULE                    0x05121315
/* F                                          0x06xxxxxx*/
#define ERROR_FFT_CREATE                      0x06060318
#define ERROR_FFT_SIZE                        0x06062021
#define ERROR_FILTER_A0                       0x06090100
#define ERROR_FILTER_APPROX                   0x06090116
#define ERROR_FILTER_FT                       0x06090620
#define ERROR_FILTER_ORD                      0x06091518
#define ERROR_FILTER_ORD_BP                   0x06091519
#define ERROR_FILTER_RP                       0x06091816
#define ERROR_FILTER_RS                       0x06091819
#define ERROR_FILTER_TYPE                     0x06092025
#define ERROR_FILTER_WP                       0x06092316
#define ERROR_FILTER_WS                       0x06092319
#define ERROR_FNAME                           0x06140113
#define ERROR_FOPEN                           0x06151605
#define ERROR_FREAD_SIZE                      0x06180501
#define ERROR_FWRITE_SIZE                     0x06231820
/* G                                          0x07xxxxxx*/
#define ERROR_GNUPLOT_TERM                    0x07161220
/* H                                          0x08xxxxxx*/
/* I                                          0x09xxxxxx*/
#define ERROR_INC_SIZE                        0x09140319
/* J                                          0x10xxxxxx*/
/* K                                          0x11xxxxxx*/
/* L                                          0x12xxxxxx*/
#define ERROR_LAPACK                          0x12011601
/* M                                          0x13xxxxxx*/
#define ERROR_MATRIX_INDEX                    0x13010914
#define ERROR_MATRIX_SINGULAR                 0x13011914
#define ERROR_MATRIX_SIZE                     0x13011926
/* N                                          0x14xxxxxx*/
#define ERROR_NEGATIVE                        0x14050701
/* O                                          0x15xxxxxx*/
/* P                                          0x16xxxxxx*/
#define ERROR_POLY_ORD                        0x16151518
#define ERROR_PTR                             0x16201800
/* Q                                          0x17xxxxxx*/
/* R                                          0x18xxxxxx*/
#define ERROR_RAND_SIGMA                      0x18011909
#define ERROR_RAND_TYPE                       0x18012009
#define ERROR_RESAMPLE_RATIO                  0x18051801
#define ERROR_RESAMPLE_FRAC_DELAY             0x18050604
/* S                                          0x19xxxxxx*/
#define ERROR_SIZE                            0x19092605
#define ERROR_SYM_TYPE                        0x19251320
/* T                                          0x20xxxxxx*/
/* U                                          0x21xxxxxx*/
#define ERROR_UNWRAP                          0x21142318
/* V                                          0x22xxxxxx*/
/* W                                          0x23xxxxxx*/
#define ERROR_WIN_PARAM                       0x23091601
#define ERROR_WIN_SYM                         0x23091925
#define ERROR_WIN_TYPE                        0x23092025
/* X                                          0x24xxxxxx*/
/* Y                                          0x25xxxxxx*/
/* Z                                          0x26xxxxxx*/

#define DAT_MASK                              0x00000001
#define DAT_DOUBLE                            0x00000000
#define DAT_COMPLEX                           0x00000001

#define DSPL_MATRIX_BLOCK                     32


#define DSPL_SYMMETRIC                        0x00000000
#define DSPL_PERIODIC                         0x00000001

#define DSPL_FLAG_DIGITAL                     0x00000000
#define DSPL_FLAG_ANALOG                      0x00000001
#define DSPL_FLAG_LOGMAG                      0x00000002
#define DSPL_FLAG_UNWRAP                      0x00000004
#define DSPL_FLAG_FFT_SHIFT                   0x00000008



#define DSPL_WIN_SYM_MASK                     0x00000001
#define DSPL_WIN_MASK                         0x00FFFFFE

#define DSPL_WIN_SYMMETRIC                    DSPL_SYMMETRIC
#define DSPL_WIN_PERIODIC                     DSPL_PERIODIC


#define DSPL_WIN_BARTLETT                     0x00000004
#define DSPL_WIN_BARTLETT_HANN                0x00000008
#define DSPL_WIN_BLACKMAN                     0x00000010
#define DSPL_WIN_BLACKMAN_HARRIS              0x00000040
#define DSPL_WIN_BLACKMAN_NUTTALL             0x00000080
#define DSPL_WIN_FLAT_TOP                     0x00000100
#define DSPL_WIN_GAUSSIAN                     0x00000400
#define DSPL_WIN_HAMMING                      0x00000800
#define DSPL_WIN_HANN                         0x00001000
#define DSPL_WIN_LANCZOS                      0x00004000
#define DSPL_WIN_NUTTALL                      0x00008000
#define DSPL_WIN_RECT                         0x00010000
#define DSPL_WIN_COS                          0x00040000
#define DSPL_WIN_CHEBY                        0x00080000
#define DSPL_WIN_KAISER                       0x00100000


#define DSPL_FILTER_TYPE_MASK                 0x000000FF
#define DSPL_FILTER_LPF                       0x00000001
#define DSPL_FILTER_HPF                       0x00000002
#define DSPL_FILTER_BPASS                     0x00000004
#define DSPL_FILTER_BSTOP                     0x00000008

#define DSPL_FILTER_APPROX_MASK               0x0000FF00
#define DSPL_FILTER_BUTTER                    0x00000100
#define DSPL_FILTER_CHEBY1                    0x00000200
#define DSPL_FILTER_CHEBY2                    0x00000400
#define DSPL_FILTER_ELLIP                     0x00000800



#define ELLIP_ITER                            16
#define ELLIP_MAX_ORD                         24

#define  DSPL_VERIF_FAILED                    1
#define  DSPL_VERIF_SUCCESS                   0

#ifdef __cplusplus
  extern "C" {
#endif



#ifdef BUILD_LIB
  /* Declare DSPL_API for Windows OS */
  #ifdef WIN_OS
    #define DSPL_API __declspec(dllexport)
  #endif /* WIN_OS */
  /* Declare DSPL_API for LINUX OS */
  #ifdef LINUX_OS
    #define DSPL_API
  #endif /* LINUX_OS */
#endif /* BUILD_DLL */

#define COMMA ,


#ifdef BUILD_LIB
  #define DECLARE_FUNC(type, fn, param)\
                       type DSPL_API fn(param);
#endif

#ifndef BUILD_LIB
  #define DECLARE_FUNC( type, fn, param)\
                        typedef type (*p_##fn)(param);\
                        extern p_##fn   fn;

#endif


/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        acos_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        asin_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        bessel_i0,                   double*       x
                                                COMMA int           n
                                                COMMA double*       y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        bilinear,                    double*       bs
                                                COMMA double*       as
                                                COMMA int           ord
                                                COMMA double*       bz
                                                COMMA double*       az);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        butter_ap,                   double
                                                COMMA int
                                                COMMA double*
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        butter_ap_zp,                int
                                                COMMA double
                                                COMMA complex_t*
                                                COMMA int*
                                                COMMA complex_t*
                                                COMMA int*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby_poly1,                 double*
                                                COMMA int
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby_poly2,                 double*
                                                COMMA int
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby1_ap,                   double
                                                COMMA int
                                                COMMA double*
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby1_ap_zp,                int
                                                COMMA double
                                                COMMA complex_t*
                                                COMMA int*
                                                COMMA complex_t*
                                                COMMA int*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby2_ap,                   double           rs
                                                COMMA int              ord
                                                COMMA double*          b
                                                COMMA double*          a);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby2_ap_wp1,               double           rp
                                                COMMA double           rs
                                                COMMA int              ord
                                                COMMA double*          b
                                                COMMA double*          a);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cheby2_ap_zp,                int
                                                COMMA double
                                                COMMA complex_t*
                                                COMMA int*
                                                COMMA complex_t*
                                                COMMA int*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cmplx2re,                    complex_t*
                                                COMMA int
                                                COMMA double*
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        concat,                      void*
                                                COMMA size_t
                                                COMMA void*
                                                COMMA size_t
                                                COMMA void*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        conv,                        double*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        conv_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        conv_fft,                    double*           a
                                                COMMA int               na
                                                COMMA double*           b
                                                COMMA int               nb
                                                COMMA fft_t*            pfft
                                                COMMA int               nfft
                                                COMMA double*           c);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        conv_fft_cmplx,              complex_t*        a
                                                COMMA int               na
                                                COMMA complex_t*        b
                                                COMMA int               nb
                                                COMMA fft_t*            pfft
                                                COMMA int               nfft
                                                COMMA complex_t*        c);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        cos_cmplx,                   complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        decimate,                    double*           x
                                                COMMA int               n
                                                COMMA int               d
                                                COMMA double*           y
                                                COMMA int*              cnt);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        decimate_cmplx,              complex_t*        x
                                                COMMA int               n
                                                COMMA int               d
                                                COMMA complex_t*        y
                                                COMMA int*              cnt);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        dft,                         double*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        dft_cmplx,                   complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(double,     dmod,                        double
                                                COMMA double);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(void,       dspl_info,                   void);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_acd,                   double*           w
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA double*           u);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_acd_cmplx,             complex_t*        w
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA complex_t*        u);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_ap,                    double            rp
                                                COMMA double            rs
                                                COMMA int               ord
                                                COMMA double*           b
                                                COMMA double*           a);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_ap_zp,                 int               ord
                                                COMMA double            rp
                                                COMMA double            rs
                                                COMMA complex_t*        z
                                                COMMA int*              nz
                                                COMMA complex_t*        p
                                                COMMA int*              np);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_asn,                   double*           w
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA double*           u);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_asn_cmplx,             complex_t*        w
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA complex_t*        u);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_cd,                    double*           u
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA double*           y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_cd_cmplx,              complex_t*        u
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA complex_t*        y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_landen,                double            k
                                                COMMA int               n
                                                COMMA double*           y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_modulareq,             double            rp
                                                COMMA double            rs
                                                COMMA int               ord
                                                COMMA double*           k);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_rat,                   double*           w
                                                COMMA int               n
                                                COMMA int               ord
                                                COMMA double            k
                                                COMMA double*           u);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_sn,                    double*           u
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA double*           y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ellip_sn_cmplx,              complex_t*        u
                                                COMMA int               n
                                                COMMA double            k
                                                COMMA complex_t*        y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        farrow_lagrange,             double*
                                                COMMA int
                                                COMMA double
                                                COMMA double
                                                COMMA double
                                                COMMA double**
                                                COMMA int*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        farrow_spline,               double*
                                                COMMA int
                                                COMMA double
                                                COMMA double
                                                COMMA double
                                                COMMA double**
                                                COMMA int*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft,                         double*
                                                COMMA int
                                                COMMA fft_t*
                                                COMMA complex_t* );
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_cmplx,                   complex_t*
                                                COMMA int
                                                COMMA fft_t*
                                                COMMA complex_t* );
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_create,                  fft_t*
                                                COMMA int);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(void,       fft_free,                    fft_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_mag,                     double*          x
                                                COMMA int              n
                                                COMMA fft_t*           pfft
                                                COMMA double           fs
                                                COMMA int              flag
                                                COMMA double*          mag
                                                COMMA double*          freq);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_mag_cmplx,               complex_t*       x
                                                COMMA int              n
                                                COMMA fft_t*           pfft
                                                COMMA double           fs
                                                COMMA int              flag
                                                COMMA double*          mag
                                                COMMA double*          freq);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_shift,                   double*
                                                COMMA int n
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fft_shift_cmplx,             complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        filter_freq_resp,            double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA double*          w
                                                COMMA int              n
                                                COMMA int              flag
                                                COMMA double*          mag
                                                COMMA double*          phi
                                                COMMA double*          tau);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        filter_iir,                  double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(double,     filter_ws1,                  int              ord
                                                COMMA double           rp
                                                COMMA double           rs
                                                COMMA int              type);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        filter_zp2ab,                complex_t*
                                                COMMA int
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA int
                                                COMMA double*
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        find_max_abs,                double*        a
                                                COMMA int            n
                                                COMMA double*        m
                                                COMMA int*          ind);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fir_linphase,                int            ord
                                                COMMA double         w0
                                                COMMA double         w1
                                                COMMA int            filter_type
                                                COMMA int            wintype
                                                COMMA double         winparam
                                                COMMA double*        h);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        flipip,                      double*
                                                COMMA int);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        flipip_cmplx,                complex_t*
                                                COMMA int);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fourier_integral_cmplx,      double*         t
                                                COMMA complex_t*      s
                                                COMMA int             nt
                                                COMMA int             nw
                                                COMMA double*         w
                                                COMMA complex_t*      y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fourier_series_dec,          double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double
                                                COMMA int
                                                COMMA double*
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fourier_series_dec_cmplx,    double*         t
                                                COMMA complex_t*      s
                                                COMMA int             nt
                                                COMMA double          period
                                                COMMA int             nw
                                                COMMA double*         w
                                                COMMA complex_t*      y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        fourier_series_rec,          double*
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        freqs,                       double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        freqs_cmplx,                 double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA complex_t*       s
                                                COMMA int              n
                                                COMMA complex_t*       h);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        freqs2time,                  double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double
                                                COMMA int
                                                COMMA fft_t*
                                                COMMA double*
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        freqz,                       double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        gnuplot_script,              int               argc
                                                COMMA char*             argv[]
                                                COMMA char*             fn);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        goertzel,                    double*
                                                COMMA int
                                                COMMA int*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        goertzel_cmplx,              complex_t*
                                                COMMA int
                                                COMMA int*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        histogram,                   double*          x
                                                COMMA int              n
                                                COMMA int              nh
                                                COMMA double*          pedges
                                                COMMA double*          ph);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        histogram_norm,              double*          y
                                                COMMA int              n
                                                COMMA int              nh
                                                COMMA double*          x
                                                COMMA double*          w);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        idft_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ifft_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA fft_t*
                                                COMMA complex_t* );
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        iir,                         double           rp
                                                COMMA double           rs
                                                COMMA int              ord
                                                COMMA double           w0
                                                COMMA double           w1
                                                COMMA int              type
                                                COMMA double*          b
                                                COMMA double*          a);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        linspace,                    double
                                                COMMA double
                                                COMMA int
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        log_cmplx,                   complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        logspace,                    double
                                                COMMA double
                                                COMMA int
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        low2bp,                      double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA double           w0
                                                COMMA double           wpl
                                                COMMA double           wph
                                                COMMA double*          beta
                                                COMMA double*          alpha);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        low2bs,                      double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA double           w0
                                                COMMA double           wsl
                                                COMMA double           wsh
                                                COMMA double*          beta
                                                COMMA double*          alpha);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        low2high,                    double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA double           w0
                                                COMMA double           w1
                                                COMMA double*          beta
                                                COMMA double*          alpha);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        low2low,                     double*          b
                                                COMMA double*          a
                                                COMMA int              ord
                                                COMMA double           w0
                                                COMMA double           w1
                                                COMMA double*          beta
                                                COMMA double*          alpha);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_eig_cmplx,            complex_t*       a
                                                COMMA int              n
                                                COMMA complex_t*       v
                                                COMMA int*             info);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_eye,                  double*          a
                                                COMMA int              n
                                                COMMA int              m);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_eye_cmplx,            complex_t*       a
                                                COMMA int              n
                                                COMMA int              m);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_mul,                  double*          a
                                                COMMA int              na
                                                COMMA int              ma
                                                COMMA double*          b
                                                COMMA int              nb
                                                COMMA int              mb
                                                COMMA double*          c);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_print,                double*          a
                                                COMMA int              n
                                                COMMA int              m
                                                COMMA const char*      name
                                                COMMA const char*      format);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_print_cmplx,          complex_t*       a
                                                COMMA int              n
                                                COMMA int              m
                                                COMMA const char*      name
                                                COMMA const char*      format);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_transpose,            double*          a
                                                COMMA int              n
                                                COMMA int              m
                                                COMMA double*          b);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_transpose_cmplx,      complex_t*       a
                                                COMMA int              n
                                                COMMA int              m
                                                COMMA complex_t*       b);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        matrix_transpose_hermite,    complex_t*       a
                                                COMMA int              n
                                                COMMA int              m
                                                COMMA complex_t*       b);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        minmax,                      double*          x
                                                COMMA int              n
                                                COMMA double*          xmin
                                                COMMA double*          xmax);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        poly_z2a_cmplx,              complex_t*
                                                COMMA int
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        polyval,                     double*
                                                COMMA int
                                                COMMA double*
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        polyval_cmplx,               complex_t*
                                                COMMA int
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        randn,                       double*
                                                COMMA int
                                                COMMA double
                                                COMMA double
                                                COMMA random_t*        prnd);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        random_init,                 random_t*        prnd
                                                COMMA int              type
                                                COMMA void*            seed);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        randu,                       double*
                                                COMMA int
                                                COMMA random_t*        prnd);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        ratcompos,                   double*          b
                                                COMMA double*          a
                                                COMMA int              n
                                                COMMA double*          c
                                                COMMA double*          d
                                                COMMA int              p
                                                COMMA double*          beta
                                                COMMA double*          alpha);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        re2cmplx,                    double*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        readbin,                     char*           fn
                                                COMMA void**          x
                                                COMMA int*            k
                                                COMMA int*            dtype);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        signal_pimp,                 double*
                                                COMMA size_t
                                                COMMA double
                                                COMMA double
                                                COMMA double
                                                COMMA double
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        signal_saw,                  double*          t
                                                COMMA size_t           n
                                                COMMA double           amp
                                                COMMA double           dt
                                                COMMA double           period
                                                COMMA double*          y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        sin_cmplx,                   complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        sinc,                        double*          x
                                                COMMA int              n
                                                COMMA double           a
                                                COMMA double*          y);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        sine_int,                    double*          x
                                                COMMA int              n
                                                COMMA double*          si);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        sqrt_cmplx,                  complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        trapint,                     double*
                                                COMMA double*
                                                COMMA int
                                                COMMA double*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        trapint_cmplx,               double*
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA complex_t*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        unwrap,                      double*
                                                COMMA int
                                                COMMA double
                                                COMMA double);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        vector_dot,                  double* x
                                                COMMA double* y
                                                COMMA int     n
                                                COMMA double* p);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        verif,                       double*          x
                                                COMMA double*          y
                                                COMMA size_t           n
                                                COMMA double           eps
                                                COMMA double*          err);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        verif_cmplx,                 complex_t*       x
                                                COMMA complex_t*       y
                                                COMMA size_t           n
                                                COMMA double           eps
                                                COMMA double*          err);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        window,                      double*          w
                                                COMMA int              n
                                                COMMA int              win_type
                                                COMMA double           param);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writebin,                    void*
                                                COMMA int
                                                COMMA int
                                                COMMA char*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writetxt,                    double*
                                                COMMA double*
                                                COMMA int
                                                COMMA char*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writetxt_3d,                 double*          x
                                                COMMA int              nx
                                                COMMA double*          y
                                                COMMA int              ny
                                                COMMA double*          z
                                                COMMA char*            fn);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writetxt_3dline,             double*          x
                                                COMMA double*          y
                                                COMMA double*          z
                                                COMMA int              n
                                                COMMA char*            fn);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writetxt_cmplx_im,           double*
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA char*);
/*----------------------------------------------------------------------------*/
DECLARE_FUNC(int,        writetxt_cmplx_re,           double*
                                                COMMA complex_t*
                                                COMMA int
                                                COMMA char*);
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
  }
#endif


void* dspl_load();
void  dspl_free(void* handle);



#endif /* DSPL_H */

