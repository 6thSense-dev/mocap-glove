#include "lockin.h"
#include <math.h>
/* Coherent bins 12,15,18 at 24 kSPS; signed real/imaginary, counts x1000. */
void lockin64(const int16_t samples[64][3],int32_t out[3][3][2]){
 static float basis[3][64][2];static int ready;
 if(!ready){for(unsigned t=0;t<3;t++)for(unsigned n=0;n<64;n++){float a=6.2831853071795864769f*(12+3*t)*n/64;basis[t][n][0]=cosf(a)/32;basis[t][n][1]=-sinf(a)/32;}ready=1;}
 for(unsigned t=0;t<3;t++)for(unsigned c=0;c<3;c++)for(unsigned part=0;part<2;part++){
  float sum=0;for(unsigned n=0;n<64;n++)sum+=samples[n][c]*basis[t][n][part];out[t][c][part]=(int32_t)lroundf(sum*1000);
 }
}
