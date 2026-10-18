#include "attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void check(const Ekf6 *s) {
    float n=0; unsigned i,j;
    for(i=0;i<4;++i) { assert(isfinite(s->q[i])); n+=s->q[i]*s->q[i]; }
    assert(fabsf(n-1)<1e-5f);
    for(i=0;i<5;++i) { assert(s->covariance[i][i]>=0); for(j=0;j<5;++j) assert(fabsf(s->covariance[i][j]-s->covariance[j][i])<1e-6f); }
}
int main(void) {
    Ekf6 s,before; float a[3]={0,0,1},g[3]={0,0,0},e[3]; unsigned i;
    assert(ekf6_init(&s,a));
    for(i=0;i<20000;++i) { assert(ekf6_update(&s,g,a,0.005f)); check(&s); }
    ekf6_euler(&s,e); assert(fabsf(e[0])+fabsf(e[1])+fabsf(e[2])<0.001f);
    /* Variable dt and slow yaw below source's 0.008 rad/s cutoff. */
    g[2]=0.004f;
    for(i=0;i<2000;++i) assert(ekf6_update(&s,g,a,(i%2)?0.007f:0.003f));
    ekf6_euler(&s,e); assert(fabsf(e[2]-0.04f*RAD_TO_DEG)<0.01f);
    assert(ekf6_init(&s,a)); g[2]=1.0f;
    for(i=0;i<1600;++i) { assert(ekf6_update(&s,g,a,0.005f)); check(&s); }
    ekf6_euler(&s,e); assert(fabsf(e[2]-(8.0f*RAD_TO_DEG-360))<0.03f);
    /* Tilted initialization, X/Y residual gyro drift is learned. */
    a[0]=-sinf(0.2f); a[1]=cosf(0.2f)*sinf(0.3f); a[2]=cosf(0.2f)*cosf(0.3f);
    assert(ekf6_init(&s,a)); ekf6_euler(&s,e);
    assert(fabsf(e[0]-0.3f*RAD_TO_DEG)<0.001f && fabsf(e[1]-0.2f*RAD_TO_DEG)<0.001f);
    a[0]=a[1]=0; a[2]=1; assert(ekf6_init(&s,a));
    g[0]=0.01f; g[1]=-0.015f; g[2]=0;
    for(i=0;i<12000;++i) { assert(ekf6_update(&s,g,a,0.005f)); check(&s); }
    ekf6_euler(&s,e); assert(fabsf(e[0])+fabsf(e[1])<0.1f);
    assert(fabsf(s.bias_xy[0]-g[0])<0.001f && fabsf(s.bias_xy[1]-g[1])<0.001f);
    /* Linear acceleration with nearly unit norm must trip innovation gate. */
    a[0]=0.6f; a[2]=0.8f; g[0]=g[1]=0;
    for(i=0;i<30;++i) assert(ekf6_update(&s,g,a,0.005f));
    assert(!s.accel_used && s.innovation_nis>EKF_CHI_SQUARE_LIMIT);
    /* Freefall/impact are prediction-only, NaN/bad dt leave state intact. */
    a[0]=a[1]=a[2]=0; assert(ekf6_update(&s,g,a,0.005f) && !s.accel_used);
    a[2]=2; assert(ekf6_update(&s,g,a,0.005f) && !s.accel_used);
    before=s; g[0]=NAN; assert(!ekf6_update(&s,g,a,0.005f)); assert(memcmp(&s,&before,sizeof(s))==0);
    g[0]=0; assert(!ekf6_update(&s,g,a,0.1f)); assert(memcmp(&s,&before,sizeof(s))==0);
    /* Rotating about all axes: synthetic gravity from an independent gyro
     * propagation oracle; checks multiplication/sign conventions when tilted. */
    {
        Mahony6 truth; float expected[3];
        a[0]=a[1]=0; a[2]=1; assert(mahony6_init(&truth,a)); assert(ekf6_init(&s,a));
        for(i=0;i<4000;++i) {
            float w,x,y,z,invalid_acc[3]={0,0,0};
            g[0]=0.2f*sinf(i*0.005f); g[1]=0.15f*cosf(i*0.004f); g[2]=0.4f;
            assert(mahony6_update(&truth,g,invalid_acc,0.005f));
            w=truth.q[0]; x=truth.q[1]; y=truth.q[2]; z=truth.q[3];
            a[0]=2*(x*z-w*y); a[1]=2*(w*x+y*z); a[2]=w*w-x*x-y*y+z*z;
            assert(ekf6_update(&s,g,a,0.005f)); check(&s);
        }
        ekf6_euler(&s,e); mahony6_euler(&truth,expected);
        for(i=0;i<3;++i) assert(fabsf(e[i]-expected[i])<1.0f);
    }
    /* Regression: rotating horizontal linear acceleration is not true yaw.
     * Original implementation drifted -27.26 / -88.36 degrees in 30s at
     * gyro=0, with/without intermittent rejected acceleration frames. */
    for(int mode=0;mode<3;++mode) {
        a[0]=a[1]=0; a[2]=1; g[0]=g[1]=g[2]=0;
        assert(ekf6_init(&s,a));
        for(i=0;i<6000;++i) {
            float t=i*.005f;
            a[0]=.12f*sinf(t*6.2831853f); a[1]=.12f*cosf(t*6.2831853f);
            a[2]=(mode==1 && i%3==0)?1.5f:1;
            g[2]=mode==2?.1f:0;
            assert(ekf6_update(&s,g,a,.005f)); check(&s);
        }
        ekf6_euler(&s,e);
        assert(fabsf(e[2]-(mode==2?3*RAD_TO_DEG:0))<.15f);
    }
    /* Reset removes old bias/covariance/filter history. */
    a[0]=a[1]=0; a[2]=1; assert(ekf6_init(&s,a)); assert(s.bias_xy[0]==0 && s.bias_xy[1]==0); check(&s);
    puts("EKF: static, variable dt, slow/multi-turn yaw, tilt, bias, acceleration gating, invalid input/reset passed");
    return 0;
}
