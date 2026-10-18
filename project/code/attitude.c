/** 姿态算法独立于 SPI、屏幕和 WiFi；保留原算法计算顺序及数值门限。
 * Madgwick 来源说明：Sebastian O.H. Madgwick IMU 算法报告
 * https://x-io.co.uk/downloads/madgwick_internal_report.pdf
 * 原项目实现声明 GPL-3.0-or-later；本次合并保留该声明。
 */
#include "attitude.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 避免快速浮点优化将 NaN/Inf 比较优化掉；memcpy 避免别名访问。 */
static int finite_float(float value)
{
    uint32_t bits;
    typedef char float_size_check[(sizeof(float)==sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(float_size_check);
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}
static float attitude_clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
void calibration_reset(ImuCalibration *c)
{
    memset(c, 0, sizeof(*c));
}
/* 400 个连续合格样本，200Hz 下约 2 秒；不能识别所有匀速运动。 */
int calibration_push(ImuCalibration *c, const float g[3], const float a[3])
{
    unsigned i;
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n) || n < 0.9f || n > 1.1f) {
        calibration_reset(c); return 0;
    }
    for (i=0; i<3; ++i) {
        if (!finite_float(g[i]) || fabsf(g[i]) > CAL_GYRO_LIMIT_DPS) {
            calibration_reset(c); return 0;
        }
        if (!c->count) {
            c->min_g[i]=c->max_g[i]=g[i]; c->min_a[i]=c->max_a[i]=a[i];
        }
        if (g[i]<c->min_g[i]) c->min_g[i]=g[i];
        if (g[i]>c->max_g[i]) c->max_g[i]=g[i];
        if (a[i]<c->min_a[i]) c->min_a[i]=a[i];
        if (a[i]>c->max_a[i]) c->max_a[i]=a[i];
        /* 峰峰值超限：丢弃整个静止窗口。 */
        if (c->max_g[i]-c->min_g[i]>CAL_GYRO_SPAN_DPS ||
            c->max_a[i]-c->min_a[i]>CAL_ACCEL_SPAN_G) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) { c->sum_g[i]+=g[i]; c->sum_a[i]+=a[i]; }
    if (++c->count < CALIBRATION_SAMPLES) return 0;
    for (i=0; i<3; ++i) {
        c->bias_dps[i]=c->sum_g[i]/c->count;
        c->initial_accel[i]=c->sum_a[i]/c->count;
    }
    c->bias_z_ref_dps=c->bias_dps[2];
    return 1;
}
/* 深度静止期间缓慢跟踪 z 轴陀螺零偏：把零偏以时间常数 TAU 一阶拉向当前
 * 原始读数（静止时原始读数≈真实零偏），偏离标定基准不超过 LIMIT，防止
 * 误把缓慢真实转动吸收成零偏。只在标定完成后生效；返回 1 表示有更新。 */
int calibration_track_gyro_z(ImuCalibration *c, float raw_gz_dps, float dt)
{
    float next;
    if (c->count < CALIBRATION_SAMPLES) return 0;
    if (!finite_float(raw_gz_dps) || !finite_float(dt) || dt <= 0.0f || dt > MAX_SAMPLE_DT) return 0;
    next = c->bias_dps[2] + (dt / IMU_BIAS_TRACK_TAU_S) * (raw_gz_dps - c->bias_dps[2]);
    c->bias_dps[2] = attitude_clamp(next,
                                    c->bias_z_ref_dps - IMU_BIAS_TRACK_LIMIT_DPS,
                                    c->bias_z_ref_dps + IMU_BIAS_TRACK_LIMIT_DPS);
    return 1;
}
int attitude_init(float q[4], float *accel_norm, const float a[3])
{
    float norm, roll, pitch, cr, sr, cp, sp;
    if (!finite_float(a[0]) || !finite_float(a[1]) || !finite_float(a[2])) return 0;
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(norm) || norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) return 0;
    roll=atan2f(a[1],a[2]);
    pitch=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
    cr=cosf(roll*0.5f); sr=sinf(roll*0.5f);
    cp=cosf(pitch*0.5f); sp=sinf(pitch*0.5f);
    q[0]=cr*cp; q[1]=sr*cp; q[2]=cr*sp; q[3]=-sr*sp;
    *accel_norm=norm;
    return 1;
}
/* R=Rz(yaw)*Ry(pitch)*Rx(roll)，pitch 在 +/-90 度处存在万向锁。 */
void attitude_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    e[0]=atan2f(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y))*RAD_TO_DEG;
    e[1]=asinf(attitude_clamp(2.0f*(w*y-z*x),-1.0f,1.0f))*RAD_TO_DEG;
    e[2]=atan2f(2.0f*(w*z+x*y),1.0f-2.0f*(y*y+z*z))*RAD_TO_DEG;
}
void attitude_cube_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    float r00=1.0f-2.0f*(y*y+z*z),r02=2.0f*(x*z+w*y);
    float r10=2.0f*(x*y+w*z),r11=1.0f-2.0f*(x*x+z*z);
    float r12=2.0f*(y*z-w*x),r20=2.0f*(x*z-w*y);
    float r22=1.0f-2.0f*(x*x+y*y);
    float cx=sqrtf(r10*r10+r11*r11);
    e[0]=atan2f(-r12,cx)*RAD_TO_DEG;
    if (cx>1e-5f) {
        e[1]=atan2f(r02,r22)*RAD_TO_DEG;
        e[2]=atan2f(r10,r11)*RAD_TO_DEG;
    } else {
        e[1]=atan2f(-r20,r00)*RAD_TO_DEG;
        e[2]=0;
    }
}
int mahony6_init(Mahony6 *s, const float a[3])
{
    Mahony6 next={0};
    if (!attitude_init(next.q,&next.accel_norm,a)) return 0;
    *s=next; return 1;
}
/* Mahony：实测重力叉积预测重力，PI 修正角速度；在副本上计算后提交。 */
int mahony6_update(Mahony6 *s, const float g[3], const float a[3], float dt)
{
    Mahony6 next=*s;
    float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    float omega[3],error[3]={0,0,0},n;
    unsigned i;
    if (!finite_float(dt) || dt<=0.0f || dt>MAX_SAMPLE_DT) return 0;
    for (i=0;i<3;++i) if (!finite_float(g[i]) || !finite_float(a[i])) return 0;
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n)) return 0;
    next.accel_norm=n;
    next.accel_used=(n>=ACC_NORM_MIN && n<=ACC_NORM_MAX);
    if (next.accel_used) {
        float ax=a[0]/n,ay=a[1]/n,az=a[2]/n;
        float vx=2.0f*(x*z-w*y),vy=2.0f*(w*x+y*z),vz=w*w-x*x-y*y+z*z;
        error[0]=ay*vz-az*vy;
        error[1]=az*vx-ax*vz;
        error[2]=ax*vy-ay*vx;
    }
    for (i=0;i<3;++i) {
        if (AHRS_KI>0.0f) {
            /* 加速度不可信时冻结积分，仍保留已有积分修正。 */
            if (next.accel_used)
                next.integral[i]=attitude_clamp(next.integral[i]+AHRS_KI*error[i]*dt,
                                                -AHRS_INTEGRAL_LIMIT,AHRS_INTEGRAL_LIMIT);
        } else next.integral[i]=0.0f;
        omega[i]=g[i]+AHRS_KP*error[i]+next.integral[i];
    }
    /* 常值角速度下的轴角精确增量：q_new=q*dq。 */
    {
        float mag=sqrtf(omega[0]*omega[0]+omega[1]*omega[1]+omega[2]*omega[2]);
        float theta=0.5f*mag*dt;
        float dq0,scale,dq1,dq2,dq3;
        if (!finite_float(mag) || !finite_float(theta)) return 0;
        dq0=cosf(theta);
        scale=(mag>1e-9f)?sinf(theta)/mag:0.5f*dt;
        dq1=omega[0]*scale; dq2=omega[1]*scale; dq3=omega[2]*scale;
        next.q[0]=w*dq0-x*dq1-y*dq2-z*dq3;
        next.q[1]=w*dq1+x*dq0+y*dq3-z*dq2;
        next.q[2]=w*dq2-x*dq3+y*dq0+z*dq1;
        next.q[3]=w*dq3+x*dq2-y*dq1+z*dq0;
    }
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!finite_float(n) || n<1e-6f) return 0;
    for (i=0;i<4;++i) next.q[i]/=n;
    *s=next; return 1;
}
void mahony6_euler(const Mahony6 *s,float e[3]) { attitude_euler(s->q,e); }
void mahony6_cube_euler(const Mahony6 *s,float e[3]) { attitude_cube_euler(s->q,e); }

int madgwick6_init(Madgwick6 *s,const float a[3])
{
    Madgwick6 next={0};
    if (!attitude_init(next.q,&next.accel_norm,a)) return 0;
    *s=next; return 1;
}
/* Madgwick：qdot=陀螺项-beta*归一化梯度，再按实际 dt 积分。 */
int madgwick6_update(Madgwick6 *s,const float g[3],const float a[3],float dt)
{
    Madgwick6 next=*s;
    float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    float qdot[4],n;
    unsigned i;
    if (!finite_float(dt)||dt<=0.0f||dt>MAX_SAMPLE_DT||
        !finite_float(MADGWICK_BETA)||MADGWICK_BETA<0.0f) return 0;
    for (i=0;i<3;++i) if (!finite_float(g[i])||!finite_float(a[i])) return 0;
    for (i=0;i<4;++i) if (!finite_float(s->q[i])) return 0;
    n=w*w+x*x+y*y+z*z;
    if (!finite_float(n)||n<1e-12f) return 0;
    qdot[0]=0.5f*(-x*g[0]-y*g[1]-z*g[2]);
    qdot[1]=0.5f*(w*g[0]+y*g[2]-z*g[1]);
    qdot[2]=0.5f*(w*g[1]-x*g[2]+z*g[0]);
    qdot[3]=0.5f*(w*g[2]+x*g[1]-y*g[0]);
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n)) return 0;
    next.accel_norm=n;
    next.accel_used=(n>=ACC_NORM_MIN&&n<=ACC_NORM_MAX);
    if (next.accel_used&&MADGWICK_BETA>0.0f) {
        float ax=a[0]/n,ay=a[1]/n,az=a[2]/n;
        float f0=2.0f*(x*z-w*y)-ax;
        float f1=2.0f*(w*x+y*z)-ay;
        float f2=1.0f-2.0f*(x*x+y*y)-az;
        float step[4]={-2.0f*y*f0+2.0f*x*f1,
                      2.0f*z*f0+2.0f*w*f1-4.0f*x*f2,
                      -2.0f*w*f0+2.0f*z*f1-4.0f*y*f2,
                      2.0f*x*f0+2.0f*y*f1};
        float step_norm=sqrtf(step[0]*step[0]+step[1]*step[1]+step[2]*step[2]+step[3]*step[3]);
        if (!finite_float(step_norm)) return 0;
        /* 重力一致时梯度为零，避免除零。 */
        if (step_norm>1e-6f)
            for (i=0;i<4;++i) qdot[i]-=MADGWICK_BETA*step[i]/step_norm;
    }
    for (i=0;i<4;++i) next.q[i]=s->q[i]+qdot[i]*dt;
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!finite_float(n)||n<1e-6f) return 0;
    for (i=0;i<4;++i) next.q[i]/=n;
    *s=next; return 1;
}
void madgwick6_euler(const Madgwick6 *s,float e[3]) { attitude_euler(s->q,e); }
void madgwick6_cube_euler(const Madgwick6 *s,float e[3]) { attitude_cube_euler(s->q,e); }

/* Quaternion error-state EKF, adapted from the QuaternionEKF design in
 * ../project/code (Wang Hongxi): gravity observation, residual bias estimate,
 * acceleration LPF and innovation gating. Uses a minimal local error instead
 * of four unconstrained quaternion errors. Units: rad/s, g, seconds.
 * The fifth error state is by; bz is deliberately not inferred from gravity.
 */
static void ekf_skew(const float v[3], float m[3][3])
{
    m[0][0]=0; m[0][1]=-v[2]; m[0][2]=v[1];
    m[1][0]=v[2]; m[1][1]=0; m[1][2]=-v[0];
    m[2][0]=-v[1]; m[2][1]=v[0]; m[2][2]=0;
}
static int ekf_rotate(float q[4], const float angle[3])
{
    float n=sqrtf(angle[0]*angle[0]+angle[1]*angle[1]+angle[2]*angle[2]);
    float k=n>1e-8f?sinf(0.5f*n)/n:0.5f;
    float c=cosf(0.5f*n), x=angle[0]*k,y=angle[1]*k,z=angle[2]*k;
    float w=q[0],qx=q[1],qy=q[2],qz=q[3];
    unsigned i;
    q[0]=w*c-qx*x-qy*y-qz*z;
    q[1]=w*x+qx*c+qy*z-qz*y;
    q[2]=w*y-qx*z+qy*c+qz*x;
    q[3]=w*z+qx*y-qy*x+qz*c;
    n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!finite_float(n) || n<1e-6f) return 0;
    for(i=0;i<4;++i) q[i]/=n;
    return 1;
}
/* Carry acceleration history into the current body frame using gyro motion.
 * Rotate by dq^-1: a gravity vector expressed in the rotating sensor frame. */
static int ekf_transport_accel(float v[3], const float angle[3])
{
    float dq[4]={1,0,0,0},x,y,z,w,out[3];
    if(!ekf_rotate(dq,angle)) return 0;
    w=dq[0]; x=dq[1]; y=dq[2]; z=dq[3];
    out[0]=(1-2*(y*y+z*z))*v[0]+2*(x*y+w*z)*v[1]+2*(x*z-w*y)*v[2];
    out[1]=2*(x*y-w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z+w*x)*v[2];
    out[2]=2*(x*z+w*y)*v[0]+2*(y*z-w*x)*v[1]+(1-2*(x*x+y*y))*v[2];
    memcpy(v,out,sizeof(out)); return 1;
}
static float ekf_yaw_rad(const float q[4])
{
    return atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}
/* Gravity does not observe heading. Preserve the gyro-predicted Euler yaw
 * exactly after the finite tilt injection (projected gain is first order). */
static void ekf_preserve_yaw(float q[4], float predicted_yaw)
{
    float d=predicted_yaw-ekf_yaw_rad(q),c=cosf(.5f*d),s=sinf(.5f*d);
    float w=q[0],x=q[1],y=q[2],z=q[3];
    q[0]=c*w-s*z; q[1]=c*x-s*y; q[2]=c*y+s*x; q[3]=c*z+s*w;
}
/* P <- F P F^T; works in-place with fixed workspace. */
static void ekf_cov_transform(float p[5][5], const float f[5][5])
{
    float t[5][5]={{0}},out[5][5]={{0}};
    unsigned i,j,k;
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<5;++k)
        t[i][j]+=f[i][k]*p[k][j];
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<5;++k)
        out[i][j]+=t[i][k]*f[j][k];
    memcpy(p,out,sizeof(out));
}
static int ekf_inverse3(const float s[3][3],float inv[3][3])
{
    float aug[3][6]; unsigned i,j,k;
    for(i=0;i<3;++i) for(j=0;j<6;++j)
        aug[i][j]=j<3?s[i][j]:(j-3==i?1.0f:0.0f);
    for(i=0;i<3;++i) {
        unsigned pivot=i;
        for(j=i+1;j<3;++j) if(fabsf(aug[j][i])>fabsf(aug[pivot][i])) pivot=j;
        if(!finite_float(aug[pivot][i]) || fabsf(aug[pivot][i])<1e-12f) return 0;
        for(k=0;k<6;++k) { float v=aug[i][k]; aug[i][k]=aug[pivot][k]; aug[pivot][k]=v; }
        { float d=aug[i][i]; for(k=0;k<6;++k) aug[i][k]/=d; }
        for(j=0;j<3;++j) if(j!=i) {
            float d=aug[j][i]; for(k=0;k<6;++k) aug[j][k]-=d*aug[i][k];
        }
    }
    for(i=0;i<3;++i) for(j=0;j<3;++j) inv[i][j]=aug[i][j+3];
    return 1;
}
int ekf6_init(Ekf6 *s,const float a[3])
{
    Ekf6 next={0}; unsigned i;
    next.innovation_nis=-1.0f;
    if(!attitude_init(next.q,&next.accel_norm,a)) return 0;
    for(i=0;i<3;++i) { next.accel_lpf[i]=a[i]; next.covariance[i][i]=0.01f; }
    for(i=3;i<5;++i) next.covariance[i][i]=0.0001f;
    *s=next; return 1;
}
int ekf6_update(Ekf6 *s,const float g[3],const float a[3],float dt)
{
    Ekf6 next=*s;
    float f[5][5]={{0}},skew[3][3],angle[3],omega[3],h[3],res[3];
    float hmat[3][5]={{0}},pht[5][3]={{0}},smat[3][3]={{0}},inv[3][3];
    float gain[5][3]={{0}},dx[5]={0},norm,alpha,predicted_yaw,yaw_jac[3];
    unsigned i,j,k;
    if(!finite_float(dt) || dt<=0 || dt>MAX_SAMPLE_DT) return 0;
    for(i=0;i<3;++i) if(!finite_float(g[i]) || !finite_float(a[i])) return 0;
    norm=0;
    for(i=0;i<4;++i) { if(!finite_float(s->q[i])) return 0; norm+=s->q[i]*s->q[i]; }
    if(!finite_float(norm) || norm<1e-12f) return 0;
    for(i=0;i<2;++i) if(!finite_float(s->bias_xy[i])) return 0;
    for(i=0;i<5;++i) for(j=0;j<5;++j) if(!finite_float(s->covariance[i][j])) return 0;
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(!finite_float(norm)) return 0;
    next.accel_norm=norm; next.accel_used=0; next.innovation_nis=-1.0f;
    for(i=0;i<3;++i) { omega[i]=g[i]-(i<2?next.bias_xy[i]:0); angle[i]=omega[i]*dt; }
    if(!ekf_rotate(next.q,angle) || !ekf_transport_accel(next.accel_lpf,angle)) return 0;
    predicted_yaw=ekf_yaw_rad(next.q);
    ekf_skew(omega,skew);
    for(i=0;i<5;++i) f[i][i]=1;
    for(i=0;i<3;++i) for(j=0;j<3;++j) f[i][j]-=skew[i][j]*dt;
    f[0][3]=-dt; f[1][4]=-dt;
    ekf_cov_transform(next.covariance,f);
    for(i=0;i<5;++i) next.covariance[i][i]+=(i<3?EKF_GYRO_NOISE:EKF_BIAS_NOISE)*dt;
    /* Reject abnormal raw acceleration before it contaminates the LPF. */
    if(norm<ACC_NORM_MIN || norm>ACC_NORM_MAX) goto commit;
    alpha=dt/(EKF_ACCEL_LPF_TAU_S+dt);
    for(i=0;i<3;++i) next.accel_lpf[i]+=alpha*(a[i]-next.accel_lpf[i]);
    norm=sqrtf(next.accel_lpf[0]*next.accel_lpf[0]+next.accel_lpf[1]*next.accel_lpf[1]+next.accel_lpf[2]*next.accel_lpf[2]);
    if(norm<1e-6f || !finite_float(norm)) goto commit;
    {
        float w=next.q[0],x=next.q[1],y=next.q[2],z=next.q[3];
        float sinp=2*(w*y-z*x),cp2=fmaxf(0.0f,1-sinp*sinp);
        h[0]=2*(x*z-w*y); h[1]=2*(w*x+y*z); h[2]=w*w-x*x-y*y+z*z;
        /* Euler heading becomes singular near vertical: gyro prediction only. */
        if(cp2<0.01f) goto commit;
        yaw_jac[0]=0; yaw_jac[1]=h[1]/cp2; yaw_jac[2]=h[2]/cp2;
    }
    ekf_skew(h,skew);
    for(i=0;i<3;++i) {
        res[i]=next.accel_lpf[i]/norm-h[i];
        for(j=0;j<3;++j) hmat[i][j]=skew[i][j];
    }
    for(i=0;i<5;++i) for(j=0;j<3;++j) for(k=0;k<5;++k)
        pht[i][j]+=next.covariance[i][k]*hmat[j][k];
    for(i=0;i<3;++i) for(j=0;j<3;++j) {
        for(k=0;k<5;++k) smat[i][j]+=hmat[i][k]*pht[k][j];
        if(i==j) smat[i][j]+=EKF_ACCEL_VARIANCE;
    }
    if(!ekf_inverse3(smat,inv)) return 0;
    next.innovation_nis=0.0f;
    for(i=0;i<3;++i) for(j=0;j<3;++j) next.innovation_nis+=res[i]*inv[i][j]*res[j];
    if(!finite_float(next.innovation_nis)) return 0;
    if(next.innovation_nis>EKF_CHI_SQUARE_LIMIT) goto commit;
    for(i=0;i<5;++i) for(j=0;j<3;++j) for(k=0;k<3;++k) gain[i][j]+=pht[i][k]*inv[k][j];
    /* Project the angle gain onto zero Euler-yaw correction. Do this BEFORE
     * Joseph covariance update; do not pretend a gravity observation reduced
     * uncertainty along the unobserved heading direction. yaw_jac dot h = 1. */
    for(j=0;j<3;++j) {
        float yaw_gain=0;
        for(i=0;i<3;++i) yaw_gain+=yaw_jac[i]*gain[i][j];
        for(i=0;i<3;++i) gain[i][j]-=h[i]*yaw_gain;
    }
    for(i=0;i<5;++i) for(j=0;j<3;++j) dx[i]+=gain[i][j]*res[j];
    /* Joseph form retains symmetry/positive variance with finite precision. */
    memset(f,0,sizeof(f));
    for(i=0;i<5;++i) for(j=0;j<5;++j) {
        f[i][j]=(i==j?1.0f:0.0f);
        for(k=0;k<3;++k) f[i][j]-=gain[i][k]*hmat[k][j];
    }
    ekf_cov_transform(next.covariance,f);
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<3;++k)
        next.covariance[i][j]+=EKF_ACCEL_VARIANCE*gain[i][k]*gain[j][k];
    if(!ekf_rotate(next.q,dx)) return 0;
    ekf_preserve_yaw(next.q,predicted_yaw);
    for(i=0;i<2;++i) next.bias_xy[i]=attitude_clamp(next.bias_xy[i]+dx[i+3],-EKF_BIAS_LIMIT_RADPS,EKF_BIAS_LIMIT_RADPS);
    /* Reset local error coordinates after quaternion injection. */
    ekf_skew(dx,skew); memset(f,0,sizeof(f));
    for(i=0;i<5;++i) f[i][i]=1;
    for(i=0;i<3;++i) for(j=0;j<3;++j) f[i][j]-=0.5f*skew[i][j];
    ekf_cov_transform(next.covariance,f);
    next.accel_used=1;
commit:
    for(i=0;i<5;++i) for(j=i;j<5;++j) {
        float v=0.5f*(next.covariance[i][j]+next.covariance[j][i]);
        if(!finite_float(v) || (i==j && v<0)) return 0;
        next.covariance[i][j]=next.covariance[j][i]=v;
    }
    *s=next; return 1;
}
void ekf6_euler(const Ekf6 *s,float e[3]) { attitude_euler(s->q,e); }
void ekf6_cube_euler(const Ekf6 *s,float e[3]) { attitude_cube_euler(s->q,e); }
