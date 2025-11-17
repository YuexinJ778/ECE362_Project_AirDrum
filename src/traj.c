#include <math.h>
#include <string.h>
#include "traj.h"

#define ACC_LSB_TO_G         0.000061f     // 与你当前 ±2g 配置一致
#define GYRO_LSB_TO_DPS      0.00875f      // 250 dps
#define G0                   9.80665f

// —— 姿态互补参数（加计定低频，陀螺定高频） ——
#define ORI_ALPHA            0.98f         // slerp 加权（越接近1越信陀螺）
#define YAW_LEAK             0.10f         // yaw 短窗口泄漏（无磁力计）

// —— 积分抗漂 ——
#define V_LEAK               0.05f         // 速度泄漏
#define P_LEAK               0.010f        // 位置泄漏
#define ZVU_ALIN_G           0.15f         // 零速：线加速度阈（g）
#define ZVU_GYRO_DPS         20.0f         // 零速：角速度阈（dps）
#define ZVU_HOLD_SAMPLES     10

#define POS_BUF_SIZE         16            // 圆修正用历史点
#define Z_CLAMP_M            0.80f         // 位置钳位防爆

// —— 几何约束/修正（可调） ——
static float STICK_LENGTH = 0.00f;         // 0 关闭
static float STICK_SOFT_ALPHA = 0.20f;     // 棒长软约束权重
static float PLANE_TOL = 0.01f;            // 平面距离容忍
static float CIRCLE_POS_TOL = 0.01f;       // 半径误差容忍

// —— 向上方向选择（与你原 main 对齐）：up = -ĝ —— 
#define UP_IS_NEG_GHAT  1

// ====== 四元数与旋转 ======
typedef struct { float w,x,y,z; } Quat;

static inline Quat q_norm(Quat q){
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) + 1e-9f;
    q.w/=n; q.x/=n; q.y/=n; q.z/=n; return q;
}
static inline Quat q_mul(Quat a, Quat b){
    Quat c;
    c.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    c.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    c.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    c.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return c;
}
static inline Quat q_from_axis_angle(float vx,float vy,float vz, float ang){
    float ha=0.5f*ang, s=sinf(ha);
    Quat q = { cosf(ha), s*vx, s*vy, s*vz }; return q_norm(q);
}
static inline Quat q_from_rp(float roll, float pitch){  // yaw 独立处理
    float cr=cosf(roll*0.5f), sr=sinf(roll*0.5f);
    float cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
    Quat q = { cr*cp, sr*cp, cr*sp, -sr*sp }; // yaw=0 的 rpy->quat 变体
    return q_norm(q);
}
static inline Vec3 rotate_vector(Quat q, Vec3 v){
    // R(q) * v
    float ww=q.w*q.w, xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    float wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    float xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;
    Vec3 r;
    r.x = (ww+xx-yy-zz)*v.x + 2.0f*(xy-wz)*v.y + 2.0f*(xz+wy)*v.z;
    r.y = 2.0f*(xy+wz)*v.x + (ww-xx+yy-zz)*v.y + 2.0f*(yz-wx)*v.z;
    r.z = 2.0f*(xz-wy)*v.x + 2.0f*(yz+wx)*v.y + (ww-xx-yy+zz)*v.z;
    return r;
}
static inline Quat slerp(Quat a, Quat b, float t){
    float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    if (dot < 0){ b.w=-b.w; b.x=-b.x; b.y=-b.y; b.z=-b.z; dot=-dot; }
    const float DOT_T = 0.9995f;
    if (dot > DOT_T){
        Quat r = { a.w + t*(b.w-a.w), a.x + t*(b.x-a.x), a.y + t*(b.y-a.y), a.z + t*(b.z-a.z) };
        return q_norm(r);
    }
    float th = acosf(dot);
    float s1 = sinf((1.0f-t)*th), s2 = sinf(t*th), s = 1.0f/sinf(th);
    Quat r = { (a.w*s1 + b.w*s2)*s, (a.x*s1 + b.x*s2)*s, (a.y*s1 + b.y*s2)*s, (a.z*s1 + b.z*s2)*s };
    return r;
}

// ====== 状态 ======
static float DT=0.02f;

static Quat  q = {1,0,0,0};     // body->world
static int   q_init = 0;
static float yaw_rel_deg = 0.0f;

static Vec3  pos={0}, vel={0};
static Vec3  a_world_prev={0};
static int   has_prev=0;

static Vec3  pos_buf[POS_BUF_SIZE];
static int   pos_buf_idx=0, pos_buf_count=0;

// ====== 工具 ======
static inline float rad(float d){ return d*0.017453292519943295f; }
static inline float clampf(float x,float a,float b){ return x<a?a:(x>b?b:x); }
static inline float vlen(Vec3 v){ return sqrtf(v.x*v.x+v.y*v.y+v.z*v.z); }
static inline Vec3  vadd(Vec3 a,Vec3 b){ Vec3 r={a.x+b.x,a.y+b.y,a.z+b.z}; return r; }
static inline Vec3  vsub(Vec3 a,Vec3 b){ Vec3 r={a.x-b.x,a.y-b.y,a.z-b.z}; return r; }
static inline Vec3  vscale(Vec3 a,float s){ Vec3 r={a.x*s,a.y*s,a.z*s}; return r; }
static inline float vdot(Vec3 a,Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline Vec3  vcross(Vec3 a,Vec3 b){
    Vec3 r={ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x }; return r;
}
static inline Vec3  vnorm(Vec3 a){ float n=vlen(a)+1e-9f; return vscale(a, 1.0f/n); }

// ====== 圆拟合（在 A,B,C 的平面内做 2D 外接圆，再映回 3D） ======
static int compute_circle_from_3_points(Vec3 A, Vec3 B, Vec3 C, Vec3* center, float* radius){
    Vec3 e1 = vnorm( vsub(B,A) );
    Vec3 n  = vnorm( vcross( vsub(B,A), vsub(C,A) ) );
    if (vlen(n) < 1e-6f) return 0; // 共线/退化
    Vec3 e2 = vcross(n, e1);

    float Ax=0, Ay=0;
    Vec3 AB=vsub(B,A), AC=vsub(C,A);
    float Bx=vdot(AB,e1), By=vdot(AB,e2);
    float Cx=vdot(AC,e1), Cy=vdot(AC,e2);

    float a = Bx - Ax, b = By - Ay;
    float c = Cx - Ax, d = Cy - Ay;
    float e = 0.5f*(Bx*Bx + By*By - Ax*Ax - Ay*Ay);
    float f = 0.5f*(Cx*Cx + Cy*Cy - Ax*Ax - Ay*Ay);
    float det = a*d - b*c;
    if (fabsf(det) < 1e-9f) return 0;

    float uc = ( e*d - b*f )/det;
    float vc = ( a*f - e*c )/det;

    *radius = sqrtf( (uc-Ax)*(uc-Ax) + (vc-Ay)*(vc-Ay) );
    *center = vadd( A, vadd( vscale(e1, uc), vscale(e2, vc) ) );
    return 1;
}

// ====== 棒长软约束 ======
static void apply_stick_length_constraint(Vec3* p){
    if (STICK_LENGTH <= 0.0f) return;
    float d=vlen(*p); if (d<1e-6f) return;
    Vec3 on = vscale(*p, STICK_LENGTH/d);
    p->x = (1.0f-STICK_SOFT_ALPHA)*p->x + STICK_SOFT_ALPHA*on.x;
    p->y = (1.0f-STICK_SOFT_ALPHA)*p->y + STICK_SOFT_ALPHA*on.y;
    p->z = (1.0f-STICK_SOFT_ALPHA)*p->z + STICK_SOFT_ALPHA*on.z;
}

// ====== 圆轨迹修正（近似在同一平面 & 半径稳定时才投影回圆） ======
static void apply_circular_motion_correction_if_valid(Vec3* p){
    if (pos_buf_count < 4) return;
    Vec3 B = pos_buf[(pos_buf_idx-1+POS_BUF_SIZE)%POS_BUF_SIZE];
    Vec3 A = pos_buf[(pos_buf_idx-2+POS_BUF_SIZE)%POS_BUF_SIZE];
    Vec3 C = pos_buf[(pos_buf_idx-3+POS_BUF_SIZE)%POS_BUF_SIZE];

    Vec3 n = vcross( vsub(B,A), vsub(C,A) );
    float nlen = vlen(n);
    if (nlen < 1e-6f) return;
    n = vscale(n, 1.0f/nlen);

    Vec3 center; float radius;
    if (!compute_circle_from_3_points(A,B,C,&center,&radius)) return;

    float dist_plane = vdot( vsub(*p, A), n );
    float dist_center = fabsf( vlen( vsub(*p, center) ) - radius );

    if (fabsf(dist_plane) < PLANE_TOL && dist_center < CIRCLE_POS_TOL){
        Vec3 p_proj = vsub(*p, vscale(n, dist_plane));
        Vec3 dir = vnorm( vsub(p_proj, center) );
        Vec3 on  = vadd(center, vscale(dir, radius));
        *p = on;
    }
}

// ====== 姿态更新（互补：陀螺积分 + 加计倾角 slerp） ======
static Quat quat_from_acc_tilt(Vec3 a_g){
    float roll  = atan2f(a_g.y, a_g.z);
    float pitch = atan2f(-a_g.x, sqrtf(a_g.y*a_g.y + a_g.z*a_g.z));
    return q_from_rp(roll, pitch); // yaw=0
}

static Quat update_orientation(Quat q_prev, Vec3 gyro_dps, Vec3 accel_g, float dt){
    float wx = gyro_dps.x * (float)M_PI/180.0f;
    float wy = gyro_dps.y * (float)M_PI/180.0f;
    float wz = gyro_dps.z * (float)M_PI/180.0f;

    float ang = dt * sqrtf(wx*wx + wy*wy + wz*wz);
    Quat q_gyro = (ang > 1e-9f) ? q_from_axis_angle(wx/ang, wy/ang, wz/ang, ang) : (Quat){1,0,0,0};
    Quat q_prop = q_norm( q_mul(q_prev, q_gyro) );

    Quat q_acc  = quat_from_acc_tilt(accel_g);
    Quat q_new  = slerp(q_acc, q_prop, ORI_ALPHA); // 低频用加计， 高频用陀螺
    return q_new;
}

// ====== 外部接口 ======
void traj_init(float dt_sec){
    DT=dt_sec; pos=(Vec3){0,0,0}; vel=(Vec3){0,0,0};
    q=(Quat){1,0,0,0}; q_init=0; yaw_rel_deg=0.0f;
    has_prev=0; pos_buf_idx=0; pos_buf_count=0;
}
void traj_reset(void){
    pos=(Vec3){0,0,0}; vel=(Vec3){0,0,0};
    yaw_rel_deg=0.0f; has_prev=0;
}

void traj_set_stick_length(float L_m){ STICK_LENGTH = L_m; }
void traj_set_plane_tol(float tol_m){ PLANE_TOL = tol_m; }
void traj_set_circle_tol(float tol_m){ CIRCLE_POS_TOL = tol_m; }

void traj_update(int16_t ax,int16_t ay,int16_t az,
                 int16_t gx,int16_t gy,int16_t gz,
                 const float acc_bias[3], const float gyr_bias[3]){
    float axf=(float)ax-acc_bias[0];
    float ayf=(float)ay-acc_bias[1];
    float azf=(float)az-acc_bias[2];

    Vec3 accel_g = { axf*ACC_LSB_TO_G, ayf*ACC_LSB_TO_G, azf*ACC_LSB_TO_G };
    Vec3 gyro_dps= { ((float)gx-gyr_bias[0])*GYRO_LSB_TO_DPS,
                     ((float)gy-gyr_bias[1])*GYRO_LSB_TO_DPS,
                     ((float)gz-gyr_bias[2])*GYRO_LSB_TO_DPS };

    q = update_orientation(q, gyro_dps, accel_g, DT);
    yaw_rel_deg = (1.0f - YAW_LEAK)*yaw_rel_deg + gyro_dps.z * DT;

    Vec3 a_world = rotate_vector(q, (Vec3){ accel_g.x*G0, accel_g.y*G0, accel_g.z*G0 });

#if UP_IS_NEG_GHAT
    a_world.z -= G0;  // world +Z 为“向上”
#else
    a_world.z += G0;
#endif

    if(!has_prev){ a_world_prev = a_world; has_prev=1; return; }
    Vec3 a_avg = vscale( vadd(a_world, a_world_prev), 0.5f );

    vel.x = (1.0f - V_LEAK)*vel.x + a_avg.x*DT;
    vel.y = (1.0f - V_LEAK)*vel.y + a_avg.y*DT;
    vel.z = (1.0f - V_LEAK)*vel.z + a_avg.z*DT;

    pos.x = (1.0f - P_LEAK)*pos.x + vel.x*DT + a_avg.x*(0.5f*DT*DT);
    pos.y = (1.0f - P_LEAK)*pos.y + vel.y*DT + a_avg.y*(0.5f*DT*DT);
    pos.z = (1.0f - P_LEAK)*pos.z + vel.z*DT + a_avg.z*(0.5f*DT*DT);

    a_world_prev = a_world;

    pos.x = clampf(pos.x, -Z_CLAMP_M, Z_CLAMP_M);
    pos.y = clampf(pos.y, -Z_CLAMP_M, Z_CLAMP_M);
    pos.z = clampf(pos.z, -Z_CLAMP_M, Z_CLAMP_M);

    float alin_g = vlen(a_world)/G0;
    float gyro_abs = fmaxf(fmaxf(fabsf(gyro_dps.x),fabsf(gyro_dps.y)),fabsf(gyro_dps.z));
    if (alin_g < ZVU_ALIN_G && gyro_abs < ZVU_GYRO_DPS){
        static int cnt=0; if(++cnt>=ZVU_HOLD_SAMPLES){
            vel.x=vel.y=vel.z=0.0f;
            pos.x*=(1.0f-8.0f*P_LEAK);
            pos.y*=(1.0f-8.0f*P_LEAK);
            pos.z*=(1.0f-8.0f*P_LEAK);
            yaw_rel_deg*=(1.0f-8.0f*YAW_LEAK);
            cnt=0;
        }
    }

    apply_stick_length_constraint(&pos);
    apply_circular_motion_correction_if_valid(&pos);

    pos_buf[pos_buf_idx] = pos;
    pos_buf_idx = (pos_buf_idx+1)%POS_BUF_SIZE;
    if (pos_buf_count < POS_BUF_SIZE) pos_buf_count++;
}

Vec3 traj_get_pos(void){ return pos; }

int traj_classify_four_zone(float z_up_m, float z_down_m, float r_upper_m){
    if (pos.z >=  z_up_m)  return 1; // CYMBAL
    if (pos.z <= -z_down_m) return 2; // UNDER_TOM
    float rxy = sqrtf(pos.x*pos.x + pos.y*pos.y);
    if (rxy >= r_upper_m)   return 3; // UPPER_TOM（上部/外圈）
    return 0;                          // BASS（中心）
}
