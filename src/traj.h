#pragma once
#include <stdint.h>

typedef struct { float x,y,z; } Vec3;

void  traj_init(float dt_sec);
void  traj_reset(void);

void  traj_update(int16_t ax,int16_t ay,int16_t az,
                  int16_t gx,int16_t gy,int16_t gz,
                  const float acc_bias[3], const float gyr_bias[3]);

Vec3  traj_get_pos(void);

// 分类：0=BASS, 1=CYMBAL, 2=UNDER_TOM, 3=UPPER_TOM
int   traj_classify_four_zone(float z_up_m, float z_down_m, float r_upper_m);

// 调参（可选）
void  traj_set_stick_length(float L_m);     // >0 开棒长软约束；<=0 关闭
void  traj_set_plane_tol(float tol_m);      // 平面距离容忍
void  traj_set_circle_tol(float tol_m);     // 半径误差容忍
