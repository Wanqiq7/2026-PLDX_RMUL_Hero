/**
 ******************************************************************************
 * @file rls_estimator.h
 * @brief RLS递归最小二乘辨识接口定义
 ******************************************************************************
 */
#ifndef RLS_ESTIMATOR_H
#define RLS_ESTIMATOR_H

#include <stdint.h>

typedef struct {
  float lambda;
  float delta;

  float trans_matrix[4];
  float gain_vector[2];
  float params_vector[2];

  uint32_t update_cnt;
} RLSInstance;

typedef struct {
  float lambda;
  float delta;
  float init_k1;
  float init_k2;
} RLS_Init_Config_s;

void RLSInit(RLSInstance *rls, RLS_Init_Config_s *config);
void RLSUpdate(RLSInstance *rls, float sample_vector[2], float actual_output);
void RLSReset(RLSInstance *rls);
void RLSGetParams(RLSInstance *rls, float *k1, float *k2);

#endif // RLS_ESTIMATOR_H
