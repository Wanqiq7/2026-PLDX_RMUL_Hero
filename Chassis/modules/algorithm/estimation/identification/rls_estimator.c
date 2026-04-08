/**
 ******************************************************************************
 * @file rls_estimator.c
 * @brief RLS递归最小二乘辨识实现
 ******************************************************************************
 */
#include "estimation/identification/rls_estimator.h"
#include <math.h>

void RLSInit(RLSInstance *rls, RLS_Init_Config_s *config) {
  rls->lambda = config->lambda;
  rls->delta = config->delta;
  rls->update_cnt = 0U;

  rls->trans_matrix[0] = config->delta;
  rls->trans_matrix[1] = 0.0f;
  rls->trans_matrix[2] = 0.0f;
  rls->trans_matrix[3] = config->delta;

  rls->gain_vector[0] = 0.0f;
  rls->gain_vector[1] = 0.0f;
  rls->params_vector[0] = config->init_k1;
  rls->params_vector[1] = config->init_k2;
}

void RLSUpdate(RLSInstance *rls, float sample_vector[2], float actual_output) {
  float phi[2];
  float p_phi[2];
  float phi_t_p_phi;
  float phi_t_theta;
  float denominator;
  float k[2];
  float error;
  float k_phi_t[4];
  float temp_p[4];

  phi[0] = sample_vector[0];
  phi[1] = sample_vector[1];

  p_phi[0] = rls->trans_matrix[0] * phi[0] + rls->trans_matrix[1] * phi[1];
  p_phi[1] = rls->trans_matrix[2] * phi[0] + rls->trans_matrix[3] * phi[1];

  phi_t_p_phi = phi[0] * p_phi[0] + phi[1] * p_phi[1];
  denominator = rls->lambda + phi_t_p_phi;

  if (fabsf(denominator) < 1e-10f) {
    return;
  }

  k[0] = p_phi[0] / denominator;
  k[1] = p_phi[1] / denominator;

  rls->gain_vector[0] = k[0];
  rls->gain_vector[1] = k[1];

  phi_t_theta = phi[0] * rls->params_vector[0] + phi[1] * rls->params_vector[1];
  error = actual_output - phi_t_theta;

  rls->params_vector[0] += k[0] * error;
  rls->params_vector[1] += k[1] * error;

  if (rls->params_vector[0] < 1e-5f) rls->params_vector[0] = 1e-5f;
  if (rls->params_vector[1] < 1e-5f) rls->params_vector[1] = 1e-5f;

  k_phi_t[0] = k[0] * phi[0];
  k_phi_t[1] = k[0] * phi[1];
  k_phi_t[2] = k[1] * phi[0];
  k_phi_t[3] = k[1] * phi[1];

  temp_p[0] = rls->trans_matrix[0] - k_phi_t[0] * rls->trans_matrix[0] -
              k_phi_t[1] * rls->trans_matrix[2];
  temp_p[1] = rls->trans_matrix[1] - k_phi_t[0] * rls->trans_matrix[1] -
              k_phi_t[1] * rls->trans_matrix[3];
  temp_p[2] = rls->trans_matrix[2] - k_phi_t[2] * rls->trans_matrix[0] -
              k_phi_t[3] * rls->trans_matrix[2];
  temp_p[3] = rls->trans_matrix[3] - k_phi_t[2] * rls->trans_matrix[1] -
              k_phi_t[3] * rls->trans_matrix[3];

  rls->trans_matrix[0] = temp_p[0] / rls->lambda;
  rls->trans_matrix[1] = temp_p[1] / rls->lambda;
  rls->trans_matrix[2] = temp_p[2] / rls->lambda;
  rls->trans_matrix[3] = temp_p[3] / rls->lambda;

  rls->update_cnt++;
}

void RLSReset(RLSInstance *rls) {
  rls->trans_matrix[0] = rls->delta;
  rls->trans_matrix[1] = 0.0f;
  rls->trans_matrix[2] = 0.0f;
  rls->trans_matrix[3] = rls->delta;
  rls->gain_vector[0] = 0.0f;
  rls->gain_vector[1] = 0.0f;
  rls->update_cnt = 0U;
}

void RLSGetParams(RLSInstance *rls, float *k1, float *k2) {
  if (k1 != NULL) *k1 = rls->params_vector[0];
  if (k2 != NULL) *k2 = rls->params_vector[1];
}
