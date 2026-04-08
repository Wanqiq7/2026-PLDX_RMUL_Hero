#include "controllers/models/heat_gate_model.h"

static float HeatGateClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void HeatGateInit(Heat_Gate_State_s *state, float heat_per_shot) {
  if (state == 0) {
    return;
  }

  state->heat_per_shot = (heat_per_shot > 0.0f) ? heat_per_shot : 0.0f;
  state->predicted_rest_heat = 0.0f;
  state->raw_rest_heat = 0.0f;
  state->barrel_cooling_value = 0.0f;
  state->barrel_heat_limit = 0.0f;
  state->last_update_ms = 0U;
  state->initialized = 0U;
}

void HeatGateReset(Heat_Gate_State_s *state, float raw_rest_heat,
                   float barrel_cooling_value, float barrel_heat_limit,
                   uint32_t now_ms) {
  float clamped_limit;
  float clamped_rest_heat;

  if (state == 0) {
    return;
  }

  clamped_limit = (barrel_heat_limit > 0.0f) ? barrel_heat_limit : 0.0f;
  clamped_rest_heat = HeatGateClampFloat(raw_rest_heat, 0.0f, clamped_limit);

  state->raw_rest_heat = clamped_rest_heat;
  state->predicted_rest_heat = clamped_rest_heat;
  state->barrel_cooling_value =
      (barrel_cooling_value > 0.0f) ? barrel_cooling_value : 0.0f;
  state->barrel_heat_limit = clamped_limit;
  state->last_update_ms = now_ms;
  state->initialized = 1U;
}

float HeatGateComputeRestHeat(uint16_t barrel_heat_limit, uint16_t barrel_heat) {
  if (barrel_heat_limit > barrel_heat) {
    return (float)(barrel_heat_limit - barrel_heat);
  }
  return 0.0f;
}

float HeatGatePredictToTime(Heat_Gate_State_s *state, uint32_t now_ms) {
  uint32_t dt_ms;
  float recovered_heat;

  if (state == 0) {
    return 0.0f;
  }

  if (!state->initialized) {
    state->last_update_ms = now_ms;
    return state->predicted_rest_heat;
  }

  if (now_ms <= state->last_update_ms) {
    return state->predicted_rest_heat;
  }

  dt_ms = now_ms - state->last_update_ms;
  recovered_heat = state->barrel_cooling_value * ((float)dt_ms / 1000.0f);
  state->predicted_rest_heat = HeatGateClampFloat(
      state->predicted_rest_heat + recovered_heat, 0.0f, state->barrel_heat_limit);
  state->last_update_ms = now_ms;

  return state->predicted_rest_heat;
}

void HeatGateUpdateRaw(Heat_Gate_State_s *state, float raw_rest_heat,
                       float barrel_cooling_value, float barrel_heat_limit,
                       uint8_t raw_updated, uint32_t now_ms) {
  float clamped_limit;
  float clamped_rest_heat;

  if (state == 0) {
    return;
  }

  clamped_limit = (barrel_heat_limit > 0.0f) ? barrel_heat_limit : 0.0f;
  clamped_rest_heat = HeatGateClampFloat(raw_rest_heat, 0.0f, clamped_limit);

  if (!state->initialized) {
    HeatGateReset(state, clamped_rest_heat, barrel_cooling_value, clamped_limit,
                  now_ms);
    return;
  }

  HeatGatePredictToTime(state, now_ms);

  state->barrel_cooling_value =
      (barrel_cooling_value > 0.0f) ? barrel_cooling_value : 0.0f;
  state->barrel_heat_limit = clamped_limit;

  if (raw_updated) {
    state->raw_rest_heat = clamped_rest_heat;
    if (state->predicted_rest_heat > clamped_rest_heat) {
      state->predicted_rest_heat = clamped_rest_heat;
    }
  }

  state->predicted_rest_heat = HeatGateClampFloat(state->predicted_rest_heat, 0.0f,
                                                  state->barrel_heat_limit);
}

uint8_t HeatGateTryReserve(Heat_Gate_State_s *state, float required_heat,
                           uint32_t now_ms) {
  if (state == 0) {
    return 0U;
  }

  HeatGatePredictToTime(state, now_ms);
  if (required_heat <= 0.0f) {
    return 1U;
  }

  if (state->predicted_rest_heat + 1e-3f < required_heat) {
    return 0U;
  }

  state->predicted_rest_heat -= required_heat;
  if (state->predicted_rest_heat < 0.0f) {
    state->predicted_rest_heat = 0.0f;
  }
  return 1U;
}

uint8_t HeatGateConsumeContinuous(Heat_Gate_State_s *state,
                                  float heat_per_second, uint32_t now_ms) {
  uint32_t dt_ms;
  float required_heat;

  if (state == 0) {
    return 0U;
  }

  if (!state->initialized) {
    return 0U;
  }

  if (now_ms <= state->last_update_ms) {
    return (state->predicted_rest_heat > 0.0f) ? 1U : 0U;
  }

  dt_ms = now_ms - state->last_update_ms;
  HeatGatePredictToTime(state, now_ms);

  if (heat_per_second <= 0.0f) {
    return (state->predicted_rest_heat > 0.0f) ? 1U : 0U;
  }

  required_heat = heat_per_second * ((float)dt_ms / 1000.0f);
  if (state->predicted_rest_heat + 1e-3f < required_heat) {
    state->predicted_rest_heat = 0.0f;
    return 0U;
  }

  state->predicted_rest_heat -= required_heat;
  if (state->predicted_rest_heat < 0.0f) {
    state->predicted_rest_heat = 0.0f;
  }
  return 1U;
}
