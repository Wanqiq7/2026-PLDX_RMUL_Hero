#ifndef HEAT_GATE_MODEL_H
#define HEAT_GATE_MODEL_H

#include <stdint.h>

typedef struct {
  float heat_per_shot;
  float predicted_rest_heat;
  float raw_rest_heat;
  float barrel_cooling_value;
  float barrel_heat_limit;
  uint32_t last_update_ms;
  uint8_t initialized;
} Heat_Gate_State_s;

void HeatGateInit(Heat_Gate_State_s *state, float heat_per_shot);
void HeatGateReset(Heat_Gate_State_s *state, float raw_rest_heat,
                   float barrel_cooling_value, float barrel_heat_limit,
                   uint32_t now_ms);
float HeatGateComputeRestHeat(uint16_t barrel_heat_limit, uint16_t barrel_heat);
void HeatGateUpdateRaw(Heat_Gate_State_s *state, float raw_rest_heat,
                       float barrel_cooling_value, float barrel_heat_limit,
                       uint8_t raw_updated, uint32_t now_ms);
float HeatGatePredictToTime(Heat_Gate_State_s *state, uint32_t now_ms);
uint8_t HeatGateTryReserve(Heat_Gate_State_s *state, float required_heat,
                           uint32_t now_ms);
uint8_t HeatGateConsumeContinuous(Heat_Gate_State_s *state,
                                  float heat_per_second, uint32_t now_ms);

#endif
