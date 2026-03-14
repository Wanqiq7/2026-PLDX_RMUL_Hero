#include "referee_generated_ui.h"

#include "ui_gen/ui.h"

// 该文件只负责将已汇总好的 UI 语义状态映射到生成图元。
// 热量权威值、预算发数与热量门控判断必须在底盘板业务层完成，
// 不允许在渲染层重新推导或显示任何预测/融合热量。

static uint8_t g_cap_value_visible = 1U;
static uint8_t g_dot_fri_on_visible = 1U;
static uint8_t g_dot_fri_off_visible = 1U;
static uint8_t g_dot_fire_allow_visible = 1U;
static uint8_t g_dot_fire_noallow_visible = 1U;
static uint8_t g_dot_autoaim_on_visible = 1U;
static uint8_t g_dot_autoaim_off_visible = 1U;
static uint8_t g_dot_chassis_online_visible = 1U;
static uint8_t g_dot_chassis_offline_visible = 1U;
static uint8_t g_fri_on_visible = 1U;
static uint8_t g_fri_off_visible = 1U;
static uint8_t g_autoaim_on_visible = 1U;
static uint8_t g_autoaim_off_visible = 1U;
static uint8_t g_fire_allow_visible = 1U;
static uint8_t g_fire_noallow_visible = 1U;
static uint8_t g_stuck_visible = 1U;
static uint8_t g_one_bullet_visible = 1U;
static uint8_t g_three_bullet_visible = 1U;
static uint8_t g_burstfire_visible = 1U;
static uint8_t g_chassis_noforce_visible = 1U;
static uint8_t g_chassis_follow_visible = 1U;
static uint8_t g_chassis_nofollow_visible = 1U;
static uint8_t g_chassis_spin_visible = 1U;
static uint8_t g_power_mild_visible = 1U;
static uint8_t g_power_nor_visible = 1U;
static uint8_t g_power_wild_visible = 1U;

static void RefereeGeneratedUISetRoundVisible(ui_interface_round_t *obj,
                                              uint8_t *visible_flag,
                                              uint8_t desired_visible) {
  if (desired_visible) {
    obj->operate_type = (*visible_flag) ? 2U : 1U;
    *visible_flag = 1U;
  } else if (*visible_flag) {
    obj->operate_type = 3U;
    *visible_flag = 0U;
  }
}

static void RefereeGeneratedUISetStringVisible(ui_interface_string_t *obj,
                                               uint8_t *visible_flag,
                                               uint8_t desired_visible) {
  if (desired_visible) {
    obj->operate_type = (*visible_flag) ? 2U : 1U;
    *visible_flag = 1U;
  } else if (*visible_flag) {
    obj->operate_type = 3U;
    *visible_flag = 0U;
  }
}

static void RefereeGeneratedUIUpdateCapValue(
    const Referee_UI_Generated_State_t *ui_state) {
  ui_g_Ungroup_CAPACITY->operate_type = 2U;
  if (ui_state->cap_online) {
    ui_g_Ungroup_CAPCITY->number = (int32_t)ui_state->cap_energy;
    ui_g_Ungroup_CAPCITY->operate_type = g_cap_value_visible ? 2U : 1U;
    g_cap_value_visible = 1U;
  } else {
    ui_g_Ungroup_CAPCITY->operate_type = 3U;
    g_cap_value_visible = 0U;
  }
}

static void RefereeGeneratedUIUpdateHeatRate(
    const Referee_UI_Generated_State_t *ui_state) {
  ui_g_Ungroup_HEAT_TEXT->operate_type = 2U;
  ui_g_Ungroup_RATE_TEXT->operate_type = 2U;
  ui_g_Ungroup_HEAT_NUMBER->number = (int32_t)ui_state->heat_value;
  ui_g_Ungroup_HEAT_NUMBER->operate_type = 2U;
  ui_g_Ungroup_RATE_NUMBER->number = (int32_t)ui_state->fire_allowance_count;
  ui_g_Ungroup_RATE_NUMBER->operate_type = 2U;
}

static void RefereeGeneratedUIUpdateFrictionState(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t fri_on = ui_state->fri_on ? 1U : 0U;
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_FRI_ON,
                                    &g_dot_fri_on_visible, fri_on);
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_FRI_OFF,
                                    &g_dot_fri_off_visible, !fri_on);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_FRI_ON, &g_fri_on_visible,
                                     fri_on);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_FRI_OFF, &g_fri_off_visible,
                                     !fri_on);
}

static void RefereeGeneratedUIUpdateFireAllowState(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t fire_allow = ui_state->fire_allow ? 1U : 0U;
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_FIRE_ALLOW,
                                    &g_dot_fire_allow_visible, fire_allow);
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_FIRE_NOALLOW,
                                    &g_dot_fire_noallow_visible, !fire_allow);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_FIRE_ALLOW,
                                     &g_fire_allow_visible, fire_allow);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_FIRE_NOALLOW,
                                     &g_fire_noallow_visible, !fire_allow);
}

static void RefereeGeneratedUIUpdateAutoaimState(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t autoaim_on = ui_state->autoaim_on ? 1U : 0U;
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_AUTOAIM_ONLINE,
                                    &g_dot_autoaim_on_visible, autoaim_on);
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_AUTOAIM_OFFLINE,
                                    &g_dot_autoaim_off_visible, !autoaim_on);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_AUTOAIM_ON,
                                     &g_autoaim_on_visible, autoaim_on);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_AUTOAIM_OFF,
                                     &g_autoaim_off_visible, !autoaim_on);
}

static void RefereeGeneratedUIUpdateChassisOnlineState(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t chassis_online = ui_state->chassis_online ? 1U : 0U;
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_CHASSIS_ONLINE,
                                    &g_dot_chassis_online_visible,
                                    chassis_online);
  RefereeGeneratedUISetRoundVisible(ui_g_Ungroup_DOT_CHASSIS_OFFLINE,
                                    &g_dot_chassis_offline_visible,
                                    !chassis_online);
}

static void RefereeGeneratedUIUpdateJamState(
    const Referee_UI_Generated_State_t *ui_state) {
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_STUCK, &g_stuck_visible,
                                     ui_state->stuck_active ? 1U : 0U);
}

static void RefereeGeneratedUIUpdateShootMode(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t show_one = (ui_state->shoot_mode_display == LOAD_1_BULLET);
  const uint8_t show_three = (ui_state->shoot_mode_display == LOAD_3_BULLET);
  const uint8_t show_burst = (ui_state->shoot_mode_display == LOAD_BURSTFIRE);

  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_1_BULLET,
                                     &g_one_bullet_visible, show_one);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_3_BULLET,
                                     &g_three_bullet_visible, show_three);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_BURSTFIRE,
                                     &g_burstfire_visible, show_burst);
}

static void RefereeGeneratedUIUpdatePowerMode(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t mild = (ui_state->power_mode == REFEREE_UI_POWER_MILD);
  const uint8_t wild = (ui_state->power_mode == REFEREE_UI_POWER_WILD);
  const uint8_t nor = !mild && !wild;

  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_POWER_MILD,
                                     &g_power_mild_visible, mild);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_POWER_NOR,
                                     &g_power_nor_visible, nor);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_POWER_WILD,
                                     &g_power_wild_visible, wild);
}

static void RefereeGeneratedUIUpdateChassisMode(
    const Referee_UI_Generated_State_t *ui_state) {
  const uint8_t noforce = (ui_state->chassis_mode == CHASSIS_ZERO_FORCE);
  const uint8_t follow =
      (ui_state->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW);
  const uint8_t spin = (ui_state->chassis_mode == CHASSIS_ROTATE);
  const uint8_t nofollow =
      (!noforce && !follow && !spin) ? 1U : 0U;

  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_CHASSIS_NOFORCE,
                                     &g_chassis_noforce_visible, noforce);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_CHASSIS_FOLLOW,
                                     &g_chassis_follow_visible, follow);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_CHASSIS_NOFOLLOW,
                                     &g_chassis_nofollow_visible, nofollow);
  RefereeGeneratedUISetStringVisible(ui_g_Ungroup_CHASSIS_SPIN,
                                     &g_chassis_spin_visible, spin);
}

void RefereeGeneratedUIInit(const referee_id_t *referee_id) {
  if (referee_id != NULL) {
    ui_self_id = referee_id->Robot_ID;
  }
  ui_delete_layer(UI_Data_Del_ALL, 0U);
  g_cap_value_visible = 1U;
  g_dot_fri_on_visible = 1U;
  g_dot_fri_off_visible = 1U;
  g_dot_fire_allow_visible = 1U;
  g_dot_fire_noallow_visible = 1U;
  g_dot_autoaim_on_visible = 1U;
  g_dot_autoaim_off_visible = 1U;
  g_dot_chassis_online_visible = 1U;
  g_dot_chassis_offline_visible = 1U;
  g_fri_on_visible = 1U;
  g_fri_off_visible = 1U;
  g_autoaim_on_visible = 1U;
  g_autoaim_off_visible = 1U;
  g_fire_allow_visible = 1U;
  g_fire_noallow_visible = 1U;
  g_stuck_visible = 1U;
  g_one_bullet_visible = 1U;
  g_three_bullet_visible = 1U;
  g_burstfire_visible = 1U;
  g_chassis_noforce_visible = 1U;
  g_chassis_follow_visible = 1U;
  g_chassis_nofollow_visible = 1U;
  g_chassis_spin_visible = 1U;
  g_power_mild_visible = 1U;
  g_power_nor_visible = 1U;
  g_power_wild_visible = 1U;
  ui_init_g();
}

void RefereeGeneratedUIUpdate(const Referee_UI_Generated_State_t *ui_state) {
  if (ui_state == NULL) {
    return;
  }

  RefereeGeneratedUIUpdateFrictionState(ui_state);
  RefereeGeneratedUIUpdateFireAllowState(ui_state);
  RefereeGeneratedUIUpdateAutoaimState(ui_state);
  RefereeGeneratedUIUpdateChassisOnlineState(ui_state);
  RefereeGeneratedUIUpdateCapValue(ui_state);
  RefereeGeneratedUIUpdateHeatRate(ui_state);
  RefereeGeneratedUIUpdateJamState(ui_state);
  RefereeGeneratedUIUpdateShootMode(ui_state);
  RefereeGeneratedUIUpdatePowerMode(ui_state);
  RefereeGeneratedUIUpdateChassisMode(ui_state);
  ui_update_g();
}
