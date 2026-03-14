//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[13];
extern uint8_t ui_g_dirty_figure[13];
extern ui_interface_string_t ui_g_now_strings[20];
extern uint8_t ui_g_dirty_string[20];

extern uint8_t ui_g_max_send_count[33];

#define ui_g_Ungroup_DOT_FRI_ON ((ui_interface_round_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_DOT_FRI_OFF ((ui_interface_round_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_DOT_CAP_OFFLINE ((ui_interface_round_t*)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_CAPCITY ((ui_interface_number_t*)&(ui_g_now_figures[3]))
#define ui_g_Ungroup_DOT_CAP_ONLINE ((ui_interface_round_t*)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_DOT_FIRE_ALLOW ((ui_interface_round_t*)&(ui_g_now_figures[5]))
#define ui_g_Ungroup_DOT_FIRE_NOALLOW ((ui_interface_round_t*)&(ui_g_now_figures[6]))
#define ui_g_Ungroup_HEAT_NUMBER ((ui_interface_number_t*)&(ui_g_now_figures[7]))
#define ui_g_Ungroup_RATE_NUMBER ((ui_interface_number_t*)&(ui_g_now_figures[8]))
#define ui_g_Ungroup_DOT_AUTOAIM_ONLINE ((ui_interface_round_t*)&(ui_g_now_figures[9]))
#define ui_g_Ungroup_DOT_AUTOAIM_OFFLINE ((ui_interface_round_t*)&(ui_g_now_figures[10]))
#define ui_g_Ungroup_DOT_CHASSIS_ONLINE ((ui_interface_round_t*)&(ui_g_now_figures[11]))
#define ui_g_Ungroup_DOT_CHASSIS_OFFLINE ((ui_interface_round_t*)&(ui_g_now_figures[12]))

#define ui_g_Ungroup_FRI_ON (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_FRI_OFF (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_AUTOAIM_ON (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_AUTOAIM_OFF (&(ui_g_now_strings[3]))
#define ui_g_Ungroup_CAPACITY (&(ui_g_now_strings[4]))
#define ui_g_Ungroup_POWER_MILD (&(ui_g_now_strings[5]))
#define ui_g_Ungroup_POWER_NOR (&(ui_g_now_strings[6]))
#define ui_g_Ungroup_POWER_WILD (&(ui_g_now_strings[7]))
#define ui_g_Ungroup_CHASSIS_NOFORCE (&(ui_g_now_strings[8]))
#define ui_g_Ungroup_FIRE_ALLOW (&(ui_g_now_strings[9]))
#define ui_g_Ungroup_FIRE_NOALLOW (&(ui_g_now_strings[10]))
#define ui_g_Ungroup_HEAT_TEXT (&(ui_g_now_strings[11]))
#define ui_g_Ungroup_RATE_TEXT (&(ui_g_now_strings[12]))
#define ui_g_Ungroup_STUCK (&(ui_g_now_strings[13]))
#define ui_g_Ungroup_1_BULLET (&(ui_g_now_strings[14]))
#define ui_g_Ungroup_3_BULLET (&(ui_g_now_strings[15]))
#define ui_g_Ungroup_BURSTFIRE (&(ui_g_now_strings[16]))
#define ui_g_Ungroup_CHASSIS_FOLLOW (&(ui_g_now_strings[17]))
#define ui_g_Ungroup_CHASSIS_NOFOLLOW (&(ui_g_now_strings[18]))
#define ui_g_Ungroup_CHASSIS_SPIN (&(ui_g_now_strings[19]))

#define ui_g_Ungroup_DOT_FRI_ON_max_send_count (ui_g_max_send_count[0])
#define ui_g_Ungroup_DOT_FRI_OFF_max_send_count (ui_g_max_send_count[1])
#define ui_g_Ungroup_DOT_CAP_OFFLINE_max_send_count (ui_g_max_send_count[2])
#define ui_g_Ungroup_CAPCITY_max_send_count (ui_g_max_send_count[3])
#define ui_g_Ungroup_DOT_CAP_ONLINE_max_send_count (ui_g_max_send_count[4])
#define ui_g_Ungroup_DOT_FIRE_ALLOW_max_send_count (ui_g_max_send_count[5])
#define ui_g_Ungroup_DOT_FIRE_NOALLOW_max_send_count (ui_g_max_send_count[6])
#define ui_g_Ungroup_HEAT_NUMBER_max_send_count (ui_g_max_send_count[7])
#define ui_g_Ungroup_RATE_NUMBER_max_send_count (ui_g_max_send_count[8])
#define ui_g_Ungroup_DOT_AUTOAIM_ONLINE_max_send_count (ui_g_max_send_count[9])
#define ui_g_Ungroup_DOT_AUTOAIM_OFFLINE_max_send_count (ui_g_max_send_count[10])
#define ui_g_Ungroup_DOT_CHASSIS_ONLINE_max_send_count (ui_g_max_send_count[11])
#define ui_g_Ungroup_DOT_CHASSIS_OFFLINE_max_send_count (ui_g_max_send_count[12])

#define ui_g_Ungroup_FRI_ON_max_send_count (ui_g_max_send_count[13])
#define ui_g_Ungroup_FRI_OFF_max_send_count (ui_g_max_send_count[14])
#define ui_g_Ungroup_AUTOAIM_ON_max_send_count (ui_g_max_send_count[15])
#define ui_g_Ungroup_AUTOAIM_OFF_max_send_count (ui_g_max_send_count[16])
#define ui_g_Ungroup_CAPACITY_max_send_count (ui_g_max_send_count[17])
#define ui_g_Ungroup_POWER_MILD_max_send_count (ui_g_max_send_count[18])
#define ui_g_Ungroup_POWER_NOR_max_send_count (ui_g_max_send_count[19])
#define ui_g_Ungroup_POWER_WILD_max_send_count (ui_g_max_send_count[20])
#define ui_g_Ungroup_CHASSIS_NOFORCE_max_send_count (ui_g_max_send_count[21])
#define ui_g_Ungroup_FIRE_ALLOW_max_send_count (ui_g_max_send_count[22])
#define ui_g_Ungroup_FIRE_NOALLOW_max_send_count (ui_g_max_send_count[23])
#define ui_g_Ungroup_HEAT_TEXT_max_send_count (ui_g_max_send_count[24])
#define ui_g_Ungroup_RATE_TEXT_max_send_count (ui_g_max_send_count[25])
#define ui_g_Ungroup_STUCK_max_send_count (ui_g_max_send_count[26])
#define ui_g_Ungroup_1_BULLET_max_send_count (ui_g_max_send_count[27])
#define ui_g_Ungroup_3_BULLET_max_send_count (ui_g_max_send_count[28])
#define ui_g_Ungroup_BURSTFIRE_max_send_count (ui_g_max_send_count[29])
#define ui_g_Ungroup_CHASSIS_FOLLOW_max_send_count (ui_g_max_send_count[30])
#define ui_g_Ungroup_CHASSIS_NOFOLLOW_max_send_count (ui_g_max_send_count[31])
#define ui_g_Ungroup_CHASSIS_SPIN_max_send_count (ui_g_max_send_count[32])

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_DOT_FRI_ON_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_DOT_FRI_OFF_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_DOT_CAP_OFFLINE_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_CAPCITY_dirty (ui_g_dirty_figure[3])
#define ui_g_Ungroup_DOT_CAP_ONLINE_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_DOT_FIRE_ALLOW_dirty (ui_g_dirty_figure[5])
#define ui_g_Ungroup_DOT_FIRE_NOALLOW_dirty (ui_g_dirty_figure[6])
#define ui_g_Ungroup_HEAT_NUMBER_dirty (ui_g_dirty_figure[7])
#define ui_g_Ungroup_RATE_NUMBER_dirty (ui_g_dirty_figure[8])
#define ui_g_Ungroup_DOT_AUTOAIM_ONLINE_dirty (ui_g_dirty_figure[9])
#define ui_g_Ungroup_DOT_AUTOAIM_OFFLINE_dirty (ui_g_dirty_figure[10])
#define ui_g_Ungroup_DOT_CHASSIS_ONLINE_dirty (ui_g_dirty_figure[11])
#define ui_g_Ungroup_DOT_CHASSIS_OFFLINE_dirty (ui_g_dirty_figure[12])

#define ui_g_Ungroup_FRI_ON_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_FRI_OFF_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_AUTOAIM_ON_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_AUTOAIM_OFF_dirty (ui_g_dirty_string[3])
#define ui_g_Ungroup_CAPACITY_dirty (ui_g_dirty_string[4])
#define ui_g_Ungroup_POWER_MILD_dirty (ui_g_dirty_string[5])
#define ui_g_Ungroup_POWER_NOR_dirty (ui_g_dirty_string[6])
#define ui_g_Ungroup_POWER_WILD_dirty (ui_g_dirty_string[7])
#define ui_g_Ungroup_CHASSIS_NOFORCE_dirty (ui_g_dirty_string[8])
#define ui_g_Ungroup_FIRE_ALLOW_dirty (ui_g_dirty_string[9])
#define ui_g_Ungroup_FIRE_NOALLOW_dirty (ui_g_dirty_string[10])
#define ui_g_Ungroup_HEAT_TEXT_dirty (ui_g_dirty_string[11])
#define ui_g_Ungroup_RATE_TEXT_dirty (ui_g_dirty_string[12])
#define ui_g_Ungroup_STUCK_dirty (ui_g_dirty_string[13])
#define ui_g_Ungroup_1_BULLET_dirty (ui_g_dirty_string[14])
#define ui_g_Ungroup_3_BULLET_dirty (ui_g_dirty_string[15])
#define ui_g_Ungroup_BURSTFIRE_dirty (ui_g_dirty_string[16])
#define ui_g_Ungroup_CHASSIS_FOLLOW_dirty (ui_g_dirty_string[17])
#define ui_g_Ungroup_CHASSIS_NOFOLLOW_dirty (ui_g_dirty_string[18])
#define ui_g_Ungroup_CHASSIS_SPIN_dirty (ui_g_dirty_string[19])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
