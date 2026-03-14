//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 13
#define TOTAL_STRING 20

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND()                                                        \
  ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings,     \
                   ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
  // Retry key elements several times to reduce persistent missing text after drops.
  ui_g_Ungroup_CAPCITY_max_send_count = 3;
  ui_g_Ungroup_HEAT_NUMBER_max_send_count = 3;
  ui_g_Ungroup_RATE_NUMBER_max_send_count = 3;
  ui_g_Ungroup_FRI_ON_max_send_count = 3;
  ui_g_Ungroup_FRI_OFF_max_send_count = 3;
  ui_g_Ungroup_AUTOAIM_ON_max_send_count = 3;
  ui_g_Ungroup_AUTOAIM_OFF_max_send_count = 3;
  ui_g_Ungroup_CAPACITY_max_send_count = 3;
  ui_g_Ungroup_POWER_MILD_max_send_count = 3;
  ui_g_Ungroup_POWER_NOR_max_send_count = 3;
  ui_g_Ungroup_POWER_WILD_max_send_count = 3;
  ui_g_Ungroup_CHASSIS_NOFORCE_max_send_count = 3;
  ui_g_Ungroup_FIRE_ALLOW_max_send_count = 3;
  ui_g_Ungroup_FIRE_NOALLOW_max_send_count = 3;
  ui_g_Ungroup_HEAT_TEXT_max_send_count = 3;
  ui_g_Ungroup_RATE_TEXT_max_send_count = 3;
  ui_g_Ungroup_CHASSIS_FOLLOW_max_send_count = 3;
  ui_g_Ungroup_CHASSIS_NOFOLLOW_max_send_count = 3;
  ui_g_Ungroup_CHASSIS_SPIN_max_send_count = 3;

  ui_g_Ungroup_DOT_FRI_ON->figure_type = 2;
  ui_g_Ungroup_DOT_FRI_ON->operate_type = 1;
  ui_g_Ungroup_DOT_FRI_ON->layer = 0;
  ui_g_Ungroup_DOT_FRI_ON->color = 5;
  ui_g_Ungroup_DOT_FRI_ON->start_x = 229;
  ui_g_Ungroup_DOT_FRI_ON->start_y = 688;
  ui_g_Ungroup_DOT_FRI_ON->width = 12;
  ui_g_Ungroup_DOT_FRI_ON->r = 6;

  ui_g_Ungroup_DOT_FRI_OFF->figure_type = 2;
  ui_g_Ungroup_DOT_FRI_OFF->operate_type = 1;
  ui_g_Ungroup_DOT_FRI_OFF->layer = 0;
  ui_g_Ungroup_DOT_FRI_OFF->color = 2;
  ui_g_Ungroup_DOT_FRI_OFF->start_x = 229;
  ui_g_Ungroup_DOT_FRI_OFF->start_y = 688;
  ui_g_Ungroup_DOT_FRI_OFF->width = 12;
  ui_g_Ungroup_DOT_FRI_OFF->r = 6;

  ui_g_Ungroup_DOT_CAP_OFFLINE->figure_type = 2;
  ui_g_Ungroup_DOT_CAP_OFFLINE->operate_type = 1;
  ui_g_Ungroup_DOT_CAP_OFFLINE->layer = 0;
  ui_g_Ungroup_DOT_CAP_OFFLINE->color = 5;
  ui_g_Ungroup_DOT_CAP_OFFLINE->start_x = 1346;
  ui_g_Ungroup_DOT_CAP_OFFLINE->start_y = 744;
  ui_g_Ungroup_DOT_CAP_OFFLINE->width = 12;
  ui_g_Ungroup_DOT_CAP_OFFLINE->r = 6;

  ui_g_Ungroup_CAPCITY->figure_type = 6;
  ui_g_Ungroup_CAPCITY->operate_type = 1;
  ui_g_Ungroup_CAPCITY->layer = 0;
  ui_g_Ungroup_CAPCITY->color = 6;
  ui_g_Ungroup_CAPCITY->start_x = 1540;
  ui_g_Ungroup_CAPCITY->start_y = 761;
  ui_g_Ungroup_CAPCITY->width = 2;
  ui_g_Ungroup_CAPCITY->font_size = 20;
  ui_g_Ungroup_CAPCITY->number = 12345;

  ui_g_Ungroup_DOT_CAP_ONLINE->figure_type = 2;
  ui_g_Ungroup_DOT_CAP_ONLINE->operate_type = 1;
  ui_g_Ungroup_DOT_CAP_ONLINE->layer = 0;
  ui_g_Ungroup_DOT_CAP_ONLINE->color = 2;
  ui_g_Ungroup_DOT_CAP_ONLINE->start_x = 1346;
  ui_g_Ungroup_DOT_CAP_ONLINE->start_y = 744;
  ui_g_Ungroup_DOT_CAP_ONLINE->width = 12;
  ui_g_Ungroup_DOT_CAP_ONLINE->r = 6;

  ui_g_Ungroup_DOT_FIRE_ALLOW->figure_type = 2;
  ui_g_Ungroup_DOT_FIRE_ALLOW->operate_type = 1;
  ui_g_Ungroup_DOT_FIRE_ALLOW->layer = 0;
  ui_g_Ungroup_DOT_FIRE_ALLOW->color = 2;
  ui_g_Ungroup_DOT_FIRE_ALLOW->start_x = 229;
  ui_g_Ungroup_DOT_FIRE_ALLOW->start_y = 569;
  ui_g_Ungroup_DOT_FIRE_ALLOW->width = 12;
  ui_g_Ungroup_DOT_FIRE_ALLOW->r = 6;

  ui_g_Ungroup_DOT_FIRE_NOALLOW->figure_type = 2;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->operate_type = 1;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->layer = 0;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->color = 5;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->start_x = 229;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->start_y = 569;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->width = 12;
  ui_g_Ungroup_DOT_FIRE_NOALLOW->r = 6;

  ui_g_Ungroup_HEAT_NUMBER->figure_type = 6;
  ui_g_Ungroup_HEAT_NUMBER->operate_type = 1;
  ui_g_Ungroup_HEAT_NUMBER->layer = 0;
  ui_g_Ungroup_HEAT_NUMBER->color = 8;
  ui_g_Ungroup_HEAT_NUMBER->start_x = 666;
  ui_g_Ungroup_HEAT_NUMBER->start_y = 572;
  ui_g_Ungroup_HEAT_NUMBER->width = 2;
  ui_g_Ungroup_HEAT_NUMBER->font_size = 20;
  ui_g_Ungroup_HEAT_NUMBER->number = 12345;

  ui_g_Ungroup_RATE_NUMBER->figure_type = 6;
  ui_g_Ungroup_RATE_NUMBER->operate_type = 1;
  ui_g_Ungroup_RATE_NUMBER->layer = 0;
  ui_g_Ungroup_RATE_NUMBER->color = 8;
  ui_g_Ungroup_RATE_NUMBER->start_x = 666;
  ui_g_Ungroup_RATE_NUMBER->start_y = 542;
  ui_g_Ungroup_RATE_NUMBER->width = 2;
  ui_g_Ungroup_RATE_NUMBER->font_size = 20;
  ui_g_Ungroup_RATE_NUMBER->number = 12345;

  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->figure_type = 2;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->operate_type = 1;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->layer = 0;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->color = 2;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->start_x = 229;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->start_y = 608;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->width = 12;
  ui_g_Ungroup_DOT_AUTOAIM_ONLINE->r = 6;

  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->figure_type = 2;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->operate_type = 1;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->layer = 0;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->color = 5;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->start_x = 229;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->start_y = 608;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->width = 12;
  ui_g_Ungroup_DOT_AUTOAIM_OFFLINE->r = 6;

  ui_g_Ungroup_DOT_CHASSIS_ONLINE->figure_type = 2;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->operate_type = 1;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->layer = 0;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->color = 5;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->start_x = 229;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->start_y = 649;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->width = 12;
  ui_g_Ungroup_DOT_CHASSIS_ONLINE->r = 6;

  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->figure_type = 2;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->operate_type = 1;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->layer = 0;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->color = 5;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->start_x = 229;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->start_y = 647;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->width = 12;
  ui_g_Ungroup_DOT_CHASSIS_OFFLINE->r = 6;

  ui_g_Ungroup_FRI_ON->figure_type = 7;
  ui_g_Ungroup_FRI_ON->operate_type = 1;
  ui_g_Ungroup_FRI_ON->layer = 0;
  ui_g_Ungroup_FRI_ON->color = 2;
  ui_g_Ungroup_FRI_ON->start_x = 252;
  ui_g_Ungroup_FRI_ON->start_y = 705;
  ui_g_Ungroup_FRI_ON->width = 2;
  ui_g_Ungroup_FRI_ON->font_size = 20;
  ui_g_Ungroup_FRI_ON->str_length = 6;
  strcpy(ui_g_Ungroup_FRI_ON->string, "FRI_ON");

  ui_g_Ungroup_FRI_OFF->figure_type = 7;
  ui_g_Ungroup_FRI_OFF->operate_type = 1;
  ui_g_Ungroup_FRI_OFF->layer = 0;
  ui_g_Ungroup_FRI_OFF->color = 2;
  ui_g_Ungroup_FRI_OFF->start_x = 252;
  ui_g_Ungroup_FRI_OFF->start_y = 705;
  ui_g_Ungroup_FRI_OFF->width = 2;
  ui_g_Ungroup_FRI_OFF->font_size = 20;
  ui_g_Ungroup_FRI_OFF->str_length = 7;
  strcpy(ui_g_Ungroup_FRI_OFF->string, "FRI_OFF");

  ui_g_Ungroup_AUTOAIM_ON->figure_type = 7;
  ui_g_Ungroup_AUTOAIM_ON->operate_type = 1;
  ui_g_Ungroup_AUTOAIM_ON->layer = 0;
  ui_g_Ungroup_AUTOAIM_ON->color = 3;
  ui_g_Ungroup_AUTOAIM_ON->start_x = 255;
  ui_g_Ungroup_AUTOAIM_ON->start_y = 625;
  ui_g_Ungroup_AUTOAIM_ON->width = 2;
  ui_g_Ungroup_AUTOAIM_ON->font_size = 20;
  ui_g_Ungroup_AUTOAIM_ON->str_length = 11;
  strcpy(ui_g_Ungroup_AUTOAIM_ON->string, "AUTOAIM: ON");

  ui_g_Ungroup_AUTOAIM_OFF->figure_type = 7;
  ui_g_Ungroup_AUTOAIM_OFF->operate_type = 1;
  ui_g_Ungroup_AUTOAIM_OFF->layer = 0;
  ui_g_Ungroup_AUTOAIM_OFF->color = 3;
  ui_g_Ungroup_AUTOAIM_OFF->start_x = 255;
  ui_g_Ungroup_AUTOAIM_OFF->start_y = 625;
  ui_g_Ungroup_AUTOAIM_OFF->width = 2;
  ui_g_Ungroup_AUTOAIM_OFF->font_size = 20;
  ui_g_Ungroup_AUTOAIM_OFF->str_length = 12;
  strcpy(ui_g_Ungroup_AUTOAIM_OFF->string, "AUTOAIM: OFF");

  ui_g_Ungroup_CAPACITY->figure_type = 7;
  ui_g_Ungroup_CAPACITY->operate_type = 1;
  ui_g_Ungroup_CAPACITY->layer = 0;
  ui_g_Ungroup_CAPACITY->color = 4;
  ui_g_Ungroup_CAPACITY->start_x = 1368;
  ui_g_Ungroup_CAPACITY->start_y = 761;
  ui_g_Ungroup_CAPACITY->width = 2;
  ui_g_Ungroup_CAPACITY->font_size = 20;
  ui_g_Ungroup_CAPACITY->str_length = 9;
  strcpy(ui_g_Ungroup_CAPACITY->string, "CAPACITY:");

  ui_g_Ungroup_POWER_MILD->figure_type = 7;
  ui_g_Ungroup_POWER_MILD->operate_type = 1;
  ui_g_Ungroup_POWER_MILD->layer = 0;
  ui_g_Ungroup_POWER_MILD->color = 8;
  ui_g_Ungroup_POWER_MILD->start_x = 1371;
  ui_g_Ungroup_POWER_MILD->start_y = 725;
  ui_g_Ungroup_POWER_MILD->width = 2;
  ui_g_Ungroup_POWER_MILD->font_size = 20;
  ui_g_Ungroup_POWER_MILD->str_length = 11;
  strcpy(ui_g_Ungroup_POWER_MILD->string, "POWER: MILD");

  ui_g_Ungroup_POWER_NOR->figure_type = 7;
  ui_g_Ungroup_POWER_NOR->operate_type = 1;
  ui_g_Ungroup_POWER_NOR->layer = 0;
  ui_g_Ungroup_POWER_NOR->color = 8;
  ui_g_Ungroup_POWER_NOR->start_x = 1369;
  ui_g_Ungroup_POWER_NOR->start_y = 725;
  ui_g_Ungroup_POWER_NOR->width = 2;
  ui_g_Ungroup_POWER_NOR->font_size = 20;
  ui_g_Ungroup_POWER_NOR->str_length = 10;
  strcpy(ui_g_Ungroup_POWER_NOR->string, "POWER: NOR");

  ui_g_Ungroup_POWER_WILD->figure_type = 7;
  ui_g_Ungroup_POWER_WILD->operate_type = 1;
  ui_g_Ungroup_POWER_WILD->layer = 0;
  ui_g_Ungroup_POWER_WILD->color = 8;
  ui_g_Ungroup_POWER_WILD->start_x = 1371;
  ui_g_Ungroup_POWER_WILD->start_y = 725;
  ui_g_Ungroup_POWER_WILD->width = 2;
  ui_g_Ungroup_POWER_WILD->font_size = 20;
  ui_g_Ungroup_POWER_WILD->str_length = 11;
  strcpy(ui_g_Ungroup_POWER_WILD->string, "POWER: WILD");

  ui_g_Ungroup_CHASSIS_NOFORCE->figure_type = 7;
  ui_g_Ungroup_CHASSIS_NOFORCE->operate_type = 1;
  ui_g_Ungroup_CHASSIS_NOFORCE->layer = 0;
  ui_g_Ungroup_CHASSIS_NOFORCE->color = 1;
  ui_g_Ungroup_CHASSIS_NOFORCE->start_x = 255;
  ui_g_Ungroup_CHASSIS_NOFORCE->start_y = 665;
  ui_g_Ungroup_CHASSIS_NOFORCE->width = 2;
  ui_g_Ungroup_CHASSIS_NOFORCE->font_size = 20;
  ui_g_Ungroup_CHASSIS_NOFORCE->str_length = 15;
  strcpy(ui_g_Ungroup_CHASSIS_NOFORCE->string, "CHASSIS_NOFORCE");

  ui_g_Ungroup_FIRE_ALLOW->figure_type = 7;
  ui_g_Ungroup_FIRE_ALLOW->operate_type = 1;
  ui_g_Ungroup_FIRE_ALLOW->layer = 0;
  ui_g_Ungroup_FIRE_ALLOW->color = 6;
  ui_g_Ungroup_FIRE_ALLOW->start_x = 253;
  ui_g_Ungroup_FIRE_ALLOW->start_y = 585;
  ui_g_Ungroup_FIRE_ALLOW->width = 2;
  ui_g_Ungroup_FIRE_ALLOW->font_size = 20;
  ui_g_Ungroup_FIRE_ALLOW->str_length = 10;
  strcpy(ui_g_Ungroup_FIRE_ALLOW->string, "FIRE_ALLOW");

  ui_g_Ungroup_FIRE_NOALLOW->figure_type = 7;
  ui_g_Ungroup_FIRE_NOALLOW->operate_type = 1;
  ui_g_Ungroup_FIRE_NOALLOW->layer = 0;
  ui_g_Ungroup_FIRE_NOALLOW->color = 6;
  ui_g_Ungroup_FIRE_NOALLOW->start_x = 253;
  ui_g_Ungroup_FIRE_NOALLOW->start_y = 585;
  ui_g_Ungroup_FIRE_NOALLOW->width = 2;
  ui_g_Ungroup_FIRE_NOALLOW->font_size = 20;
  ui_g_Ungroup_FIRE_NOALLOW->str_length = 12;
  strcpy(ui_g_Ungroup_FIRE_NOALLOW->string, "FIRE_NOALLOW");

  ui_g_Ungroup_HEAT_TEXT->figure_type = 7;
  ui_g_Ungroup_HEAT_TEXT->operate_type = 1;
  ui_g_Ungroup_HEAT_TEXT->layer = 0;
  ui_g_Ungroup_HEAT_TEXT->color = 0;
  ui_g_Ungroup_HEAT_TEXT->start_x = 572;
  ui_g_Ungroup_HEAT_TEXT->start_y = 570;
  ui_g_Ungroup_HEAT_TEXT->width = 2;
  ui_g_Ungroup_HEAT_TEXT->font_size = 20;
  ui_g_Ungroup_HEAT_TEXT->str_length = 5;
  strcpy(ui_g_Ungroup_HEAT_TEXT->string, "HEAT:");

  ui_g_Ungroup_RATE_TEXT->figure_type = 7;
  ui_g_Ungroup_RATE_TEXT->operate_type = 1;
  ui_g_Ungroup_RATE_TEXT->layer = 0;
  ui_g_Ungroup_RATE_TEXT->color = 0;
  ui_g_Ungroup_RATE_TEXT->start_x = 572;
  ui_g_Ungroup_RATE_TEXT->start_y = 543;
  ui_g_Ungroup_RATE_TEXT->width = 2;
  ui_g_Ungroup_RATE_TEXT->font_size = 20;
  ui_g_Ungroup_RATE_TEXT->str_length = 5;
  strcpy(ui_g_Ungroup_RATE_TEXT->string, "RATE:");

  ui_g_Ungroup_STUCK->figure_type = 7;
  ui_g_Ungroup_STUCK->operate_type = 1;
  ui_g_Ungroup_STUCK->layer = 0;
  ui_g_Ungroup_STUCK->color = 0;
  ui_g_Ungroup_STUCK->start_x = 876;
  ui_g_Ungroup_STUCK->start_y = 448;
  ui_g_Ungroup_STUCK->width = 4;
  ui_g_Ungroup_STUCK->font_size = 35;
  ui_g_Ungroup_STUCK->str_length = 5;
  strcpy(ui_g_Ungroup_STUCK->string, "STUCK");

  ui_g_Ungroup_1_BULLET->figure_type = 7;
  ui_g_Ungroup_1_BULLET->operate_type = 1;
  ui_g_Ungroup_1_BULLET->layer = 0;
  ui_g_Ungroup_1_BULLET->color = 0;
  ui_g_Ungroup_1_BULLET->start_x = 571;
  ui_g_Ungroup_1_BULLET->start_y = 518;
  ui_g_Ungroup_1_BULLET->width = 2;
  ui_g_Ungroup_1_BULLET->font_size = 20;
  ui_g_Ungroup_1_BULLET->str_length = 8;
  strcpy(ui_g_Ungroup_1_BULLET->string, "1_BULLET");

  ui_g_Ungroup_3_BULLET->figure_type = 7;
  ui_g_Ungroup_3_BULLET->operate_type = 1;
  ui_g_Ungroup_3_BULLET->layer = 0;
  ui_g_Ungroup_3_BULLET->color = 0;
  ui_g_Ungroup_3_BULLET->start_x = 571;
  ui_g_Ungroup_3_BULLET->start_y = 518;
  ui_g_Ungroup_3_BULLET->width = 2;
  ui_g_Ungroup_3_BULLET->font_size = 20;
  ui_g_Ungroup_3_BULLET->str_length = 8;
  strcpy(ui_g_Ungroup_3_BULLET->string, "3_BULLET");

  ui_g_Ungroup_BURSTFIRE->figure_type = 7;
  ui_g_Ungroup_BURSTFIRE->operate_type = 1;
  ui_g_Ungroup_BURSTFIRE->layer = 0;
  ui_g_Ungroup_BURSTFIRE->color = 0;
  ui_g_Ungroup_BURSTFIRE->start_x = 571;
  ui_g_Ungroup_BURSTFIRE->start_y = 518;
  ui_g_Ungroup_BURSTFIRE->width = 2;
  ui_g_Ungroup_BURSTFIRE->font_size = 20;
  ui_g_Ungroup_BURSTFIRE->str_length = 9;
  strcpy(ui_g_Ungroup_BURSTFIRE->string, "BURSTFIRE");

  ui_g_Ungroup_CHASSIS_FOLLOW->figure_type = 7;
  ui_g_Ungroup_CHASSIS_FOLLOW->operate_type = 1;
  ui_g_Ungroup_CHASSIS_FOLLOW->layer = 0;
  ui_g_Ungroup_CHASSIS_FOLLOW->color = 1;
  ui_g_Ungroup_CHASSIS_FOLLOW->start_x = 255;
  ui_g_Ungroup_CHASSIS_FOLLOW->start_y = 665;
  ui_g_Ungroup_CHASSIS_FOLLOW->width = 2;
  ui_g_Ungroup_CHASSIS_FOLLOW->font_size = 20;
  ui_g_Ungroup_CHASSIS_FOLLOW->str_length = 14;
  strcpy(ui_g_Ungroup_CHASSIS_FOLLOW->string, "CHASSIS_FOLLOW");

  ui_g_Ungroup_CHASSIS_NOFOLLOW->figure_type = 7;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->operate_type = 1;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->layer = 0;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->color = 1;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->start_x = 255;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->start_y = 665;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->width = 2;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->font_size = 20;
  ui_g_Ungroup_CHASSIS_NOFOLLOW->str_length = 16;
  strcpy(ui_g_Ungroup_CHASSIS_NOFOLLOW->string, "CHASSIS_NOFOLLOW");

  ui_g_Ungroup_CHASSIS_SPIN->figure_type = 7;
  ui_g_Ungroup_CHASSIS_SPIN->operate_type = 1;
  ui_g_Ungroup_CHASSIS_SPIN->layer = 0;
  ui_g_Ungroup_CHASSIS_SPIN->color = 1;
  ui_g_Ungroup_CHASSIS_SPIN->start_x = 255;
  ui_g_Ungroup_CHASSIS_SPIN->start_y = 665;
  ui_g_Ungroup_CHASSIS_SPIN->width = 2;
  ui_g_Ungroup_CHASSIS_SPIN->font_size = 20;
  ui_g_Ungroup_CHASSIS_SPIN->str_length = 12;
  strcpy(ui_g_Ungroup_CHASSIS_SPIN->string, "CHASSIS_SPIN");

  uint32_t idx = 0;
  for (int i = 0; i < TOTAL_FIGURE; i++) {
    ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
    ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
    ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
    ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
    ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
    ui_g_dirty_figure[i] = ui_g_max_send_count[i];
    idx++;
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
    ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
    ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
    ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
    ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
    ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
    idx++;
  }

  SCAN_AND_SEND();

  for (int i = 0; i < TOTAL_FIGURE; i++) {
    ui_g_now_figures[i].operate_type = 2;
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    ui_g_now_strings[i].operate_type = 2;
  }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
  for (int i = 0; i < TOTAL_FIGURE; i++) {
    if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i],
               sizeof(ui_g_now_figures[i])) != 0) {
      ui_g_dirty_figure[i] = ui_g_max_send_count[i];
      ui_g_last_figures[i] = ui_g_now_figures[i];
    }
  }
  for (int i = 0; i < TOTAL_STRING; i++) {
    if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i],
               sizeof(ui_g_now_strings[i])) != 0) {
      ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
      ui_g_last_strings[i] = ui_g_now_strings[i];
    }
  }
#endif
  SCAN_AND_SEND();
}
