//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_g_group_cap_relay;
extern ui_interface_arc_t *ui_g_group_cap_volume;

void ui_init_g_group_cap();
void ui_update_g_group_cap();
void ui_remove_g_group_cap();

extern ui_interface_arc_t *ui_g_group_shoot_yawArc;
extern ui_interface_rect_t *ui_g_group_shoot_box;
extern ui_interface_line_t *ui_g_group_shoot_speed;

void ui_init_g_group_shoot();
void ui_update_g_group_shoot();
void ui_remove_g_group_shoot();

extern ui_interface_line_t *ui_g_group_static_chassisL;
extern ui_interface_line_t *ui_g_group_static_chassisR;
extern ui_interface_line_t *ui_g_group_static_visionP1;
extern ui_interface_line_t *ui_g_group_static_visionP2;
extern ui_interface_line_t *ui_g_group_static_visionV1;
extern ui_interface_string_t *ui_g_group_static_NewText;

void ui_init_g_group_static();
void ui_update_g_group_static();
void ui_remove_g_group_static();

extern ui_interface_round_t *ui_g_group_status_NewRound;
extern ui_interface_string_t *ui_g_group_status_NewText;

void ui_init_g_group_status();
void ui_update_g_group_status();
void ui_remove_g_group_status();

extern ui_interface_rect_t *ui_g_group_vision_NewRect;

void ui_init_g_group_vision();
void ui_update_g_group_vision();
void ui_remove_g_group_vision();

void _ui_init_g_group_cap_0(void);
void _ui_update_g_group_cap_0(void);
void _ui_remove_g_group_cap_0(void);

void _ui_init_g_group_shoot_0(void);
void _ui_update_g_group_shoot_0(void);
void _ui_remove_g_group_shoot_0(void);

void _ui_init_g_group_static_0(void);
void _ui_update_g_group_static_0(void);
void _ui_remove_g_group_static_0(void);
void _ui_init_g_group_static_1(void);
void _ui_update_g_group_static_1(void);
void _ui_remove_g_group_static_1(void);

void _ui_init_g_group_status_0(void);
void _ui_update_g_group_status_0(void);
void _ui_remove_g_group_status_0(void);
void _ui_init_g_group_status_1(void);
void _ui_update_g_group_status_1(void);
void _ui_remove_g_group_status_1(void);

void _ui_init_g_group_vision_0(void);
void _ui_update_g_group_vision_0(void);
void _ui_remove_g_group_vision_0(void);

#endif // UI_g_H
