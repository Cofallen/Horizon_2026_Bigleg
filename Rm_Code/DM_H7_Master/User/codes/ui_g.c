//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"
#include "All_Init.h"

ui_2_frame_t ui_g_group_cap_0;

ui_interface_number_t *ui_g_group_cap_relay = (ui_interface_number_t*)&(ui_g_group_cap_0.data[0]);
ui_interface_arc_t *ui_g_group_cap_volume = (ui_interface_arc_t*)&(ui_g_group_cap_0.data[1]);

void _ui_init_g_group_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group_cap_0.data[i].figure_name[0] = 0;
        ui_g_group_cap_0.data[i].figure_name[1] = 0;
        ui_g_group_cap_0.data[i].figure_name[2] = i + 0;
        ui_g_group_cap_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_group_cap_0.data[i].operate_type = 0;
    }

    ui_g_group_cap_relay->figure_type = 5;
    ui_g_group_cap_relay->operate_type = 1;
    ui_g_group_cap_relay->layer = 0;
    ui_g_group_cap_relay->color = 0;
    ui_g_group_cap_relay->start_x = 452;
    ui_g_group_cap_relay->start_y = 316;
    ui_g_group_cap_relay->width = 2;
    ui_g_group_cap_relay->font_size = 20;
    ui_g_group_cap_relay->number = (int32_t)(IMU_Data.pitch * 1000.0f);

    ui_g_group_cap_volume->figure_type = 4;
    ui_g_group_cap_volume->operate_type = 1;
    ui_g_group_cap_volume->layer = 0;
    ui_g_group_cap_volume->color = 0;
    ui_g_group_cap_volume->start_x = 960;
    ui_g_group_cap_volume->start_y = 520;
    ui_g_group_cap_volume->width = 20;
    ui_g_group_cap_volume->start_angle = 230;
    ui_g_group_cap_volume->end_angle = 270;
    ui_g_group_cap_volume->rx = 380;
    ui_g_group_cap_volume->ry = 400;


    ui_proc_2_frame(&ui_g_group_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_cap_0, sizeof(ui_g_group_cap_0));
}

void _ui_update_g_group_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group_cap_0.data[i].operate_type = 2;
    }
    ui_g_group_cap_relay->number = (int32_t)(IMU_Data.pitch * 1000.0f);
    ui_proc_2_frame(&ui_g_group_cap_0);
        
    SEND_MESSAGE((uint8_t *) &ui_g_group_cap_0, sizeof(ui_g_group_cap_0));
}

void _ui_remove_g_group_cap_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_group_cap_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_group_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_cap_0, sizeof(ui_g_group_cap_0));
}


void ui_init_g_group_cap() {
    _ui_init_g_group_cap_0();
}

void ui_update_g_group_cap() {
    _ui_update_g_group_cap_0();
}

void ui_remove_g_group_cap() {
    _ui_remove_g_group_cap_0();
}

ui_5_frame_t ui_g_group_shoot_0;

ui_interface_arc_t *ui_g_group_shoot_yawArc = (ui_interface_arc_t*)&(ui_g_group_shoot_0.data[0]);
ui_interface_rect_t *ui_g_group_shoot_box = (ui_interface_rect_t*)&(ui_g_group_shoot_0.data[1]);
ui_interface_line_t *ui_g_group_shoot_speed = (ui_interface_line_t*)&(ui_g_group_shoot_0.data[2]);

void _ui_init_g_group_shoot_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_group_shoot_0.data[i].figure_name[0] = 0;
        ui_g_group_shoot_0.data[i].figure_name[1] = 1;
        ui_g_group_shoot_0.data[i].figure_name[2] = i + 0;
        ui_g_group_shoot_0.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_group_shoot_0.data[i].operate_type = 0;
    }

    ui_g_group_shoot_yawArc->figure_type = 4;
    ui_g_group_shoot_yawArc->operate_type = 1;
    ui_g_group_shoot_yawArc->layer = 0;
    ui_g_group_shoot_yawArc->color = 4;
    ui_g_group_shoot_yawArc->start_x = 1585;
    ui_g_group_shoot_yawArc->start_y = 727;
    ui_g_group_shoot_yawArc->width = 20;
    ui_g_group_shoot_yawArc->start_angle = 15;
    ui_g_group_shoot_yawArc->end_angle = 345;
    ui_g_group_shoot_yawArc->rx = 120;
    ui_g_group_shoot_yawArc->ry = 120;

    ui_g_group_shoot_box->figure_type = 1;
    ui_g_group_shoot_box->operate_type = 1;
    ui_g_group_shoot_box->layer = 0;
    ui_g_group_shoot_box->color = 8;
    ui_g_group_shoot_box->start_x = 1563;
    ui_g_group_shoot_box->start_y = 658;
    ui_g_group_shoot_box->width = 1;
    ui_g_group_shoot_box->end_x = 1618;
    ui_g_group_shoot_box->end_y = 823;

    ui_g_group_shoot_speed->figure_type = 0;
    ui_g_group_shoot_speed->operate_type = 1;
    ui_g_group_shoot_speed->layer = 0;
    ui_g_group_shoot_speed->color = 1;
    ui_g_group_shoot_speed->start_x = 1565;
    ui_g_group_shoot_speed->start_y = 661;
    ui_g_group_shoot_speed->width = 100;
    ui_g_group_shoot_speed->end_x = 1620;
    ui_g_group_shoot_speed->end_y = 661;


    ui_proc_5_frame(&ui_g_group_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_shoot_0, sizeof(ui_g_group_shoot_0));
}

void _ui_update_g_group_shoot_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_group_shoot_0.data[i].operate_type = 2;
    }

    float yaw = -IMU_Data.YawTotalAngle;

    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    int start = (int)(yaw + 15.0f);
    int end   = (int)(yaw - 15.0f);

    if (start >= 360) start -= 360;
    if (end < 0) end += 360;

    ui_g_group_shoot_yawArc->start_angle = start;
    ui_g_group_shoot_yawArc->end_angle = end;

    ui_proc_5_frame(&ui_g_group_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_shoot_0, sizeof(ui_g_group_shoot_0));
}

void _ui_remove_g_group_shoot_0() {
    for (int i = 0; i < 3; i++) {
        ui_g_group_shoot_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_group_shoot_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_shoot_0, sizeof(ui_g_group_shoot_0));
}


void ui_init_g_group_shoot() {
    _ui_init_g_group_shoot_0();
}

void ui_update_g_group_shoot() {
    _ui_update_g_group_shoot_0();
}

void ui_remove_g_group_shoot() {
    _ui_remove_g_group_shoot_0();
}

ui_5_frame_t ui_g_group_static_0;

ui_interface_line_t *ui_g_group_static_chassisL = (ui_interface_line_t*)&(ui_g_group_static_0.data[0]);
ui_interface_line_t *ui_g_group_static_chassisR = (ui_interface_line_t*)&(ui_g_group_static_0.data[1]);
ui_interface_line_t *ui_g_group_static_visionP1 = (ui_interface_line_t*)&(ui_g_group_static_0.data[2]);
ui_interface_line_t *ui_g_group_static_visionP2 = (ui_interface_line_t*)&(ui_g_group_static_0.data[3]);
ui_interface_line_t *ui_g_group_static_visionV1 = (ui_interface_line_t*)&(ui_g_group_static_0.data[4]);

void _ui_init_g_group_static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_group_static_0.data[i].figure_name[0] = 0;
        ui_g_group_static_0.data[i].figure_name[1] = 2;
        ui_g_group_static_0.data[i].figure_name[2] = i + 0;
        ui_g_group_static_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_group_static_0.data[i].operate_type = 0;
    }

    ui_g_group_static_chassisL->figure_type = 0;
    ui_g_group_static_chassisL->operate_type = 1;
    ui_g_group_static_chassisL->layer = 0;
    ui_g_group_static_chassisL->color = 0;
    ui_g_group_static_chassisL->start_x = 402;
    ui_g_group_static_chassisL->start_y = 20;
    ui_g_group_static_chassisL->width = 5;
    ui_g_group_static_chassisL->end_x = 702;
    ui_g_group_static_chassisL->end_y = 299;

    ui_g_group_static_chassisR->figure_type = 0;
    ui_g_group_static_chassisR->operate_type = 1;
    ui_g_group_static_chassisR->layer = 0;
    ui_g_group_static_chassisR->color = 0;
    ui_g_group_static_chassisR->start_x = 1500;
    ui_g_group_static_chassisR->start_y = 20;
    ui_g_group_static_chassisR->width = 5;
    ui_g_group_static_chassisR->end_x = 1200;
    ui_g_group_static_chassisR->end_y = 299;

    ui_g_group_static_visionP1->figure_type = 0;
    ui_g_group_static_visionP1->operate_type = 1;
    ui_g_group_static_visionP1->layer = 0;
    ui_g_group_static_visionP1->color = 8;
    ui_g_group_static_visionP1->start_x = 744;
    ui_g_group_static_visionP1->start_y = 477;
    ui_g_group_static_visionP1->width = 1;
    ui_g_group_static_visionP1->end_x = 1180;
    ui_g_group_static_visionP1->end_y = 477;

    ui_g_group_static_visionP2->figure_type = 0;
    ui_g_group_static_visionP2->operate_type = 1;
    ui_g_group_static_visionP2->layer = 0;
    ui_g_group_static_visionP2->color = 8;
    ui_g_group_static_visionP2->start_x = 844;
    ui_g_group_static_visionP2->start_y = 435;
    ui_g_group_static_visionP2->width = 1;
    ui_g_group_static_visionP2->end_x = 1080;
    ui_g_group_static_visionP2->end_y = 435;

    ui_g_group_static_visionV1->figure_type = 0;
    ui_g_group_static_visionV1->operate_type = 1;
    ui_g_group_static_visionV1->layer = 0;
    ui_g_group_static_visionV1->color = 8;
    ui_g_group_static_visionV1->start_x = 960;
    ui_g_group_static_visionV1->start_y = 380;
    ui_g_group_static_visionV1->width = 1;
    ui_g_group_static_visionV1->end_x = 960;
    ui_g_group_static_visionV1->end_y = 520;


    ui_proc_5_frame(&ui_g_group_static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_0, sizeof(ui_g_group_static_0));
}

void _ui_update_g_group_static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_group_static_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_group_static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_0, sizeof(ui_g_group_static_0));
}

void _ui_remove_g_group_static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_group_static_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_group_static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_0, sizeof(ui_g_group_static_0));
}

ui_string_frame_t ui_g_group_static_1;
ui_interface_string_t* ui_g_group_static_NewText = &(ui_g_group_static_1.option);

void _ui_init_g_group_static_1() {
    ui_g_group_static_1.option.figure_name[0] = 0;
    ui_g_group_static_1.option.figure_name[1] = 2;
    ui_g_group_static_1.option.figure_name[2] = 5;
    ui_g_group_static_1.option.operate_type = 1;

    ui_g_group_static_NewText->figure_type = 7;
    ui_g_group_static_NewText->operate_type = 1;
    ui_g_group_static_NewText->layer = 0;
    ui_g_group_static_NewText->color = 0;
    ui_g_group_static_NewText->start_x = 33;
    ui_g_group_static_NewText->start_y = 810;
    ui_g_group_static_NewText->width = 2;
    ui_g_group_static_NewText->font_size = 20;
    ui_g_group_static_NewText->str_length = 6;
    strcpy(ui_g_group_static_NewText->string, "STATUS");


    ui_proc_string_frame(&ui_g_group_static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_1, sizeof(ui_g_group_static_1));
}

void _ui_update_g_group_static_1() {
    ui_g_group_static_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_group_static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_1, sizeof(ui_g_group_static_1));
}

void _ui_remove_g_group_static_1() {
    ui_g_group_static_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_group_static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_static_1, sizeof(ui_g_group_static_1));
}

void ui_init_g_group_static() {
    _ui_init_g_group_static_0();
    _ui_init_g_group_static_1();
}

void ui_update_g_group_static() {
    _ui_update_g_group_static_0();
    _ui_update_g_group_static_1();
}

void ui_remove_g_group_static() {
    _ui_remove_g_group_static_0();
    _ui_remove_g_group_static_1();
}

ui_1_frame_t ui_g_group_status_0;

ui_interface_round_t *ui_g_group_status_NewRound = (ui_interface_round_t*)&(ui_g_group_status_0.data[0]);

void _ui_init_g_group_status_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_status_0.data[i].figure_name[0] = 0;
        ui_g_group_status_0.data[i].figure_name[1] = 3;
        ui_g_group_status_0.data[i].figure_name[2] = i + 0;
        ui_g_group_status_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_group_status_0.data[i].operate_type = 0;
    }

    ui_g_group_status_NewRound->figure_type = 2;
    ui_g_group_status_NewRound->operate_type = 1;
    ui_g_group_status_NewRound->layer = 0;
    ui_g_group_status_NewRound->color = 1;
    ui_g_group_status_NewRound->start_x = 83;
    ui_g_group_status_NewRound->start_y = 721;
    ui_g_group_status_NewRound->width = 20;
    ui_g_group_status_NewRound->r = 32;


    ui_proc_1_frame(&ui_g_group_status_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_0, sizeof(ui_g_group_status_0));
}

void _ui_update_g_group_status_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_status_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_group_status_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_0, sizeof(ui_g_group_status_0));
}

void _ui_remove_g_group_status_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_status_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_group_status_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_0, sizeof(ui_g_group_status_0));
}

ui_string_frame_t ui_g_group_status_1;
ui_interface_string_t* ui_g_group_status_NewText = &(ui_g_group_status_1.option);

void _ui_init_g_group_status_1() {
    ui_g_group_status_1.option.figure_name[0] = 0;
    ui_g_group_status_1.option.figure_name[1] = 3;
    ui_g_group_status_1.option.figure_name[2] = 1;
    ui_g_group_status_1.option.operate_type = 1;

    ui_g_group_status_NewText->figure_type = 7;
    ui_g_group_status_NewText->operate_type = 1;
    ui_g_group_status_NewText->layer = 0;
    ui_g_group_status_NewText->color = 0;
    ui_g_group_status_NewText->start_x = 706;
    ui_g_group_status_NewText->start_y = 785;
    ui_g_group_status_NewText->width = 4;
    ui_g_group_status_NewText->font_size = 40;
    ui_g_group_status_NewText->str_length = 13;
    strcpy(ui_g_group_status_NewText->string, "SEE YOU AGAIN");


    ui_proc_string_frame(&ui_g_group_status_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_1, sizeof(ui_g_group_status_1));
}

void _ui_update_g_group_status_1() {
    ui_g_group_status_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_group_status_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_1, sizeof(ui_g_group_status_1));
}

void _ui_remove_g_group_status_1() {
    ui_g_group_status_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_group_status_1);
    SEND_MESSAGE((uint8_t *) &ui_g_group_status_1, sizeof(ui_g_group_status_1));
}

void ui_init_g_group_status() {
    _ui_init_g_group_status_0();
    _ui_init_g_group_status_1();
}

void ui_update_g_group_status() {
    _ui_update_g_group_status_0();
    _ui_update_g_group_status_1();
}

void ui_remove_g_group_status() {
    _ui_remove_g_group_status_0();
    _ui_remove_g_group_status_1();
}

ui_1_frame_t ui_g_group_vision_0;

ui_interface_rect_t *ui_g_group_vision_NewRect = (ui_interface_rect_t*)&(ui_g_group_vision_0.data[0]);

void _ui_init_g_group_vision_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_vision_0.data[i].figure_name[0] = 0;
        ui_g_group_vision_0.data[i].figure_name[1] = 4;
        ui_g_group_vision_0.data[i].figure_name[2] = i + 0;
        ui_g_group_vision_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_group_vision_0.data[i].operate_type = 0;
    }

    ui_g_group_vision_NewRect->figure_type = 1;
    ui_g_group_vision_NewRect->operate_type = 1;
    ui_g_group_vision_NewRect->layer = 0;
    ui_g_group_vision_NewRect->color = 0;
    ui_g_group_vision_NewRect->start_x = 672;
    ui_g_group_vision_NewRect->start_y = 252;
    ui_g_group_vision_NewRect->width = 1;
    ui_g_group_vision_NewRect->end_x = 1242;
    ui_g_group_vision_NewRect->end_y = 822;


    ui_proc_1_frame(&ui_g_group_vision_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_vision_0, sizeof(ui_g_group_vision_0));
}

void _ui_update_g_group_vision_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_vision_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_group_vision_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_vision_0, sizeof(ui_g_group_vision_0));
}

void _ui_remove_g_group_vision_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_group_vision_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_group_vision_0);
    SEND_MESSAGE((uint8_t *) &ui_g_group_vision_0, sizeof(ui_g_group_vision_0));
}


void ui_init_g_group_vision() {
    _ui_init_g_group_vision_0();
}

void ui_update_g_group_vision() {
    _ui_update_g_group_vision_0();
}

void ui_remove_g_group_vision() {
    _ui_remove_g_group_vision_0();
}

