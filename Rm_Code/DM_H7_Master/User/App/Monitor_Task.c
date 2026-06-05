#include "Monitor_Task.h"
#include "All_Init.h"
#include "ui_interface.h"
#include "cmsis_os.h"
#include "ui_g.h"

void RobotUI_Init(void)
{
   ui_self_id = User_data.robot_status.robot_id;
    // _ui_init_g_group_cap_0();      osDelay(40);
    // _ui_init_g_group_shoot_0();    osDelay(40);
    // _ui_init_g_group_static_0();   osDelay(40);
    // _ui_init_g_group_static_1();   osDelay(40);
    // _ui_init_g_group_status_0();   osDelay(40);
    // _ui_init_g_group_status_1();   osDelay(40);
    // _ui_init_g_group_vision_0();   osDelay(40);
         _ui_init_g_group_cap_0();    
    _ui_init_g_group_shoot_0();   
    _ui_init_g_group_static_0();   
    _ui_init_g_group_static_1(); 
    _ui_init_g_group_status_0();  
    _ui_init_g_group_status_1(); 
    _ui_init_g_group_vision_0();
    
    
}


void RobotUI_Update(void)
{
    ui_self_id = User_data.robot_status.robot_id;
    if (WHW_V_DBUS.KeyBoard.C == 1)
    {
        RobotUI_Init();
    }
       
    _ui_update_g_group_cap_0();      
    _ui_update_g_group_shoot_0();   
    _ui_update_g_group_static_0();   
    _ui_update_g_group_static_1();   
    _ui_update_g_group_status_0();  
    _ui_update_g_group_status_1();  
    _ui_init_g_group_vision_0();  
}