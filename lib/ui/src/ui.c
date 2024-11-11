// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: yk

#include "ui.h"
#include "ui_helpers.h"

const char* switch1_state = "Close"; // 初始状态
const char* switch2_state = "Close"; // 初始状态
const char* switch3_state = "Close"; // 初始状态

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1(lv_event_t * e);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_Label1;
lv_obj_t * ui_Label2;
void ui_event_TextArea1(lv_event_t * e);
lv_obj_t * ui_TextArea1;
void ui_event_TextArea2(lv_event_t * e);
lv_obj_t * ui_TextArea2;
lv_obj_t * ui_Keyboard1;
void ui_event_Button1(lv_event_t * e);
lv_obj_t * ui_Button1;
lv_obj_t * ui_Label3;
// CUSTOM VARIABLES


// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
lv_obj_t * ui_Screen2;
void ui_event_Button2(lv_event_t * e);
lv_obj_t * ui_Button2;
lv_obj_t * ui_Label4;
lv_obj_t * ui_Label5;
lv_obj_t * ui_Label6;
lv_obj_t * ui_Label7;
lv_obj_t * ui_Label8;
lv_obj_t * ui_Label9;
lv_obj_t * ui_Label10;
void ui_event_Switch1(lv_event_t * e);
lv_obj_t * ui_Switch1;
void ui_event_Switch2(lv_event_t * e);
lv_obj_t * ui_Switch2;
void ui_event_Switch3(lv_event_t * e);
lv_obj_t * ui_Switch3;
lv_obj_t * ui_Label11;
lv_obj_t * ui_Label12;
lv_obj_t * ui_Label13;
lv_obj_t * ui_Label14;
lv_obj_t * ui_Label15;
lv_obj_t * ui_Label16;
lv_obj_t * ui_Label17;
lv_obj_t * ui_Label18;
lv_obj_t * ui_Label19;
lv_obj_t * ui_Button3;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_TextArea1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_keyboard_set_target(ui_Keyboard1,  ui_TextArea1);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_TextArea2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_keyboard_set_target(ui_Keyboard1,  ui_TextArea2);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_Button1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen2_screen_init);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
    }
}

void ui_event_Switch1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_checked_set_text_value(ui_Label17, target, "Open", "Close");
        switch1_state = (strcmp(switch1_state, "Close") == 0) ? "Open" : "Close"; // 更新状态
    }
}

void ui_event_Switch2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_checked_set_text_value(ui_Label18, target, "Open", "Close");
        switch2_state = (strcmp(switch2_state, "Close") == 0) ? "Open" : "Close"; // 更新状态
    }
}

void ui_event_Switch3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_checked_set_text_value(ui_Label19, target, "Open", "Close");
         switch3_state = (strcmp(switch3_state, "Close") == 0) ? "Open" : "Close"; // 更新状态
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
