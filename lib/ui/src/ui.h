// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: yk

#ifndef _YK_UI_H
#define _YK_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1(lv_event_t * e);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_Label2;
void ui_event_TextArea1(lv_event_t * e);
extern lv_obj_t * ui_TextArea1;
void ui_event_TextArea2(lv_event_t * e);
extern lv_obj_t * ui_TextArea2;
extern lv_obj_t * ui_Keyboard1;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label3;
// CUSTOM VARIABLES

// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
extern lv_obj_t * ui_Screen2;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Label7;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Label10;
void ui_event_Switch1(lv_event_t * e);
extern lv_obj_t * ui_Switch1;
void ui_event_Switch2(lv_event_t * e);
extern lv_obj_t * ui_Switch2;
void ui_event_Switch3(lv_event_t * e);
extern lv_obj_t * ui_Switch3;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_Label13;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_Label15;
extern lv_obj_t * ui_Label16;
extern lv_obj_t * ui_Label17;
extern lv_obj_t * ui_Label18;
extern lv_obj_t * ui_Label19;
extern lv_obj_t * ui_Button3;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
