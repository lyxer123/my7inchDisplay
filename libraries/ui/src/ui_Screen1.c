// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: yk

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label1 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -160);
    lv_obj_set_y(ui_Label1, -140);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "zhanghao");
    lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -160);
    lv_obj_set_y(ui_Label2, -40);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "mima");
    lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TextArea1 = lv_textarea_create(ui_Screen1);
    lv_obj_set_width(ui_TextArea1, 250);
    lv_obj_set_height(ui_TextArea1, LV_SIZE_CONTENT);    /// 40
    lv_obj_set_x(ui_TextArea1, 120);
    lv_obj_set_y(ui_TextArea1, -140);
    lv_obj_set_align(ui_TextArea1, LV_ALIGN_CENTER);
    lv_textarea_set_text(ui_TextArea1, "cacac");
    lv_textarea_set_placeholder_text(ui_TextArea1, "Placeholder...");
    lv_textarea_set_one_line(ui_TextArea1, true);
    lv_obj_set_style_text_font(ui_TextArea1, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_TextArea2 = lv_textarea_create(ui_Screen1);
    lv_obj_set_width(ui_TextArea2, 250);
    lv_obj_set_height(ui_TextArea2, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextArea2, 120);
    lv_obj_set_y(ui_TextArea2, -40);
    lv_obj_set_align(ui_TextArea2, LV_ALIGN_CENTER);
    lv_textarea_set_text(ui_TextArea2, "fwfwgwf");
    lv_textarea_set_placeholder_text(ui_TextArea2, "Placeholder...");
    lv_textarea_set_one_line(ui_TextArea2, true);
    lv_obj_set_style_text_font(ui_TextArea2, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_Keyboard1 = lv_keyboard_create(ui_Screen1);
    lv_obj_set_width(ui_Keyboard1, 300);
    lv_obj_set_height(ui_Keyboard1, 120);
    lv_obj_set_x(ui_Keyboard1, -215);
    lv_obj_set_y(ui_Keyboard1, 76);
    lv_obj_set_align(ui_Keyboard1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_Button1 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Button1, 100);
    lv_obj_set_height(ui_Button1, 50);
    lv_obj_set_x(ui_Button1, -2);
    lv_obj_set_y(ui_Button1, 128);
    lv_obj_set_align(ui_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, -353);
    lv_obj_set_y(ui_Label3, -219);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "14");

    lv_obj_add_event_cb(ui_TextArea1, ui_event_TextArea1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_TextArea2, ui_event_TextArea2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Screen1, ui_event_Screen1, LV_EVENT_ALL, NULL);

}
