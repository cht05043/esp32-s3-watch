// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Timer_screen_init(void)
{
    ui_Timer = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Timer, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Timer, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Timer, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Timer, &ui_img_bg3_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container5 = lv_obj_create(ui_Timer);
    lv_obj_remove_style_all(ui_Container5);
    lv_obj_set_width(ui_Container5, 112);
    lv_obj_set_height(ui_Container5, 29);
    lv_obj_set_x(ui_Container5, 120);
    lv_obj_set_y(ui_Container5, 280);
    lv_obj_set_align(ui_Container5, LV_ALIGN_TOP_MID);
    lv_obj_set_flex_flow(ui_Container5, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container5, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container5, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel9 = lv_obj_create(ui_Container5);
    lv_obj_set_width(ui_Panel9, 10);
    lv_obj_set_height(ui_Panel9, 10);
    lv_obj_set_x(ui_Panel9, 5);
    lv_obj_set_y(ui_Panel9, 135);
    lv_obj_set_align(ui_Panel9, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel9, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel9, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel9, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel6 = lv_obj_create(ui_Container5);
    lv_obj_set_width(ui_Panel6, 10);
    lv_obj_set_height(ui_Panel6, 10);
    lv_obj_set_x(ui_Panel6, 5);
    lv_obj_set_y(ui_Panel6, 135);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel6, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel7 = lv_obj_create(ui_Container5);
    lv_obj_set_width(ui_Panel7, 10);
    lv_obj_set_height(ui_Panel7, 10);
    lv_obj_set_x(ui_Panel7, 5);
    lv_obj_set_y(ui_Panel7, 135);
    lv_obj_set_align(ui_Panel7, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel7, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_startbutton = lv_btn_create(ui_Timer);
    lv_obj_set_width(ui_startbutton, 60);
    lv_obj_set_height(ui_startbutton, 30);
    lv_obj_set_x(ui_startbutton, -80);
    lv_obj_set_y(ui_startbutton, 60);
    lv_obj_set_align(ui_startbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_startbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_startbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_startbutton, lv_color_hex(0x8F8F8F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_startbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_startbutton, 120, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label5 = lv_label_create(ui_startbutton);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "START");
    lv_obj_set_style_text_color(ui_Label5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_stopbutton = lv_btn_create(ui_Timer);
    lv_obj_set_width(ui_stopbutton, 60);
    lv_obj_set_height(ui_stopbutton, 30);
    lv_obj_set_x(ui_stopbutton, 0);
    lv_obj_set_y(ui_stopbutton, 60);
    lv_obj_set_align(ui_stopbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_stopbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_stopbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_stopbutton, lv_color_hex(0x8F8F8F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_stopbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_stopbutton, 120, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_stopbutton);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "STOP");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_clearbutton = lv_btn_create(ui_Timer);
    lv_obj_set_width(ui_clearbutton, 60);
    lv_obj_set_height(ui_clearbutton, 30);
    lv_obj_set_x(ui_clearbutton, 80);
    lv_obj_set_y(ui_clearbutton, 60);
    lv_obj_set_align(ui_clearbutton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_clearbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_clearbutton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_clearbutton, lv_color_hex(0x8F8F8F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_clearbutton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_clearbutton, 120, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_clearbutton);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "CLEAR");
    lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timerhour = lv_label_create(ui_Timer);
    lv_obj_set_width(ui_timerhour, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timerhour, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timerhour, -60);
    lv_obj_set_y(ui_timerhour, -60);
    lv_obj_set_align(ui_timerhour, LV_ALIGN_CENTER);
    lv_label_set_text(ui_timerhour, "00");
    lv_obj_set_style_text_font(ui_timerhour, &ui_font_Size45, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timerminute = lv_label_create(ui_Timer);
    lv_obj_set_width(ui_timerminute, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timerminute, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timerminute, 10);
    lv_obj_set_y(ui_timerminute, -60);
    lv_obj_set_align(ui_timerminute, LV_ALIGN_CENTER);
    lv_label_set_text(ui_timerminute, "00");
    lv_obj_set_style_text_font(ui_timerminute, &ui_font_Size45, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timersecond = lv_label_create(ui_Timer);
    lv_obj_set_width(ui_timersecond, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timersecond, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timersecond, 70);
    lv_obj_set_y(ui_timersecond, -60);
    lv_obj_set_align(ui_timersecond, LV_ALIGN_CENTER);
    lv_label_set_text(ui_timersecond, "00");
    lv_obj_set_style_text_font(ui_timersecond, &ui_font_Size32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timermilisec = lv_label_create(ui_Timer);
    lv_obj_set_width(ui_timermilisec, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timermilisec, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_timermilisec, LV_ALIGN_CENTER);
    lv_label_set_text(ui_timermilisec, "00");
    lv_obj_set_style_text_font(ui_timermilisec, &ui_font_Size32, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Timer, ui_event_Timer, LV_EVENT_ALL, NULL);

}
