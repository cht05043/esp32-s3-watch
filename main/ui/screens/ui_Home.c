// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Home_screen_init(void)
{
    ui_Home = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Home, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Home, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Home, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Home, &ui_img_bg2_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container1 = lv_obj_create(ui_Home);
    lv_obj_remove_style_all(ui_Container1);
    lv_obj_set_width(ui_Container1, 200);
    lv_obj_set_height(ui_Container1, 200);
    lv_obj_set_align(ui_Container1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Container1, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_MinutePng = lv_img_create(ui_Container1);
    lv_img_set_src(ui_MinutePng, &ui_img_clockwise_min_png);
    lv_obj_set_width(ui_MinutePng, LV_SIZE_CONTENT);   /// 20
    lv_obj_set_height(ui_MinutePng, LV_SIZE_CONTENT);    /// 160
    lv_obj_set_x(ui_MinutePng, 0);
    lv_obj_set_y(ui_MinutePng, -80);
    lv_obj_set_align(ui_MinutePng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_MinutePng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_MinutePng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_pivot(ui_MinutePng, 9, 157);
    lv_img_set_zoom(ui_MinutePng, 128);

    ui_HourPng = lv_img_create(ui_Container1);
    lv_img_set_src(ui_HourPng, &ui_img_clockwise_hour_png);
    lv_obj_set_width(ui_HourPng, LV_SIZE_CONTENT);   /// 18
    lv_obj_set_height(ui_HourPng, LV_SIZE_CONTENT);    /// 98
    lv_obj_set_x(ui_HourPng, 0);
    lv_obj_set_y(ui_HourPng, -50);
    lv_obj_set_align(ui_HourPng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_HourPng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_HourPng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_pivot(ui_HourPng, 9, 98);
    lv_img_set_zoom(ui_HourPng, 128);

    ui_secondPng = lv_img_create(ui_Container1);
    lv_img_set_src(ui_secondPng, &ui_img_clock_sec_white_png);
    lv_obj_set_width(ui_secondPng, LV_SIZE_CONTENT);   /// 4
    lv_obj_set_height(ui_secondPng, LV_SIZE_CONTENT);    /// 116
    lv_obj_set_align(ui_secondPng, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_secondPng, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_secondPng, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_center1 = lv_obj_create(ui_Container1);
    lv_obj_set_width(ui_center1, 10);
    lv_obj_set_height(ui_center1, 10);
    lv_obj_set_align(ui_center1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_center1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_center1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_center1, lv_color_hex(0x4B7715), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_center1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_center1, lv_color_hex(0x1D2123), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_center1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_center = lv_obj_create(ui_Container1);
    lv_obj_set_width(ui_center, 30);
    lv_obj_set_height(ui_center, 30);
    lv_obj_set_align(ui_center, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_center, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_center, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_center, lv_color_hex(0xA0B322), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_center, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_center, 100, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container2 = lv_obj_create(ui_Home);
    lv_obj_remove_style_all(ui_Container2);
    lv_obj_set_width(ui_Container2, 142);
    lv_obj_set_height(ui_Container2, 97);
    lv_obj_set_x(ui_Container2, -30);
    lv_obj_set_y(ui_Container2, 90);
    lv_obj_set_align(ui_Container2, LV_ALIGN_LEFT_MID);
    lv_obj_clear_flag(ui_Container2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_weekday = lv_label_create(ui_Container2);
    lv_obj_set_width(ui_weekday, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_weekday, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_weekday, 0);
    lv_obj_set_y(ui_weekday, 20);
    lv_obj_set_align(ui_weekday, LV_ALIGN_CENTER);
    lv_label_set_text(ui_weekday, "WED");
    lv_obj_set_style_text_color(ui_weekday, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_weekday, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_weekday, &ui_font_Size32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_date = lv_label_create(ui_Container2);
    lv_obj_set_width(ui_date, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_date, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_date, 0);
    lv_obj_set_y(ui_date, -20);
    lv_obj_set_align(ui_date, LV_ALIGN_CENTER);
    lv_label_set_text(ui_date, "05/15");
    lv_obj_set_style_text_color(ui_date, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_date, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_date, &ui_font_Size32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TimeHour = lv_label_create(ui_Home);
    lv_obj_set_width(ui_TimeHour, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TimeHour, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TimeHour, -90);
    lv_obj_set_y(ui_TimeHour, -100);
    lv_obj_set_align(ui_TimeHour, LV_ALIGN_CENTER);
    lv_label_set_text(ui_TimeHour, "15");
    lv_obj_set_style_text_color(ui_TimeHour, lv_color_hex(0xE2FD45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_TimeHour, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_TimeHour, &ui_font_Size45, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TimeMinute = lv_label_create(ui_Home);
    lv_obj_set_width(ui_TimeMinute, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_TimeMinute, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_TimeMinute, 90);
    lv_obj_set_y(ui_TimeMinute, -100);
    lv_obj_set_align(ui_TimeMinute, LV_ALIGN_CENTER);
    lv_label_set_text(ui_TimeMinute, "32");
    lv_obj_set_style_text_color(ui_TimeMinute, lv_color_hex(0xE2FD45), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_TimeMinute, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_TimeMinute, &ui_font_Size45, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Container3 = lv_obj_create(ui_Home);
    lv_obj_remove_style_all(ui_Container3);
    lv_obj_set_width(ui_Container3, 112);
    lv_obj_set_height(ui_Container3, 29);
    lv_obj_set_x(ui_Container3, 120);
    lv_obj_set_y(ui_Container3, 280);
    lv_obj_set_align(ui_Container3, LV_ALIGN_TOP_MID);
    lv_obj_set_flex_flow(ui_Container3, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Container3, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Container3, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel3 = lv_obj_create(ui_Container3);
    lv_obj_set_width(ui_Panel3, 10);
    lv_obj_set_height(ui_Panel3, 10);
    lv_obj_set_x(ui_Panel3, 5);
    lv_obj_set_y(ui_Panel3, 135);
    lv_obj_set_align(ui_Panel3, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel3, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel3, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel4 = lv_obj_create(ui_Container3);
    lv_obj_set_width(ui_Panel4, 10);
    lv_obj_set_height(ui_Panel4, 10);
    lv_obj_set_x(ui_Panel4, 5);
    lv_obj_set_y(ui_Panel4, 135);
    lv_obj_set_align(ui_Panel4, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel4, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel5 = lv_obj_create(ui_Container3);
    lv_obj_set_width(ui_Panel5, 10);
    lv_obj_set_height(ui_Panel5, 10);
    lv_obj_set_x(ui_Panel5, 5);
    lv_obj_set_y(ui_Panel5, 135);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_TOP_MID);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel5, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_powerArc = lv_arc_create(ui_Home);
    lv_obj_set_width(ui_powerArc, 150);
    lv_obj_set_height(ui_powerArc, 150);
    lv_obj_set_x(ui_powerArc, 0);
    lv_obj_set_y(ui_powerArc, -50);
    lv_obj_set_align(ui_powerArc, LV_ALIGN_CENTER);
    lv_arc_set_value(ui_powerArc, 50);
    lv_arc_set_bg_angles(ui_powerArc, 240, 300);
    lv_obj_set_style_arc_color(ui_powerArc, lv_color_hex(0x3F3F49), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_powerArc, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_powerArc, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(ui_powerArc, true, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_powerArc, lv_color_hex(0x2D9A90), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_powerArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_powerArc, 8, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_blend_mode(ui_powerArc, LV_BLEND_MODE_NORMAL, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_powerArc, 40, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_powerImg = lv_img_create(ui_Home);
    lv_img_set_src(ui_powerImg, &ui_img_flash_png);
    lv_obj_set_width(ui_powerImg, LV_SIZE_CONTENT);   /// 13
    lv_obj_set_height(ui_powerImg, LV_SIZE_CONTENT);    /// 18
    lv_obj_set_x(ui_powerImg, 0);
    lv_obj_set_y(ui_powerImg, -100);
    lv_obj_set_align(ui_powerImg, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_powerImg, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_powerImg, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_StepImg = lv_img_create(ui_Home);
    lv_img_set_src(ui_StepImg, &ui_img_step_png);
    lv_obj_set_width(ui_StepImg, LV_SIZE_CONTENT);   /// 19
    lv_obj_set_height(ui_StepImg, LV_SIZE_CONTENT);    /// 15
    lv_obj_set_x(ui_StepImg, 90);
    lv_obj_set_y(ui_StepImg, 90);
    lv_obj_set_align(ui_StepImg, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_StepImg, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_StepImg, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_stepCount = lv_label_create(ui_Home);
    lv_obj_set_width(ui_stepCount, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_stepCount, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_stepCount, 90);
    lv_obj_set_y(ui_stepCount, 70);
    lv_obj_set_align(ui_stepCount, LV_ALIGN_CENTER);
    lv_label_set_text(ui_stepCount, "100");

    lv_obj_add_event_cb(ui_Home, ui_event_Home, LV_EVENT_ALL, NULL);

}
