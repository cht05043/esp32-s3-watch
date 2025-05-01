SET(SOURCES screens/ui_Home.c
    screens/ui_Timer.c
    screens/ui_SetTimePage.c
    screens/ui_Wakeup.c
    screens/ui_loading.c
    screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_bg2_png.c
    images/ui_img_clockwise_min_png.c
    images/ui_img_clockwise_hour_png.c
    images/ui_img_clock_sec_white_png.c
    images/ui_img_flash_png.c
    images/ui_img_step_png.c
    images/ui_img_bg3_png.c
    images/ui_img_icn_settings_png.c
    images/ui_img_bg1_png.c
    fonts/ui_font_Size32.c
    fonts/ui_font_Size45.c)

add_library(ui ${SOURCES})
