/*
 * Screen_Main.c
 *
 *  Created on: 3 de mar de 2020
 *      Author: Rinaldo Dos Santos
 *      Sinteck Next
 */
#include "main.h"
#include "usbd_cdc_if.h"
#include "lvgl/lvgl.h"
#include "SInteck/GUI/Screen_Main.h"

extern TIM_HandleTypeDef htim3;

uint32_t TelaAtiva;
extern float tensao, corrente, potencia, setVolts, setAMP;

static lv_obj_t * Tela_Principal;
static lv_task_t * Task_Principal;
static lv_style_t style_fundo;
static lv_obj_t * Set;
static lv_obj_t * Tensao;
static lv_obj_t * Corrente;
static lv_obj_t * Potencia;
static lv_obj_t * slider;
static lv_obj_t * slider_label;

void GUI_PowerSupply(void)
{
	// Create a Screen
	Tela_Principal = lv_obj_create(NULL, NULL);

	lv_style_copy(&style_fundo, &lv_style_plain_color);
	style_fundo.body.main_color = LV_COLOR_BLACK;
	style_fundo.body.grad_color = LV_COLOR_BLACK;
	lv_obj_set_style(Tela_Principal, &style_fundo);				// Configura o estilo criado
	//
	print_headers();
	print_set_prog();
	create_buttons();
	print_values();
	create_slider();
	create_switch();

	// Task Update Main Screen
	Task_Principal = lv_task_create(update_main_screen, 250, LV_TASK_PRIO_MID, NULL);

	lv_scr_load(Tela_Principal);
	TelaAtiva = TelaPrincipal;
}

void print_set_prog(void)
{
	// Create a new style
	static lv_style_t style_txt;
	lv_style_copy(&style_txt, &lv_style_plain);
	style_txt.text.font = &lv_font_eurostile_32;
	style_txt.text.letter_space = 1;
	style_txt.text.line_space = 1;
	style_txt.text.color = LV_COLOR_LIME;

	// Create a new label
	Set = lv_label_create(Tela_Principal, NULL);
	lv_obj_set_style(Set, &style_txt);                    		// Set the created style
	lv_label_set_long_mode(Set, LV_LABEL_LONG_EXPAND);     		// Break the long lines
	lv_label_set_recolor(Set, true);                      		// Enable re-coloring by commands in the text
	lv_label_set_align(Set, LV_ALIGN_OUT_TOP_MID);       		// Center aligned lines
	lv_label_set_text_fmt(Set, "SET   V:%3.1fV   I:%3.1fA", setVolts, setAMP);
	lv_obj_set_width(Set, 300);                           		// Set a width
	lv_obj_align(Set, NULL, LV_ALIGN_OUT_TOP_MID, -200, 78);      	// Align to center
}

void print_headers(void)
{
	// Create a new style
	static lv_style_t style_txt;
	lv_style_copy(&style_txt, &lv_style_plain);
	style_txt.body.main_color = LV_COLOR_BLUE;
	style_txt.text.font = &lv_font_roboto_28;
	style_txt.text.letter_space = 1;
	style_txt.text.line_space = 1;
	style_txt.text.color = LV_COLOR_WHITE;

	// Create a new label
	Tensao = lv_label_create(Tela_Principal, NULL);
	lv_obj_set_style(Tensao, &style_txt);                    		// Set the created style
	lv_label_set_long_mode(Tensao, LV_LABEL_LONG_EXPAND);     		// Break the long lines
	lv_label_set_recolor(Tensao, true);                      		// Enable re-coloring by commands in the text
	lv_label_set_align(Tensao, LV_ALIGN_OUT_TOP_MID);       		// Center aligned lines
	lv_label_set_text(Tensao, "Power Supply Sinteck");
	lv_obj_set_width(Tensao, 300);                           		// Set a width
	lv_obj_align(Tensao, NULL, LV_ALIGN_OUT_TOP_MID, 0, 36);      	// Align to center
}

void create_slider(void)
{
	/* Create a slider in the center of the display */
	slider = lv_slider_create(Tela_Principal, NULL);
	lv_obj_set_width(slider, LV_DPI * 4);
	lv_obj_align(slider, NULL, LV_ALIGN_OUT_TOP_MID, -180, 440);
	lv_obj_set_event_cb(slider, slider_event_cb);
	lv_slider_set_range(slider, 0, 4095);

	/* Create a label below the slider */
	slider_label = lv_label_create(Tela_Principal, NULL);
	lv_label_set_text(slider_label, "0");
	lv_obj_set_auto_realign(slider_label, true);
	lv_obj_align(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

	/* Create an informative label */
//	lv_obj_t * info = lv_label_create(Tela_Principal, NULL);
//	lv_label_set_text(info, "PWM:\n");
//	lv_obj_align(info, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
}

void create_buttons(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(Tela_Principal, NULL);
    lv_obj_set_event_cb(btn1, btn_event_handler_up);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 300, 20);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "+");

    lv_obj_t * btn2 = lv_btn_create(Tela_Principal, NULL);
    lv_obj_set_event_cb(btn2, btn_event_handler_dn);
    lv_obj_align(btn2, NULL, LV_ALIGN_CENTER, 300, 130);
    lv_btn_set_toggle(btn2, true);
    lv_btn_toggle(btn2);
    lv_btn_set_fit2(btn2, LV_FIT_NONE, LV_FIT_TIGHT);

    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "-");
}

void print_values(void)
{
	// Create a new style
	static lv_style_t style_txt;
	lv_style_copy(&style_txt, &lv_style_plain);
	style_txt.text.font = &lv_font_eurostile_128;
	style_txt.text.letter_space = 1;
	style_txt.text.line_space = 1;
	style_txt.text.color = LV_COLOR_WHITE;

	// Create a new label
	Tensao = lv_label_create(Tela_Principal, NULL);
	lv_obj_set_style(Tensao, &style_txt);                    		// Set the created style
	lv_label_set_long_mode(Tensao, LV_LABEL_LONG_EXPAND);     		// Break the long lines
	lv_label_set_recolor(Tensao, true);                      		// Enable re-coloring by commands in the text
	lv_label_set_align(Tensao, LV_ALIGN_IN_LEFT_MID);       		// Center aligned lines
	lv_label_set_text_fmt(Tensao, "%3.1fV", tensao);
	lv_obj_set_width(Tensao, 300);                           		// Set a width
	lv_obj_align(Tensao, NULL, LV_ALIGN_IN_LEFT_MID, 10, -100);      // Align to center

	// Create a new label
	Corrente = lv_label_create(Tela_Principal, NULL);
	lv_obj_set_style(Corrente, &style_txt);                    		// Set the created style
	lv_label_set_long_mode(Corrente, LV_LABEL_LONG_EXPAND);     		// Break the long lines
	lv_label_set_recolor(Corrente, true);                      		// Enable re-coloring by commands in the text
	lv_label_set_align(Corrente, LV_ALIGN_IN_LEFT_MID);       		// Center aligned lines
	lv_label_set_text_fmt(Corrente, "%3.1fA", corrente);
	lv_obj_set_width(Corrente, 300);                           		// Set a width
	lv_obj_align(Corrente, NULL, LV_ALIGN_IN_LEFT_MID, 10, 0);      // Align to center

	// Create a new label
	Potencia = lv_label_create(Tela_Principal, NULL);
	lv_obj_set_style(Potencia, &style_txt);                    		// Set the created style
	lv_label_set_long_mode(Potencia, LV_LABEL_LONG_EXPAND);     		// Break the long lines
	lv_label_set_recolor(Potencia, true);                      		// Enable re-coloring by commands in the text
	lv_label_set_align(Potencia, LV_ALIGN_IN_LEFT_MID);       		// Center aligned lines
	if(potencia > 1000)
		lv_label_set_text_fmt(Potencia, "%3.1fkW", potencia/1000.0f);
	else
		lv_label_set_text_fmt(Potencia, "%3.1fW", potencia);
	lv_obj_set_width(Potencia, 300);                           		// Set a width
	lv_obj_align(Potencia, NULL, LV_ALIGN_IN_LEFT_MID, 10, 100);      // Align to center
}

void create_switch(void)
{
	/*Create styles for the switch*/
	static lv_style_t bg_style;
	static lv_style_t indic_style;
	static lv_style_t knob_on_style;
	static lv_style_t knob_off_style;

	lv_style_copy(&bg_style, &lv_style_pretty);
	bg_style.body.radius = LV_RADIUS_CIRCLE;
	bg_style.body.padding.top = 6;
	bg_style.body.padding.bottom = 6;

	lv_style_copy(&indic_style, &lv_style_pretty_color);
	indic_style.body.radius = LV_RADIUS_CIRCLE;
	indic_style.body.main_color = lv_color_hex(0x9fc8ef);
	indic_style.body.grad_color = lv_color_hex(0x9fc8ef);
	indic_style.body.padding.left = 0;
	indic_style.body.padding.right = 0;
	indic_style.body.padding.top = 0;
	indic_style.body.padding.bottom = 0;

	lv_style_copy(&knob_off_style, &lv_style_pretty);
	knob_off_style.body.radius = LV_RADIUS_CIRCLE;
	knob_off_style.body.shadow.width = 4;
	knob_off_style.body.shadow.type = LV_SHADOW_BOTTOM;

	lv_style_copy(&knob_on_style, &lv_style_pretty_color);
	knob_on_style.body.radius = LV_RADIUS_CIRCLE;
	knob_on_style.body.shadow.width = 4;
	knob_on_style.body.shadow.type = LV_SHADOW_BOTTOM;

	/*Create a switch and apply the styles*/
	lv_obj_t *sw1 = lv_sw_create(Tela_Principal, NULL);
	lv_sw_set_style(sw1, LV_SW_STYLE_BG, &bg_style);
	lv_sw_set_style(sw1, LV_SW_STYLE_INDIC, &indic_style);
	lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_ON, &knob_on_style);
	lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_OFF, &knob_off_style);
	lv_obj_align(sw1, NULL, LV_ALIGN_CENTER, 300, -80);
	lv_obj_set_event_cb(sw1, event_handler);
}

void update_main_screen(lv_task_t * param)
{
	lv_label_set_text_fmt(Tensao, "%3.1fV", tensao);
	lv_label_set_text_fmt(Corrente, "%3.1fA", corrente);
	if(potencia > 1000)
		lv_label_set_text_fmt(Potencia, "%3.1fkW", potencia/1000.0f);
	else
		lv_label_set_text_fmt(Potencia, "%3.1fW", potencia);
}

void slider_event_cb(lv_obj_t * slider, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        static char buf[5]; /* max 3 bytes for number plus 1 null terminating byte */
        snprintf(buf, 5, "%u", lv_slider_get_value(slider));
        lv_label_set_text(slider_label, buf);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, lv_slider_get_value(slider));
        //lv_slider_set_value(slider, lv_slider_get_value(slider), LV_ANIM_OFF);
    }
}

void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        //printf("State: %s\n", lv_sw_get_state(obj) ? "On" : "Off");
    	if(lv_sw_get_state(obj)) {
    		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_SET);
    	}
    	else {
    		HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
    	}
    }
}

void btn_event_handler_up(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        static char buf[5]; /* max 3 bytes for number plus 1 null terminating byte */
        if(lv_slider_get_value(slider) <= 4095) {
        	snprintf(buf, 5, "%u", lv_slider_get_value(slider)+1);
        	lv_label_set_text(slider_label, buf);
        	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, lv_slider_get_value(slider)+1);
        	lv_slider_set_value(slider, lv_slider_get_value(slider)+1, LV_ANIM_OFF);
        }
    }
    else if(event == LV_EVENT_VALUE_CHANGED) {
        //printf("Toggled\n");
    }
}

void btn_event_handler_dn(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
    	if(lv_slider_get_value(slider) >= 1) {
    		static char buf[5]; /* max 3 bytes for number plus 1 null terminating byte */
    		snprintf(buf, 5, "%u", lv_slider_get_value(slider)-1);
    		lv_label_set_text(slider_label, buf);
    		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, lv_slider_get_value(slider)-1);
    		lv_slider_set_value(slider, lv_slider_get_value(slider)-1, LV_ANIM_OFF);
    	}
    }
    else if(event == LV_EVENT_VALUE_CHANGED) {
        //printf("Toggled\n");
    }
}
