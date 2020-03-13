/*
 * Screen_Main.h
 *
 *  Created on: 3 de mar de 2020
 *      Author: rinal
 */

#ifndef GUI_SCREEN_MAIN_H_
#define GUI_SCREEN_MAIN_H_

void GUI_PowerSupply(void);
void print_values(void);
void create_buttons(void);
void print_set_prog(void);
void print_headers(void);
void create_slider(void);
void update_main_screen(lv_task_t * param);
void slider_event_cb(lv_obj_t * slider, lv_event_t event);
void create_switch(void);
void event_handler(lv_obj_t * obj, lv_event_t event);
void btn_event_handler_up(lv_obj_t * obj, lv_event_t event);
void btn_event_handler_dn(lv_obj_t * obj, lv_event_t event);

enum GUI {
	TelaPrincipal,
	TelaAudio
};


#endif /* GUI_SCREEN_MAIN_H_ */
