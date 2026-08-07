#pragma once
#include "../hal/idisplay.h"
#include "../hal/itouch.h"

void lvgl_port_init(IDisplay &display, ITouch &touch);
void lvgl_port_tick(); // call every loop() iteration
