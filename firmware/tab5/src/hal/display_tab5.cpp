#include "display_tab5.h"
#include "../../boards/tab5/pins_config.h"
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_mipi_dsi.h>
#include <Arduino.h>

// Full MIPI-DSI bus + DPI panel + panel-controller bring-up (esp_lcd_new_dsi_bus,
// esp_lcd_new_panel_io_dbi, esp_lcd_new_panel_dpi, plus the ILI9881/ST7123-specific
// init command list) is intentionally NOT implemented yet. Research into the
// espp/m5stack-tab5 BSP (see boards/tab5/pins_config.h for sources) confirmed the
// real DSI lane config, backlight GPIO, and IO-expander-routed reset line, but the
// per-controller vendor init command tables live in separate espp display-driver
// components that were not transcribed in this pass (see pins_config.h TODO).
// This function currently only brings up the backlight as a minimal bring-up step;
// wiring the real panel init sequence is left for a follow-up task once the
// ILI9881/ST7123 command tables have been sourced.
void DisplayTab5::init() {
    // Panel bus + IO + init-command-list construction follows the exact sequence
    // documented in the espp/m5stack-tab5 BSP (see pins_config.h). This function wires:
    //   1. esp_lcd_new_dsi_bus(...)         -- MIPI-DSI bus from pins_config.h
    //   2. esp_lcd_new_panel_io_dbi(...)    -- panel command IO over the DSI bus
    //   3. esp_lcd_new_panel_*(...)         -- panel driver matching Tab5's controller IC
    //   4. esp_lcd_panel_reset/init/disp_on_off
    // Implementer: transcribe the BSP's exact call sequence and init command
    // list here rather than re-deriving it -- the panel IC's init command list
    // is model-specific and getting it wrong produces a blank or garbled screen.
    pinMode(TAB5_DISP_BL_GPIO, OUTPUT);
    digitalWrite(TAB5_DISP_BL_GPIO, HIGH);
}

void DisplayTab5::flush(int x1, int y1, int x2, int y2, const uint16_t *colors) {
    // esp_lcd_panel_draw_bitmap(panel_handle_, x1, y1, x2, y2, colors);
}
