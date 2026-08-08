#pragma once
#include "idisplay.h"
#include "../../boards/tab5/pins_config.h"

class DisplayTab5 : public IDisplay {
public:
    // Which panel controller runtime detection settled on. The Tab5 ships with
    // one of THREE mutually exclusive display ICs; see detectController() in
    // the .cpp. St7121 and St7123 are different panels that both answer at
    // I2C 0x55 and need different init tables, lane rates and DPI timing --
    // they are told apart by the touch firmware version, exactly as M5Stack's
    // own M5GFX library does.
    enum class Controller { Unknown, Ili9881, St7121, St7123 };

    void init() override;
    // Logical (LVGL-facing) size. This is NOT necessarily the panel's native
    // size -- the panel is natively 720x1280 portrait and flush() rotates into
    // it. See TAB5_DISPLAY_ROTATION in boards/tab5/pins_config.h.
    int width() const override { return TAB5_DISP_WIDTH; }
    int height() const override { return TAB5_DISP_HEIGHT; }
    void flush(int x1, int y1, int x2, int y2, const uint16_t *colors) override;

    // Diagnostics. `ready()` is false when panel bring-up failed, in which case
    // flush() is a no-op and the rest of the firmware still runs (same
    // degrade-don't-brick policy as touch and the radios).
    Controller controller() const { return controller_; }
    bool ready() const { return panel_handle_ != nullptr; }

private:
    void *panel_handle_ = nullptr; // esp_lcd_panel_handle_t, opaque here to keep this header light
    Controller controller_ = Controller::Unknown;
};
