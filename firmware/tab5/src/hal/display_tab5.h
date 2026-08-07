#pragma once
#include "idisplay.h"

class DisplayTab5 : public IDisplay {
public:
    void init() override;
    int width() const override { return TAB5_DISP_WIDTH_; }
    int height() const override { return TAB5_DISP_HEIGHT_; }
    void flush(int x1, int y1, int x2, int y2, const uint16_t *colors) override;

private:
    static constexpr int TAB5_DISP_WIDTH_ = 1280;
    static constexpr int TAB5_DISP_HEIGHT_ = 720;
    void *panel_handle_ = nullptr; // esp_lcd_panel_handle_t, opaque here to keep this header light
};
