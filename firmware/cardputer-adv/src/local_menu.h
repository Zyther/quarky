#pragma once

namespace LocalMenu {
void init();
void tick(); // call every loop() iteration; polls keyboard, redraws if changed
}
