#include "io_expander.h"

#include <Wire.h>

#include "../../boards/tab5/pins_config.h"

namespace {

// Read-modify-write one PI4IOE5V6408 register, touching only the given bits.
// Note the single-byte register address -- this differs from the GT911 on the
// same bus, which uses 16-bit big-endian register addresses.
bool update(uint8_t addr, uint8_t reg, uint8_t set_bits, uint8_t clear_bits) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {  // repeated start, keep the bus held
        return false;
    }
    if (Wire.requestFrom((int)addr, 1) != 1) {
        return false;
    }
    const uint8_t before = (uint8_t)Wire.read();
    const uint8_t after = (uint8_t)((before | set_bits) & (uint8_t)~clear_bits);
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(after);
    return Wire.endTransmission() == 0;
}

}  // namespace

namespace tab5_ioexp {

bool set_output(uint8_t i2c_addr, uint8_t bit, bool level) {
    const uint8_t mask = (uint8_t)(1u << bit);
    if (!update(i2c_addr, PI4IOE5V6408_REG_OUTPUT, level ? mask : 0,
                level ? 0 : mask)) {
        return false;
    }
    if (!update(i2c_addr, PI4IOE5V6408_REG_OUTPUT_HIGH_IM, 0, mask)) {
        return false;
    }
    return update(i2c_addr, PI4IOE5V6408_REG_DIRECTION, mask, 0);
}

}  // namespace tab5_ioexp
