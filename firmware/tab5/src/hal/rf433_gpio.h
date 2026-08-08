#pragma once
#include "irf433.h"

class Rf433Gpio : public IRF433 {
public:
    bool init() override;
};
