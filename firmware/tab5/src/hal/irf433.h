#pragma once

// Detection/init-only RF433R/T interface (Task 18). Full transmit/receive/
// clone/replay logic is Phase 3 scope.
class IRF433 {
public:
    virtual ~IRF433() = default;
    virtual bool init() = 0;
};
