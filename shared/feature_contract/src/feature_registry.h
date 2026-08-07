#pragma once
#include "feature_module.h"
#include <cstring>
#include <functional>

constexpr int kMaxFeatureModules = 64;

class FeatureRegistry {
public:
    bool register_module(const FeatureModule &m);
    const FeatureModule *find_by_id(const char *id) const;
    void for_each_in_category(Category c, const std::function<void(const FeatureModule &)> &fn) const;
    int count() const { return count_; }

private:
    FeatureModule modules_[kMaxFeatureModules];
    int count_ = 0;
};
