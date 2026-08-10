#include "feature_registry.h"

bool FeatureRegistry::register_module(const FeatureModule &m) {
    if (count_ >= kMaxFeatureModules) return false;
    if (find_by_id(m.id) != nullptr) return false; // reject duplicate ids
    modules_[count_++] = m;
    return true;
}

const FeatureModule *FeatureRegistry::find_by_id(const char *id) const {
    for (int i = 0; i < count_; i++) {
        if (strcmp(modules_[i].id, id) == 0) return &modules_[i];
    }
    return nullptr;
}

void FeatureRegistry::for_each_in_category(Category c, const std::function<void(const FeatureModule &)> &fn) const {
    for (int i = 0; i < count_; i++) {
        if (modules_[i].category == c) fn(modules_[i]);
    }
}

int FeatureRegistry::count_in_category(Category c) const {
    int n = 0;
    for (int i = 0; i < count_; i++) {
        if (modules_[i].category == c) n++;
    }
    return n;
}
