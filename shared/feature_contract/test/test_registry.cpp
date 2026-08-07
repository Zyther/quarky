#include <unity.h>
#include "feature_registry.h"

void test_register_and_find_by_id() {
    FeatureRegistry reg;
    FeatureModule m{"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV};
    TEST_ASSERT_TRUE(reg.register_module(m));

    const FeatureModule *found = reg.find_by_id("ping");
    TEST_ASSERT_NOT_NULL(found);
    TEST_ASSERT_EQUAL_STRING("Ping Satellite", found->name);
}

void test_duplicate_id_rejected() {
    FeatureRegistry reg;
    FeatureModule m{"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV};
    TEST_ASSERT_TRUE(reg.register_module(m));
    TEST_ASSERT_FALSE(reg.register_module(m));
}

void test_for_each_in_category() {
    FeatureRegistry reg;
    reg.register_module({"ping", "Ping", Category::UTILITY, Affinity::CARDPUTER_ADV});
    reg.register_module({"wifi_scan", "WiFi Scan", Category::WIFI, Affinity::TAB5_NATIVE});

    int count = 0;
    reg.for_each_in_category(Category::UTILITY, [&count](const FeatureModule &) { count++; });
    TEST_ASSERT_EQUAL_INT(1, count);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_register_and_find_by_id);
    RUN_TEST(test_duplicate_id_rejected);
    RUN_TEST(test_for_each_in_category);
    return UNITY_END();
}
