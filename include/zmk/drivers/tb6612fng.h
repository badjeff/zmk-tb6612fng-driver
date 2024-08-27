#pragma once

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Helper macros used to convert sensor values. */
#define TB6612FNG_SVALUE_TO_ENABLE(svalue) ((uint32_t)(svalue).val1)
#define TB6612FNG_SVALUE_TO_VELOCITY_VEL(svalue) ((uint32_t)(svalue).val1)
#define TB6612FNG_SVALUE_TO_VELOCITY_INV(svalue) ((bool)(svalue).val2)
#define TB6612FNG_SVALUE_TO_SYNC(svalue) ((uint32_t)(svalue).val1)

/** @brief Sensor specific attributes of TB6612FNG. */
enum drv883x_attribute {
	TB6612FNG_ATTR_ENABLE,
	TB6612FNG_ATTR_VELOCITY,
	TB6612FNG_ATTR_SYNC,
};

#ifdef __cplusplus
}
#endif
