#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "vl6180x_api.h"
#include "vl6180x_def.h"
#include "vl6180x_platform.h"

#include "esp_log.h"
#define TAG "VL6180X"

#define VL6180x_I2C_ADDRESS_DEFAULT 0x29

typedef struct {
    i2c_port_t i2c_port;
    i2c_master_dev_handle_t* handle;
    int32_t TimingBudgetMicroSeconds;
} VL6180X;

bool VL6180X_init(VL6180X* vl);
bool VL6180X_read(VL6180X* vl, uint16_t *pRangeMilliMeter);


bool VL6180X_init(VL6180X* vl) {
    /* device init */
    if (VL6180x_InitData(vl->handle) < 0) {
        ESP_LOGE(TAG, "InitData");
        return false;
    }
    if (VL6180x_Prepare(vl->handle) < 0) {
        ESP_LOGE(TAG, "Prepare");
        return false;
    }
    return true;
}



bool VL6180X_read(VL6180X* vl, uint16_t *pRangeMilliMeter) {
    // ESP_LOGI(TAG, "VL6180X_read");
    VL6180x_RangeData_t Range;
    int status = VL6180x_RangePollMeasurement(vl->handle, &Range);
    if (status != 0 || Range.errorStatus != 0) {
        ESP_LOGW(TAG, "i2c status: %d, range status: %s", status,
                 VL6180x_RangeGetStatusErrString(Range.errorStatus));
        return false;
    }
    *pRangeMilliMeter = Range.range_mm;
    return true;
}


