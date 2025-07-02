

#include "accelerometer.h"

#if defined(SL_COMPONENT_CATALOG_PRESENT)
#include "sl_component_catalog.h"
#endif

#if defined (SL_CATALOG_ICM20689_DRIVER_PRESENT)
#include "sl_icm20689_config.h"
#define  SL_IMU_INT_PORT SL_ICM20689_INT_PORT
#define  SL_IMU_INT_PIN  SL_ICM20689_INT_PIN
#elif defined (SL_CATALOG_ICM20648_DRIVER_PRESENT)
#include "sl_icm20648_config.h"
#define  SL_IMU_INT_PORT SL_ICM20648_INT_PORT
#define  SL_IMU_INT_PIN  SL_ICM20648_INT_PIN
#else
#error "No IMU driver defined"
#endif

// Accelerometer data from sensor
typedef struct imu_data {
  int16_t x;
  int16_t y;
  int16_t z;
} imu_data_t;

sl_status_t accelerometer_setup(GPIOINT_IrqCallbackPtrExt_t callbackPtr)
{
  sl_status_t status = SL_STATUS_OK;
  int int_id;

  // Initialize accelerometer sensor
  status = sl_imu_init();
  if (status != SL_STATUS_OK) {
    return status;
  }
  sl_imu_configure(ACCELEROMETER_FREQ);
  // Setup interrupt from accelerometer on falling edge
  GPIO_PinModeSet(SL_IMU_INT_PORT, SL_IMU_INT_PIN, gpioModeInput, 0);
  int_id = GPIOINT_CallbackRegisterExt(SL_IMU_INT_PIN, callbackPtr, 0);
  if (int_id != INTERRUPT_UNAVAILABLE) {
    GPIO_ExtIntConfig(SL_IMU_INT_PORT, SL_IMU_INT_PIN, int_id, false, true, true);
  } else {
    status = SL_STATUS_FAIL;
  }
  return status;
}

sl_status_t accelerometer_read(acc_data_t* dst)
{
  if (!sl_imu_is_data_ready()) {
    return SL_STATUS_FAIL;
  }
  sl_imu_update();
  int16_t m[3];
  sl_imu_get_acceleration(m);
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  if (dst != NULL) {
    dst->x = m[0];
    dst->y = m[1];
    dst->z = m[2];
  }
  CORE_EXIT_CRITICAL();
  return SL_STATUS_OK;
}
