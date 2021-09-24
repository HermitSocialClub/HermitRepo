#include "../target/ftc265/librealsense2/rs.h"
#include "../target/ftc265/librealsense2/h/rs_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Helper method to grab the pipeline from a T265Camera object
 * using a C-ABI interface.
 * The returned pipeline is only valid for the lifetime of
 * the camera's internal `deviceAndSensors` object.
 */
rs2_pipeline* tomato_steal_device_and_sensors_pipeline(void* env, void* this_obj);

#ifdef __cplusplus
}
#endif
