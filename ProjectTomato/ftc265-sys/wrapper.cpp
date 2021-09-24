#include "../ftc265/lib/src/main/cpp/t265wrapper.hpp"
#include "wrapper.h"

extern "C" rs2_pipeline* tomato_steal_device_and_sensors_pipeline(void* env, void* this_obj) {
    auto actualEnv = reinterpret_cast<JNIEnv*>(env);
    auto actualThisObj = reinterpret_cast<jobject>(this_obj);
    auto devAndSensors = getDeviceFromClass(actualEnv, actualThisObj);
    return reinterpret_cast<rs2_pipeline*>(devAndSensors->pipeline);
}
