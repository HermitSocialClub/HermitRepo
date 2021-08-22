#include <iostream>
#include <jni.h>

extern "C" JNIEXPORT jint JNICALL Java_org_hermitsocialclub_tomato_LibTomato_splat(JNIEnv* env) {
    return 69;
}
