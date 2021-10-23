use jni::objects::{JMethodID, JObject, JValue};
use jni::signature::{JavaType, Primitive};
use jni::sys::jobject;
use jni::JNIEnv;

const VISION_PIPELINE_CLASS: &str = "org/hermitsocialclub/hydra/vision/VisionPipeline";
const TELEMETRY_TYPE: &str = "Lorg/hermitsocialclub/telecat/PersistantTelemetry;";

/// A wrapper around an `org.hermitsocialclub.telecat.PersistantTelemetry` instance.
/// This is only valid for the lifetime of the `PersistantTelemetry` object it wraps.
// noinspection SpellCheckingInspection
pub struct PersistantTelemetry<'a> {
    env: JNIEnv<'a>,
    obj: JObject<'a>,
    set_data: JMethodID<'a>,
}

impl<'a> PersistantTelemetry<'a> {
    #[allow(clippy::wrong_self_convention)]
    pub fn set_data<K, V>(&self, key: K, value: V)
    where
        K: ToString,
        V: ToString,
    {
        let j_key = self.env.new_string(key.to_string()).unwrap();
        let j_value = self.env.new_string(value.to_string()).unwrap();
        self.env
            .call_method_unchecked(
                self.obj,
                self.set_data,
                JavaType::Primitive(Primitive::Void),
                &[JValue::Object(j_key.into()), JValue::Object(j_value.into())],
            )
            .unwrap();
    }
}

/// Gets a [`PersistantTelemetry`] from a `VisionPipeline` instance.
pub fn get_telemetry_from_pipeline(env: JNIEnv, pipeline: jobject) -> PersistantTelemetry {
    let telemetry_field_id = env
        .get_field_id(VISION_PIPELINE_CLASS, "telemetry", TELEMETRY_TYPE)
        .unwrap();
    let telemetry = env
        .get_field_unchecked(
            pipeline,
            telemetry_field_id,
            JavaType::Object(TELEMETRY_TYPE.to_string()),
        )
        .unwrap()
        .l()
        .unwrap();
    let telemetry_class = env.get_object_class(telemetry).unwrap();
    let set_data = env
        .get_method_id(telemetry_class, "setData", "(Ljava/lang/String;Ljava/lang/Object;)V")
        .unwrap();
    PersistantTelemetry {
        env,
        obj: telemetry,
        set_data,
    }
}
