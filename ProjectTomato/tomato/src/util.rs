use std::any::Any;
use std::panic::{catch_unwind, UnwindSafe};

use jni::JNIEnv;

/// Project Tomato `#[catch_panic]` handler.
pub fn catch_panic_handler(env: JNIEnv, err: Box<dyn Any + Send + 'static>) {
    let msg = match err.downcast_ref::<&'static str>() {
        Some(s) => *s,
        None => match err.downcast_ref::<String>() {
            Some(s) => &s[..],
            None => "Box<dyn Any>",
        },
    };
    env.throw_new("org/hermitsocialclub/tomato/TomatoException", msg).unwrap();
}
