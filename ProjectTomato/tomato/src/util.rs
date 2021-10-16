use std::panic::{catch_unwind, UnwindSafe};

use jni::JNIEnv;

/// Catches Rust panics and throws a Java exception.
///
/// Don't call this function directly; instead use the
/// `#[catch_panic]` macro.
pub fn __catch_panic<F: FnOnce() -> R + UnwindSafe, R: Default>(env: JNIEnv, f: F) -> R {
    match catch_unwind(f) {
        Ok(result) => result,
        Err(err) => {
            let msg = match err.downcast_ref::<&'static str>() {
                Some(s) => *s,
                None => match err.downcast_ref::<String>() {
                    Some(s) => &s[..],
                    None => "Box<dyn Any>",
                },
            };
            env.throw_new("org/hermitsocialclub/tomato/TomatoException", msg)
                .unwrap();
            R::default()
        }
    }
}
