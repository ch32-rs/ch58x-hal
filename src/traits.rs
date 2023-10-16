macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance>: crate::gpio::Pin {
            fn is_remap(&self) -> bool;
        }
    };
}

// See: https://stackoverflow.com/questions/26731243/how-do-i-use-a-macro-across-module-files
pub(crate) use pin_trait;
