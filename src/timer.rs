//! TMRx Timer.
//!
//! 4 26-bit timers TMR0 to TMR3. TMR1 and TMR2 support DMA.

#[derive(Clone, Copy)]
pub enum InputCaptureMode {
    Rising,
    Falling,
    BothEdges,
}
