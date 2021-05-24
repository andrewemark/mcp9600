// Typestate pattern

pub trait ReadWriteState {}

// Encode the typestates with zero-variant enums
pub enum ReadOnly {}
pub enum ReadWrite {}

impl ReadWriteState for ReadOnly {}
impl ReadWriteState for ReadWrite {}
