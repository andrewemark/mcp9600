/// Sets the most significant nibble of a u8 to a value
pub fn set_ms_nibble(byte: &mut u8, val: u8) {
    *byte &= 0x0Fu8;
    *byte |= val << 4
}

/// Sets the least significant nibble of a u8 to a value
pub fn set_ls_nibble(byte: &mut u8, val: u8) {
    *byte &= 0xF0u8;
    *byte |= val & 0x0F;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_set_ms_nibble() {
        let mut a = 0x00;
        set_ms_nibble(&mut a, 0xF);
        assert_eq!(a, 0xF0);

        let mut b = 0x0F;
        set_ms_nibble(&mut b, 0xA);
        assert_eq!(b, 0xAF);

        let mut c = 0xBF;
        set_ms_nibble(&mut c, 0xA);
        assert_eq!(c, 0xAF);
    }

    #[test]
    fn test_set_ls_nibble() {
        let mut a = 0x00;
        set_ls_nibble(&mut a, 0xF);
        assert_eq!(a, 0x0F);

        let mut b = 0x0F;
        set_ls_nibble(&mut b, 0xA);
        assert_eq!(b, 0x0A);

        let mut c = 0xBF;
        set_ls_nibble(&mut c, 0xA);

        assert_eq!(c, 0xBA);
    }
}