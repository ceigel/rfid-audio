const HEX_CHARS_UPPER: &[u8; 16] = b"0123456789ABCDEF";

fn generate_iter(len: usize) -> impl Iterator<Item = (usize, usize)> {
    (0..len).step_by(2).zip((0..len).skip(1).step_by(2))
}
fn byte2hex(byte: u8) -> (u8, u8) {
    let high = HEX_CHARS_UPPER[((byte & 0xf0) >> 4) as usize];
    let low = HEX_CHARS_UPPER[(byte & 0x0f) as usize];

    (high, low)
}

pub fn encode_to_slice(input: &[u8], output: &mut [u8]) {
    for (byte, (i, j)) in input
        .as_ref()
        .iter()
        .zip(generate_iter(input.as_ref().len() * 2))
    {
        let (high, low) = byte2hex(*byte);
        output[i] = high;
        output[j] = low;
    }
}
