/*
* Revision: V1.0
* Author: Shishir Dey
* Date: January 20th, 2022
* Description: Initial commit
*/

#[allow(dead_code)]
pub fn calculate_checksum(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for byte in data {
        crc = crc.wrapping_add(*byte as u16);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc() {
        let data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
        let crc = calculate_checksum(&data);
        assert_eq!(crc, 0x24);
    }
}
