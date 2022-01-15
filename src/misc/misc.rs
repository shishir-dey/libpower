#[allow(dead_code)]
fn increment(x: i32) -> i32 {
    x + 1
}

#[allow(dead_code)]
fn decrement(x: i32) -> i32 {
    x - 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_increment() {
        assert_eq!(increment(3), 4);
    }

    #[test]
    fn test_decrement() {
        assert_eq!(decrement(3), 2);
    }
}
