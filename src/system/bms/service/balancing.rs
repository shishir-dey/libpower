/// Balancing
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BalancingMode {
    Off,
    Passive,
    Active,
}

pub trait Balancing<const N: usize> {
    fn balancing_mode(&self) -> BalancingMode;
    fn set_balancing_mode(&mut self, mode: BalancingMode);

    fn balancing_current_ma(&self, cell: usize) -> f32;
}
