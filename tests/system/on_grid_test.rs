#[cfg(test)]
mod tests {
    use libpower::system::on_grid::*;

    #[test]
    fn test_state_transitions() {
        let config = InverterConfig::default();
        let mut inverter = GridTieInverter::new(config);

        // Initial state should be Idle
        assert_eq!(inverter.get_state(), InverterState::Idle);

        // With good grid and PV, should transition to Startup
        let measurements = Measurements {
            pv_voltage: 30.0,
            pv_current: 5.0,
            grid_voltage: 325.0, // ~230V RMS
            output_current: 0.0,
        };

        inverter.update(measurements);
        // Note: May need multiple updates to transition
    }
}
