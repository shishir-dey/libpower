#[cfg(test)]
mod tests {
    mod perturb_and_observe_tests {
        use libpower::mppt::mppt::perturb_and_observe::MPPT;

        #[test]
        fn test_new_mppt_initialization() {
            let mppt = MPPT::new();
            assert_eq!(mppt.get_pv_i(), 0.0);
            assert_eq!(mppt.get_pv_v(), 0.0);
            assert_eq!(mppt.get_pv_power(), 0.0);
            assert_eq!(mppt.get_mppt_v_out(), 0.0);
        }

        #[test]
        fn test_first_calculation() {
            let mut mppt = MPPT::new();
            mppt.calculate(1.0, 2.0);

            // First calculation should only store values
            assert_eq!(mppt.get_pv_i(), 0.0);
            assert_eq!(mppt.get_pv_v(), 0.0);
            assert_eq!(mppt.get_pv_power(), 0.0);
        }

        #[test]
        fn test_power_increase_voltage_increase() {
            let mut mppt = MPPT::new();

            // Initial call to set previous values
            mppt.calculate(1.0, 10.0);

            // Second call with increased voltage and power
            mppt.calculate(1.5, 12.0);

            assert_eq!(mppt.get_pv_i(), 1.5);
            assert_eq!(mppt.get_pv_v(), 12.0);
            assert_eq!(mppt.get_pv_power(), 18.0);
        }

        #[test]
        fn test_power_increase_voltage_decrease() {
            let mut mppt = MPPT::new();
            // Initial call to set previous values
            mppt.calculate(1.0, 10.0);

            // Second call with decreased voltage but increased power
            mppt.calculate(2.0, 8.0);

            assert_eq!(mppt.get_pv_i(), 2.0);
            assert_eq!(mppt.get_pv_v(), 8.0);
            assert_eq!(mppt.get_pv_power(), 16.0);
        }
    }

    mod incremental_conductance_tests {
        use libpower::mppt::mppt::incremental_conductance::MPPT;

        #[test]
        fn test_new_mppt_initialization() {
            let mppt = MPPT::new();
            assert_eq!(mppt.get_pv_i(), 0.0);
            assert_eq!(mppt.get_pv_v(), 0.0);
            assert_eq!(mppt.get_mppt_v_out(), 0.0);
        }

        #[test]
        fn test_first_calculation() {
            let mut mppt = MPPT::new();
            mppt.calculate(1.0, 2.0);

            // First calculation should only store values
            assert_eq!(mppt.get_pv_i(), 0.0);
            assert_eq!(mppt.get_pv_v(), 0.0);
        }

        #[test]
        fn test_positive_delta_v_positive_delta_i() {
            let mut mppt = MPPT::new();

            // Initial call to set previous values
            mppt.calculate(1.0, 10.0);

            // Second call with increased voltage and current
            mppt.calculate(1.5, 12.0);

            assert_eq!(mppt.get_pv_i(), 1.5);
            assert_eq!(mppt.get_pv_v(), 12.0);
        }

        #[test]
        fn test_positive_delta_v_negative_delta_i() {
            let mut mppt = MPPT::new();

            // Initial call to set previous values
            mppt.calculate(2.0, 10.0);

            // Second call with increased voltage but decreased current
            mppt.calculate(1.5, 12.0);

            assert_eq!(mppt.get_pv_i(), 1.5);
            assert_eq!(mppt.get_pv_v(), 12.0);
        }

        #[test]
        fn test_negative_delta_v_positive_delta_i() {
            let mut mppt = MPPT::new();

            // Initial call to set previous values
            mppt.calculate(1.0, 12.0);

            // Second call with decreased voltage but increased current
            mppt.calculate(1.5, 10.0);

            assert_eq!(mppt.get_pv_i(), 1.5);
            assert_eq!(mppt.get_pv_v(), 10.0);
        }
    }
}
