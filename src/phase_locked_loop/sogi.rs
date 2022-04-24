/*
* Version 1.0 | Shishir Dey | April 24th, 2022
* Description: Initial commit (Work in progress)
*/

#[allow(dead_code)]
struct OrthogonalSignalGenerator {
    k: f32,
    x: f32,
    y: f32,
    b0: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    qb0: f32,
    qb1: f32,
    qb2: f32,
}

#[allow(dead_code)]
struct NotchFilter {
    a1: f32,
    b0: f32,
    b1: f32,
}

#[allow(dead_code)]
struct SOGI {
    u: [f32; 3],                          /* 1ph AC signal measured and normalized */
    osg_u: [f32; 3],                      /* Estimated grid voltage */
    osg_qu: [f32; 3],                     /* Estimated orthogonal grid voltage */
    u_q: [f32; 2],                        /* Q axis components of the estimated grid */
    u_d: [f32; 2],                        /* D axis components of the estimated grid */
    ylf: [f32; 2],                        /* Notch filtered output */
    fo: f32,                              /* Instantaneous grid frequency */
    fnom: f32,                            /* Nominal grid frequency */
    theta: [f32; 2],                      /* Grid phase angle */
    cos: f32,                             /* Cosine of grid phase angle */
    sin: f32,                             /* Sine of grid phase angle */
    delta_t: f32,                         /* 1/Frequency of calling PLL routine */
    lpf_coeff: NotchFilter,               /* Notch filter coefficients */
    osg_coeff: OrthogonalSignalGenerator, /* Orthogonal signal generator coefficients */
}

impl SOGI {
    #[allow(dead_code)]
    pub fn new(fnom: f32, delta_t: f32) -> SOGI {
        let mut sogi = SOGI {
            // TODO
            u: [0.0; 3],
            osg_u: [0.0; 3],
            osg_qu: [0.0; 3],
            u_q: [0.0; 2],
            u_d: [0.0; 2],
            ylf: [0.0; 2],
            fo: 0.0,
            fnom: fnom,
            theta: [0.0; 2],
            cos: 0.0,
            sin: 0.0,
            delta_t: delta_t,
            lpf_coeff: NotchFilter {
                a1: 0.0,
                b0: 0.0,
                b1: 0.0,
            },
            osg_coeff: OrthogonalSignalGenerator {
                k: 0.0,
                x: 0.0,
                y: 0.0,
                b0: 0.0,
                b2: 0.0,
                a1: 0.0,
                a2: 0.0,
                qb0: 0.0,
                qb1: 0.0,
                qb2: 0.0,
            },
        };
        sogi.init(fnom);
        sogi
    }
    #[allow(dead_code)]
    pub fn init(&mut self, fnom: f32) {
        self.fnom = fnom;
    }
    #[allow(dead_code)]
    pub fn coeff_update(&mut self) {
        // TODO
    }
    #[allow(dead_code)]
    pub fn run(&mut self) {
        // TODO
    }
}

/* Placeholder for module's unit tests */
#[cfg(test)]
#[allow(unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_will_always_fail() {
        assert!(false);
    }
}
