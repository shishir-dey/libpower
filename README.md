<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="assets/libpower-dark.svg" width="1280">
  <source media="(prefers-color-scheme: light)" srcset="assets/libpower-light.svg" width="1280">
  <img alt="libpower logo" src="assets/libpower-light.svg" width="1280">
</picture>
</div>

<p align="center">
    <a href="https://img.shields.io/badge/license-MIT-blue.svg">
        <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="GitHub license" />
    </a>&nbsp;
    <a href="https://img.shields.io/github/actions/workflow/status/shishir-dey/libpower/rust_host.yml?branch=main">
        <img src="https://img.shields.io/github/actions/workflow/status/shishir-dey/libpower/rust_host.yml?branch=main"
            alt="GitHub Workflow Status" />
    </a>&nbsp;
    <a href="https://img.shields.io/github/commit-activity/m/shishir-dey/libpower">
        <img src="https://img.shields.io/github/commit-activity/m/shishir-dey/libpower" alt="GitHub commit activity" />
    </a>&nbsp;
    <a href="https://img.shields.io/github/last-commit/shishir-dey/libpower">
        <img src="https://img.shields.io/github/last-commit/shishir-dey/libpower" alt="GitHub last commit" />
    </a>&nbsp;
    <a href="https://img.shields.io/badge/PRs-welcome-brightgreen.svg">
        <img src="https://img.shields.io/badge/PRs-welcome-brightgreen.svg" alt="PRs Welcome" />
    </a>&nbsp;
</p>

<hr>


<h3>About the project</h3>

libpower is a Rust library containing algorithms commonly used in power electronics systems, targeted for use in embedded systems

<h3>Project Structure</h3>

```
src/
├── control/                       # Control algorithms
│   ├── cntl_2p2z.rs               # 2-pole 2-zero controller
│   ├── cntl_3p3z.rs               # 3-pole 3-zero controller
│   ├── cntl_pi.rs                 # Proportional-Integral (PI) controller
│   └── cntl_pid.rs                # Proportional-Integral-Derivative (PID) controller
│
├── filter/                        # Digital filters
│   ├── butterworth_hpf.rs         # Butterworth high-pass filter
│   ├── butterworth_lpf.rs         # Butterworth low-pass filter
│   ├── chebyshev_hpf.rs           # Chebyshev high-pass filter
│   └── chebyshev_lpf.rs           # Chebyshev low-pass filter
│
├── lib.rs                         # Crate root and public API
│
├── modulation/                    # Modulation techniques
│   └── svpwm.rs                   # Space Vector PWM implementation
│
├── motor_control/                 # Motor control algorithms
│   └── foc.rs                     # Field-Oriented Control (FOC)
│
├── mppt/                          # Maximum Power Point Tracking (MPPT)
│   ├── incremental_conductance.rs # Incremental Conductance algorithm
│   └── perturb_and_observe.rs     # Perturb & Observe algorithm
│
├── pll/                           # Phase-Locked Loops (PLL)
│   └── spll_1ph_sogi.rs           # Single-phase SOGI-based PLL
│
├── signal/                        # Signal processing utilities
│   └── signal.rs                  # Signal utility functions
│
└── transform/                     # Power system transforms
    ├── abc_dq0.rs                 # ABC to DQ0 transformation
    ├── clarke.rs                  # Clarke transform
    ├── dq0_abc.rs                 # DQ0 to ABC transformation
    ├── iclarke.rs                 # Inverse Clarke transform
    ├── ipark.rs                   # Inverse Park transform
    └── park.rs                    # Park transform
```

<h3>References</h3>

+ [Texas Instruments Solar library](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/SolarLib.pdf)

+ [GitHub/pms67](https://github.com/pms67)
