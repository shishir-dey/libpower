<p align="center">
    <img src="assets/logo.jpg" alt="libpower-logo"/>
</p>

<p align="center">
    <a href="https://img.shields.io/badge/license-MIT-blue.svg">
        <img src="https://img.shields.io/badge/license-MIT-blue.svg" alt="GitHub license" />
    </a>&nbsp;
    <a href="https://img.shields.io/github/workflow/status/shishir-dey/libpower/Rust%20(host)">
        <img src="https://img.shields.io/github/workflow/status/shishir-dey/libpower/Rust%20(host)"
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


<h3>Documentation</h3>

| Module      | Sub-module | Description |
| ----------- | ----------- | ----------- |
| Battery | - | - |
| Charge Controller      | mppt.rs       | [Maximum power point tracking](https://en.wikipedia.org/wiki/Maximum_power_point_tracking) |
| Control      | pid.rs       | [PID controller](https://en.wikipedia.org/wiki/PID_controller) |
| Inverter      | spwm.rs       | [Pulse-width modulation: Electrical](https://en.wikipedia.org/wiki/Pulse-width_modulation#Electrical) |
| Math      | -       | Math, for when you are high |
| Phase Locked Loop | sogi.rs | Second Order Generalized Integrator |
| Portable | hal.rs | Hardware Abstraction Layer |
| Portable | osal.rs | OS Abstraction Layer |
| Signal      | Filter/fir.rs       | [Finite impulse response](https://en.wikipedia.org/wiki/Finite_impulse_response) |
| Signal      | Filter/iir.rs       | [Infinite impulse response](https://en.wikipedia.org/wiki/Infinite_impulse_response) |
| Signal      | Filter/kalman.rs       | [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) |
| Signal      | Generator/generator.rs       | Generate desired signals in the form of Rust arrays |
| Signal      | Transform/fft.rs       | [Fast Fourier Transform](https://en.wikipedia.org/wiki/Fast_Fourier_transform) |
| Transform      | abc_dq0.rs       | ABC to DQ0 Transform |
| Transform      | clarke.rs       | Clarke Transform |
| Transform      | dq0_abc.rs       | DQ0 to ABC Transform |
| Transform      | iclarke.rs       | Inverse Clarke Transform |
| Transform      | ipark.rs       | Inverse Park Transform |
| Transform      | park.rs       | Park Transform |


<h3>References</h3>

+ [Texas Instruments Solar library](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/SolarLib.pdf)

+ [GitHub/pms67](https://github.com/pms67)
