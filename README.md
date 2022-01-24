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
| Charge Controller      | pwm.rs       | [Pulse-width modulation: Power delivery](https://en.wikipedia.org/wiki/Pulse-width_modulation#Power_delivery) |
| Control      | pid.rs       | [PID controller](https://en.wikipedia.org/wiki/PID_controller) |
| Filter      | fir.rs       | [Finite impulse response](https://en.wikipedia.org/wiki/Finite_impulse_response) |
| Filter      | iir.rs       | [Infinite impulse response](https://en.wikipedia.org/wiki/Infinite_impulse_response) |
| Filter      | kalman.rs       | [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) |
| Inverter      | spwm.rs       | [Pulse-width modulation: Electrical](https://en.wikipedia.org/wiki/Pulse-width_modulation#Electrical) |
| Optimization | - | [Optimization problem](https://en.wikipedia.org/wiki/Optimization_problem) |
| Phase Locked Loop | sogi.rs | Second Order Generalized Integrator |
| Signal      | wavegen.rs       | Generate desired signals in the form of Rust arrays |
| Kernel      | tick.rs       | Implement a ticking mechanism that has to be managed by the application. To be used by the modules that need to be run as a process |
| Transform      | clarke.rs       | Clarke Transform |
| Transform      | fft.rs       | [Fast Fourier Transform](https://en.wikipedia.org/wiki/Fast_Fourier_transform) |
| Transform      | iclarke.rs       | Inverse Clarke Transform |
| Transform      | ipark.rs       | Inverse Park Transform |
| Transform      | park.rs       | Park Transform |
| Misc      | misc.rs       | Contains additional helper functions doing the math |


<h3>References</h3>

+ [Texas Instruments Solar library](https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/SolarLib.pdf)

+ [GitHub/pms67](https://github.com/pms67)
