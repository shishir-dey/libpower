![libpower logo](assets/logo.jpg "libpower logo")


<h3>About the project</h3>

libpower is a Rust library containing algorithms commonly used in power electronics systems, targeted for use in embedded systems


<h3>Documentation</h3>

| Module      | Sub-module | Description |
| ----------- | ----------- | ----------- |
| Charge Controller      | mppt.rs       | [Maximum power point tracking](https://en.wikipedia.org/wiki/Maximum_power_point_tracking) |
| Charge Controller      | pwm.rs       | [Pulse-width modulation: Power delivery](https://en.wikipedia.org/wiki/Pulse-width_modulation#Power_delivery) |
| Control      | pid.rs       | [PID controller](https://en.wikipedia.org/wiki/PID_controller) |
| Filter      | fir.rs       | [Finite impulse response](https://en.wikipedia.org/wiki/Finite_impulse_response) |
| Filter      | iir.rs       | [Infinite impulse response](https://en.wikipedia.org/wiki/Infinite_impulse_response) |
| Inverter      | spwm.rs       | [Pulse-width modulation: Electrical](https://en.wikipedia.org/wiki/Pulse-width_modulation#Electrical) |
| Signal      | wavegen.rs       | Generate desired signals in the form of Rust arrays |
| System      | tick.rs       | Implement a ticking mechanism that has to be managed by the application. To be used by the library for keeping track of time |
| Transform      | fft.rs       | [Fast Fourier Transform](https://en.wikipedia.org/wiki/Fast_Fourier_transform) |
| Misc      | misc.rs       | Contains additional helper functions doing the math |