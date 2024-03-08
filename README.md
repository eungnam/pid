
# Project Name: PID Numerical Test

## Version: v0.x.x-alpha

### Description
This program conducts an experiment comparing the outputs (\(u(t)\)) of the classic PID formula and the modified PID formula.

### Environment
- Developed using Visual Studio Code version 1.86.2 for Python and C/C++ programming.
- Utilizes the GCC/G++ compiler version 8.1.0.
- Performance calculations are executed in MATLAB version R2022a.

### Prerequisites
1. Install Visual Studio Code (freeware).
2. Install a compiler (GCC/G++ recommended).
3. Install MATLAB.

### Files
0. `Appendix` - Proof of the modified PID formula.
1. `1_reference.py` - Python file that outputs two types of reference \(u(t)\).
2. `2_test` - Outputs two types of single precision \(u(t)\).
3. `3_PerformanceTest` - Uses MATLAB to test the performance of the outputs.

### Usage
1. Execute `2_reference.py` to output two types of reference \(u(t)\). You can set the values for Kp, Ki, and Kd.
2. Execute `3_test` to output two types of single precision \(u(t)\). You can also set the values for Kp, Ki, and Kd.
3. In the editing file of `4_PerformanceTest`, copy and paste the output values from `2_reference.py` and `3_test` to execute the performance test.
