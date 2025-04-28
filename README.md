# 1D Kalman Filter Localization

This project implements a simple 1D Kalman Filter in C++ to estimate the position and velocity of a moving object based on noisy position measurements.

## 🛠 Features
- **State:** `[position, velocity]`
- **Measurements:** Only noisy **position** observations
- **Prediction and Correction:** Standard Kalman filter equations
- **Uncertainty Modeling:** Process noise and measurement noise
- **Outputs:** Estimated position, estimated velocity, and evolving state covariance matrix
- **Visualization:** (Optional) Export to CSV and plot with Python using `matplotlib`

## 📊 Example Output

```
Step 0:
  True position: 0.0025
  Measured position (noisy): 0.594998
  Estimated position: 0.594412
  Estimated velocity: 0.108599
  Current covariance matrix P_:
  0.0999009  0.0098902
  0.00989056     99.021

Step 1:
  True position: 0.01
  Measured position (noisy): 0.824575
  Estimated position: 0.80654
  Estimated velocity: 1.94628
  Current covariance matrix P_:
  0.0916812  0.824564
  0.824564   17.3003

Step 2:
  True position: 0.02
  Measured position (noisy): 0.695704
  Estimated position: 0.752313
  Estimated velocity: 0.500131
  Current covariance matrix P_:
  0.0814676  0.473426
  0.473426   5.21618

Step 3:
  True position: 0.029
  Measured position (noisy): 0.720291
  Estimated position: 0.744243
  Estimated velocity: 0.24179
  Current covariance matrix P_:
  0.0704417  0.294118
  0.294118   2.29958

Step 4:
  True position: 0.0375
  Measured position (noisy): 0.725675
  Estimated position: 0.742165
  Estimated velocity: 0.165369
  Current covariance matrix P_:
  0.0618701   0.19983
  0.19983   1.26232

Step 5:
  True position: 0.048
  Measured position (noisy): 0.94549
  Estimated position: 0.862941
  Estimated velocity: 0.464528
  Current covariance matrix P_:
  0.0554485  0.145265
  0.145265  0.798664

Step 6:
  True position: 0.0595
  Measured position (noisy): 1.00727
  Estimated position: 0.958688
  Estimated velocity: 0.563908
  Current covariance matrix P_:
  0.0506144  0.111183
  0.111183  0.558356

Step 7:
  True position: 0.0715
  Measured position (noisy): 1.08815
  Estimated position: 1.0499
  Estimated velocity: 0.647791
  Current covariance matrix P_:
  0.0469312 0.0886347
  0.0886347   0.42032

Step 8:
  True position: 0.0845
  Measured position (noisy): 1.0916
  Estimated position: 1.1045
  Estimated velocity: 0.630926
  Current covariance matrix P_:
  0.0440908 0.0730547
  0.0730547  0.334862

Step 9:
  True position: 0.0965
  Measured position (noisy): 0.825955
  Estimated position: 1.02394
  Estimated velocity: 0.399986
  Current covariance matrix P_:
  0.0418775 0.0619243
  0.0619243  0.278887
```

✅ Over time, the filter:
- **Reduces uncertainty** (smaller covariance matrix values)
- **Smooths noisy measurements**
- Estimates **velocity**, which was never directly measured

## 📂 Project Structure

```
├── include/
│   └── kalman_filter.hpp
├── src/
│   └── kalman_filter.cpp
├── build/           # Compiled binaries
├── CMakeLists.txt
└── README.md
```

## 🚀 Build and Run Instructions

### Build

```bash
# Create build folder
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Compile
make
```

### Run

```bash
./kalman_filter
```

Outputs results to console.

## 📚 Notes

- **Process noise** models uncertainty in the motion model.
- **Measurement noise** models sensor inaccuracies.
- Initial uncertainty is set high to simulate starting with poor knowledge.
- Kalman Filter fuses predictions and measurements optimally to minimize mean square error.

## 🤖 Future Work
- Extend to 2D / 3D tracking (position and velocity vectors).
- Compare with ground truth paths.
- Implement Extended Kalman Filter (EKF) for nonlinear models.
