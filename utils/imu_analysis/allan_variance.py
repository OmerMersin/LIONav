#!/usr/bin/env python3
import numpy as np, allantools as at, matplotlib.pyplot as plt, csv

# --- Load IMU CSV ---
data = np.loadtxt("imu_data.csv", delimiter=",", skiprows=1)
t = data[:,0]
gx, gy, gz = data[:,4:7].T
ax, ay, az = data[:,1:4].T
rate = 1/np.median(np.diff(t))
print(f"Sampling rate ≈ {rate:.2f} Hz")

# --- Allan variance ---
taus = np.logspace(-1, 3, 60)
adev_gz = at.adev(gz, rate=rate, data_type="freq", taus=taus)
adev_az = at.adev(az, rate=rate, data_type="freq", taus=taus)

# --- Plot ---
plt.loglog(adev_gz[0], adev_gz[1], label="gyro Z")
plt.loglog(adev_az[0], adev_az[1], label="accel Z")
plt.xlabel("Tau [s]"); plt.ylabel("Allan Deviation"); plt.legend(); plt.grid()
plt.show()

# --- Estimate noise densities ---
tau0 = 1.0  # 1 s point in the -½ slope region
gyro_noise = np.interp(tau0, adev_gz[0], adev_gz[1]) * np.sqrt(tau0)
acc_noise  = np.interp(tau0, adev_az[0], adev_az[1]) * np.sqrt(tau0)

print(f"≈ imuGyrNoise {gyro_noise:.2e} rad/s/√Hz")
print(f"≈ imuAccNoise {acc_noise:.2e} m/s²/√Hz")

# --- Very rough bias random walk estimate (from +½ slope region) ---
tauB = 100.0
gyro_bias = np.interp(tauB, adev_gz[0], adev_gz[1]) / np.sqrt(tauB)
acc_bias  = np.interp(tauB, adev_az[0], adev_az[1]) / np.sqrt(tauB)

print(f"≈ imuGyrBiasN {gyro_bias:.2e}")
print(f"≈ imuAccBiasN {acc_bias:.2e}")
