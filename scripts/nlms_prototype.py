import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

# NLMS parameters (seems like the best I can do that doesn't diverge or sound bad)
N = 128                  # Filter length (number of taps)
mu = 0.05                 # NLMS step size (0 < mu <= 1)
epsilon = 1e-8           # Small constant to avoid divide-by-zero
mse_window = 30          # Moving average window for MSE

# desired: clean speech + noise
# reference: noise reference (correlated with noise in desired)
fs_d, desired = wavfile.read('../audio/desired_signal.wav')
fs_r, reference = wavfile.read('../audio/reference_noise.wav')

if desired.ndim > 1:
    desired = desired[:,0]
if reference.ndim > 1:
    reference = reference[:,0]

# Normalize to float32 in [-1, 1]
desired = desired.astype(np.float32) / np.max(np.abs(desired))
reference = reference.astype(np.float32) / np.max(np.abs(reference))

num_samples = min(len(desired), len(reference))
d = desired[:num_samples]
x = reference[:num_samples]

w = np.zeros(N)            # Filter coefficients
y = np.zeros(num_samples)  # Filter output
e = np.zeros(num_samples)  # Error signal

for n in range(N, num_samples):
    x_segment = x[n-N:n][::-1]                # Last N samples
    y[n] = np.dot(w, x_segment)               # Filter output
    e[n] = d[n] - y[n]                        # Error signal
    norm_factor = np.dot(x_segment, x_segment) + epsilon
    w = w + (mu / norm_factor) * e[n] * x_segment  # NLMS update

mse = e**2
mse_windowed = np.convolve(mse, np.ones(mse_window)/mse_window, mode='valid')

noise_before = d - x[:num_samples]  # Rough noise estimate
SNR_in = 10 * np.log10(np.var(d) / np.var(noise_before))
SNR_out = 10 * np.log10(np.var(d) / np.var(e))
SNR_improvement = SNR_out - SNR_in

print(f"SNR in: {SNR_in:.2f} dB")
print(f"SNR out: {SNR_out:.2f} dB")
print(f"SNR improvement: {SNR_improvement:.2f} dB")

output_dir = "../plots"
os.makedirs(output_dir, exist_ok=True)

plt.figure(figsize=(10,5))
plt.plot(mse_windowed)
plt.xlabel("Sample")
plt.ylabel("Mean Squared Error (MSE)")
plt.title("NLMS Convergence Curve")
plt.grid(True)
plt.savefig(os.path.join(output_dir, "nlms_convergence_curve.png"), dpi=300)
plt.show()

plt.figure(figsize=(10,4))
plt.plot(d, label='Desired (Signal + Noise)')
plt.plot(y, label='NLMS Filter Output')
plt.xlabel("Sample")
plt.ylabel("Amplitude")
plt.title("Adaptive Filter Output")
plt.legend()
plt.savefig(os.path.join(output_dir, "adaptive_filter_output.png"), dpi=300)
plt.show()

# Save the residual (speech with noise reduced)
e_int16 = np.int16(e / np.max(np.abs(e)) * 32767)
wavfile.write('filtered_output.wav', fs_d, e_int16)

# Save estimated noise
y_int16 = np.int16(y / np.max(np.abs(y)) * 32767)
wavfile.write('estimated_noise.wav', fs_d, y_int16)