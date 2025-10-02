import numpy as np
from scipy.io import wavfile

clean_file = '../audio/clean_audio.wav' 
noise_file = '../audio/noise.wav'        
desired_output = '../audio/desired_signal.wav'
reference_output = '../audio/reference_noise.wav'

fs_clean, clean = wavfile.read(clean_file)
fs_noise, noise = wavfile.read(noise_file)

# Ensure mono audio
if clean.ndim > 1:
    clean = clean[:, 0]
if noise.ndim > 1:
    noise = noise[:, 0]

# Normalize to float32 in [-1, 1]
clean = clean.astype(np.float32)
clean /= np.max(np.abs(clean))

noise = noise.astype(np.float32)
noise /= np.max(np.abs(noise))

min_len = min(len(clean), len(noise))
clean = clean[:min_len]
noise = noise[:min_len]

#adjust so that reference_noise.wav and noise.wav have similar power levels
noise_amplitude = 0.25

# Create desired signal by overlaying noise
desired_signal = clean + noise * noise_amplitude

desired_signal /= np.max(np.abs(desired_signal))

reference_noise = noise_amplitude * noise

# Convert to 16-bit PCM
wavfile.write(desired_output, fs_clean, np.int16(desired_signal * 32767))
wavfile.write(reference_output, fs_clean, np.int16(reference_noise * 32767))

print(f"Saved desired_signal.wav ({desired_output}) and reference_noise.wav ({reference_output})")
print(f"Sample rate: {fs_clean} Hz, length: {min_len} samples")