import numpy as np
from scipy.interpolate import interp1d
from scipy.fft import fft, ifft
import matplotlib.pyplot as plt

class DataAugmentation:
    def time_shift(self, signal, shift):
        """Shift the signal forward or backward in time."""
        return np.roll(signal, shift)

    def time_scale(self, signal, scale_factor):
        """Stretch or compress the signal in time."""
        x = np.arange(len(signal))
        f = interp1d(x, signal, kind='linear')
        x_new = np.linspace(0, len(signal) - 1, int(len(signal) * scale_factor))
        return f(x_new)

    def random_crop(self, signal, crop_length):
        """Extract a random segment of the signal."""
        start = np.random.randint(0, len(signal) - crop_length)
        return signal[start:start + crop_length]

    def pad_or_truncate(self, signal, target_length):
        """Pad or truncate the signal to a fixed length."""
        if len(signal) > target_length:
            return signal[:target_length]
        else:
            return np.pad(signal, (0, target_length - len(signal)), mode='constant')

    def add_noise(self, signal, noise_level=0.01):
        """Add Gaussian noise to the signal."""
        noise = np.random.normal(0, noise_level, len(signal))
        return signal + noise

    def amplitude_scale(self, signal, scale_factor):
        """Scale the amplitude of the signal."""
        return signal * scale_factor

    def signal_inversion(self, signal):
        """Invert the signal."""
        return -signal

    def time_warp(self, signal, warp_factor):
        """Apply non-linear time warping to the signal."""
        x = np.arange(len(signal))
        warp = np.cumsum(np.random.uniform(1 - warp_factor, 1 + warp_factor, len(signal)))
        warp = warp / warp[-1] * (len(signal) - 1)
        f = interp1d(warp, signal, kind='linear', fill_value="extrapolate")
        return f(x)

    def low_pass_filter(self, signal, cutoff):
        """Apply a low-pass filter to the signal."""
        fft_signal = fft(signal)
        fft_signal[int(cutoff):] = 0
        return np.real(ifft(fft_signal))
        # return np.real(fft_signal)

    def interpolate_signals(self, signal1, signal2, alpha=0.5):
        """Interpolate between two signals."""
        return alpha * signal1 + (1 - alpha) * signal2
    
    def augment_signal(self, signal, type, param):
        """Augment the signal based on the specified type and parameters."""
        if type == 'time_shift':
            return self.time_shift(signal, param)
        elif type == 'time_scale':
            return self.time_scale(signal, param)
        elif type == 'random_crop':
            return self.random_crop(signal, param)
        elif type == 'pad_or_truncate':
            return self.pad_or_truncate(signal, param)
        elif type == 'add_noise':
            return self.add_noise(signal, param)
        elif type == 'amplitude_scale':
            return self.amplitude_scale(signal, param)
        elif type == 'signal_inversion':
            return self.signal_inversion(signal)
        elif type == 'time_warp':
            return self.time_warp(signal, param)
        elif type == 'low_pass_filter':
            return self.low_pass_filter(signal, param)
        else:
            raise ValueError("Unknown augmentation type. Given {type} but expected one of ['time_shift', 'time_scale',\
                              'random_crop', 'pad_or_truncate', 'add_noise', 'amplitude_scale', 'signal_inversion',\
                              'time_warp', 'low_pass_filter']")

def test_data_augmentation():
    # Create a sample signal
    t = np.linspace(0, 1, 500)
    signal = np.sin(2 * np.pi * 5 * t) + 0.5 * np.sin(2 * np.pi * 10 * t)

    # Create an instance of the DataAugmentation class
    augmenter = DataAugmentation()

    # Apply augmentation methods
    shifted_signal = augmenter.time_shift(signal, shift=50)
    scaled_signal = augmenter.time_scale(signal, scale_factor=1.5)
    cropped_signal = augmenter.random_crop(signal, crop_length=200)
    padded_signal = augmenter.pad_or_truncate(signal, target_length=600)
    noisy_signal = augmenter.add_noise(signal, noise_level=0.1)
    scaled_amplitude_signal = augmenter.amplitude_scale(signal, scale_factor=1.5)
    inverted_signal = augmenter.signal_inversion(signal)
    warped_signal = augmenter.time_warp(signal, warp_factor=0.8)
    filtered_signal = augmenter.low_pass_filter(signal, cutoff=10)

    # Plot the original and augmented signals
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 3, 1)
    plt.plot(signal)
    plt.title("Original Signal")

    plt.subplot(3, 3, 2)
    plt.plot(shifted_signal)
    plt.title("Time Shifted Signal")

    plt.subplot(3, 3, 3)
    plt.plot(scaled_signal)
    plt.title("Time Scaled Signal")

    plt.subplot(3, 3, 4)
    plt.plot(cropped_signal)
    plt.title("Random Cropped Signal")

    plt.subplot(3, 3, 5)
    plt.plot(padded_signal)
    plt.title("Padded/Truncated Signal")

    plt.subplot(3, 3, 6)
    plt.plot(noisy_signal)
    plt.title("Noisy Signal")

    plt.subplot(3, 3, 7)
    plt.plot(scaled_amplitude_signal)
    plt.title("Amplitude Scaled Signal")

    plt.subplot(3, 3, 8)
    plt.plot(inverted_signal)
    plt.title("Inverted Signal")

    plt.subplot(3, 3, 9)
    plt.plot(warped_signal)
    plt.title("Time Warped Signal")

    # plt.subplot(3, 3, 9)
    # plt.plot(filtered_signal)
    # plt.title("Filtered Signal")

    plt.tight_layout()
    plt.show()

# Run the test function
if __name__ == "__main__":
    test_data_augmentation()