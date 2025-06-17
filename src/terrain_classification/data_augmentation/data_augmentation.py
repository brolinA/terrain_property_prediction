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

    def signal_inversion(self, signal, param=None):
        """Invert the signal."""
        return -signal

    def time_warp(self, signal, warp_factor):
        """Apply non-linear time warping to the signal."""
        x = np.arange(len(signal))
        warp = np.cumsum(np.random.uniform(1 - warp_factor, 1 + warp_factor, len(signal)))
        warp = warp / warp[-1] * (len(signal) - 1)
        f = interp1d(warp, signal, kind='linear', fill_value="extrapolate")
        return f(x)

    def low_pass_filter(self, signal, cutoff=None):
        """Apply a low-pass filter to the signal."""
        fft_signal = fft(signal)
        if cutoff is None:
            cutoff = len(signal) // 2
        # Set frequencies above the cutoff to zero
        # fft_signal[int(cutoff):] = 0
        # return np.real(ifft(fft_signal))
        return np.real(fft_signal)
    
    def combine_signals(self, signals, components:dict=None):
        combined_signals = {}
        if components==None or len(components.items())==0:
            print("Cannot combine signals since components list is either empty or None.")
            return combined_signals

        for key_ in components.keys():
            if(len(components[key_])==0):
                print(f"{key_} has no components to combine. So skipping..")
                continue

            vals = components[key_]
            no_of_steps = len(signals[vals[0]]) #get the number of steps to reshape the data

            steps_to_stack = [np.array(signals[val]).flatten() for val in vals]
            combined_signals[key_] = np.column_stack(steps_to_stack).reshape(no_of_steps, -1)
        
        return combined_signals
    
    def interleave_signal (self, signals, components:list, index):
        steps_to_stack = [np.array(signals[comp][index]).flatten() for comp in components]
        return np.column_stack(steps_to_stack)
    
    def correlation_matrix(self, signals, components:list, index):
        correlation_variables = [np.array(signals[comp][index]).flatten() for comp in components]
        correlation = np.corrcoef(correlation_variables)
        #get the upper triangular matrix above the diagonal
        up_tri = np.triu(correlation, k=1)
        #get the non-zero values and flatten it.
        return up_tri[np.nonzero(up_tri)]
    
    def signal_derivate(self, signal):
        return np.gradient(signal)
    
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
        elif type == 'low_pass_filter' or type =='fft':
            return self.low_pass_filter(signal, param)
        elif type == 'derivative':
            return self.signal_derivate(signal)
        else:
            print(f"Unknown augmentation type. Given {type} but expected one of ['time_shift', 'time_scale',"
                              "'random_crop', 'pad_or_truncate', 'add_noise', 'amplitude_scale', 'signal_inversion',"
                              "'time_warp', 'low_pass_filter']")
            return []

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
    filtered_signal = augmenter.low_pass_filter(signal)

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

    # plt.subplot(3, 3, 9)
    # plt.plot(warped_signal)
    # plt.title("Time Warped Signal")

    plt.subplot(3, 3, 9)
    plt.plot(filtered_signal)
    plt.title("Filtered Signal")

    plt.tight_layout()
    plt.show()

# Run the test function
if __name__ == "__main__":
    test_data_augmentation()