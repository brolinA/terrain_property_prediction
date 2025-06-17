import numpy as np
# import pandas as pd
import matplotlib.pyplot as plt
import pywt
import os
import math
from terrain_classification.data_extractor.data_extractor import DataExtractor

class WaveletAnalysis:
    def __init__(self, wavelet_type='db4'):
        self.wavelet_type = wavelet_type
        self.input_signal = None
        self.wavelet_results = []

    def perform_analysis(self, input_signals, level=None):
        """Perform wavelet analysis on the input signals."""
        self.input_signal = input_signals
        self.wavelet_results = []  # Reset results for each analysis
        for signal in input_signals:
            extracted_details = self.extract_details(signal, level=level)
            self.wavelet_results.append(extracted_details)

        return self.wavelet_results

    def extract_details(self, signal, level=None):
        """Perform wavelet analysis on each segment."""
        # wavelet_type = 'db4'
        # Perform Discrete Wavelet Decomposition
        if level is None:
            max_level = pywt.dwt_max_level(len(signal), pywt.Wavelet(self.wavelet_type).dec_len)
        else:
            max_level = level

        coeffs = pywt.wavedec(signal, self.wavelet_type, level=max_level)

        # Reconstruct detail coefficients at each level
        details = []
        for i in range(1, len(coeffs)):
            coeff_list = [np.zeros_like(c) if j != i else coeffs[j] for j, c in enumerate(coeffs)]
            detail = pywt.waverec(coeff_list, self.wavelet_type)
            # details.append(np.abs(detail[:len(signal)]))  # Ensure equal length
            details.append(np.abs(detail))  # Ensure equal length
        # self.wavelet_results.append(np.vstack(details))  # Store the scalogram-like array

        return np.vstack(details)  # Return the scalogram-like array
    

    def plot_wavelet(self, wavelets, title="wavelet_results"):
        """Plot the wavelet results in an n x n grid."""
        num_segments = len(wavelets)
        grid_size = math.ceil(math.sqrt(num_segments))  # Calculate grid size

        # Create subplots
        if not grid_size == 1:
            fig, axes = plt.subplots(grid_size, grid_size, figsize=(10, 10))
            axes = axes.flatten()  # Flatten the 2D array of axes for easy iteration

            # Loop through the wavelet results and plot each one
            for i, scalogram_array in enumerate(wavelets):
                extent = [0, len(self.input_signal[0]) if self.input_signal is not None else 0, 1, scalogram_array.shape[0]]
                axes[i].imshow(scalogram_array, extent=extent, aspect='auto', cmap='jet', origin='lower')
                axes[i].set_title(f"Segment {i + 1}")
                axes[i].set_xlabel('Time')
                axes[i].set_ylabel('Level')
                axes[i].set_yticks(np.arange(1, scalogram_array.shape[0] + 1))  # Set integer y-ticks

            # Hide any unused subplots
            for j in range(num_segments, len(axes)):
                axes[j].axis('off')
        else:
            plt.figure(figsize=(10,10))
            plt.imshow(wavelets[0], aspect='auto', cmap='jet', origin='lower')
            plt.title("Wavelet")
            plt.xlabel("Time")
            plt.ylabel("Level")
            plt.yticks(np.arange(1, len(wavelets) + 1))

        # Adjust layout
        plt.suptitle(title)
        plt.tight_layout()
        plt.show()

    def plot_results(self, title="results"):
        self.plot_wavelet(self.wavelet_results, title)

    def plot_signals(self):
        if not self.input_signal == None:
            num_segments = len(self.input_signal)
            grid_size = math.ceil(math.sqrt(num_segments))  # Calculate grid size

            # Create subplots
            if not grid_size == 1:
                fig, axes = plt.subplots(grid_size, grid_size, figsize=(5, 5))
                axes = axes.flatten()  # Flatten the 2D array of axes for easy iteration

                # Loop through the wavelet results and plot each one
                for i, sig in enumerate(self.input_signal):
                    axes[i].plot(sig)
                    axes[i].set_title(f"Signal {i + 1}")
                    axes[i].set_xlabel('Time')
                    axes[i].set_ylabel('Amplitude')

                # Hide any unused subplots
                for j in range(num_segments, len(axes)):
                    axes[j].axis('off')
            else:
                plt.figure(figsize=(5,5))
                plt.plot(self.input_signal)
                plt.title("Input signal")
                plt.xlabel("Time")
                plt.ylabel("Amplitude")

            # Adjust layout
            plt.tight_layout()
            plt.show()

def run_WaveletAnalysis():
    # sample signal for testing
    t = np.linspace(0, 1, 49)
    signal1 = np.sin(2 * np.pi * 50 * t) + np.sin(2 * np.pi * 150 * t)
    signal2 = np.cos(2 * np.pi * 50 * t) + np.cos(2 * np.pi * 150 * t)
    
    input_signals = [signal1]  # List of input signals
    input_signals.append(signal2)  # Append the same signal for testing
    
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    file_path = os.path.join(parent_dir, 'data/sand/trial4.csv')
    
    data_extractor = DataExtractor()
    data_extractor.load_data(file_path)
    data_extractor.extract_steps(legs=['fl', 'rl'], components=['x', 'y', 'z'])
    # data_extractor.plot_steps('fl-z')

    # Create an instance of the WaveletAnalysis class
    wavelet_analysis = WaveletAnalysis("db4")

    # # Perform wavelet analysis
    wavelet_results = wavelet_analysis.perform_analysis(data_extractor.steps['fl-z'])
    # wavelet_results = wavelet_analysis.perform_analysis(input_signals)

    # # Plot the results
    wavelet_analysis.plot_results()

if __name__ == "__main__":
    run_WaveletAnalysis()