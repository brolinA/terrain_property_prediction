import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from sklearn import preprocessing as pre

class DataExtractor:
    def __init__(self):
        self.file_path = None
        self.data = None
        self.steps = {} #dict to store steps for each leg

    def load_data(self, file_path):
        """Load data from the CSV file and preprocess it."""
        self.data = pd.read_csv(file_path)

        # Strip leading/trailing spaces from column names
        self.data.columns = self.data.columns.str.strip()

        # print data types of the columns
        # print(f"[DataExtractor] Columns in the data: \n{self.data.dtypes}")
   
    def preprocess_data(self, exclude_colunms=None, convert_to_numeric=True):
        """Preprocess the data by removing unnecessary columns and converting types.
        
        Args:
            exclude_colunms (list): List of columns to exclude from the data.
            convert_to_numeric (bool): Whether to convert columns to numeric types.
        """

        if exclude_colunms is None:
            exclude_colunms = ['time', 'time-stamp', 'time-stamp-2']
        # Remove specified columns
        for col in exclude_colunms:
            # if col in self.data.columns:
            #     self.data = self.data.drop(columns=[col])  
            # Remove columns with 'time' in their name
            self.data = self.data[self.data.columns.drop(list(self.data.filter(regex=col)))]
        
        if convert_to_numeric:
            # Convert specified columns to numeric, coercing errors to NaN
            for col in self.data.columns:
                self.data[col] = pd.to_numeric(self.data[col], errors='coerce')

        # convert float columns to 3 decimal places
        for col in self.data.columns:
            if self.data[col].dtype == 'float64':
                self.data[col] = self.data[col].round(3)
   
    def extract_steps(self, normalize_data=False, legs=None, components=None):
        """Extract the steps from the data.
        Args:
            legs (list): List of legs to extract steps from.
            components (list): List of components to extract steps from.
        """

        if legs is None:
            #print error message and exit
            print("Error: 'legs' parameter is not provided. Please provide a list of legs.")
            return
        
        if components is None:
            print("Error: 'components' parameter is not provided. Please provide a list of component.")
            return

        for leg in legs:
            for component in components:
                col_name = f"{leg}-{component}"
                contact_col_name = f"{leg}-contact"
                
                if not col_name in self.data.columns or not contact_col_name in self.data.columns:
                    print(f"[DataExtractor] Column '{col_name}' or '{contact_col_name}' not found in the DataFrame.")
                    continue
                
                # Extract the data for the specified leg and component
                self.steps[col_name] = self.extract_step_from_column(self.data[col_name].to_numpy(), self.data[contact_col_name])

                if normalize_data:
                    # Normalize the data to be between 0 and 1
                    for i, step in enumerate(self.steps[col_name]):
                        # Normalize each step segment
                        self.steps[col_name][i] = pre.MinMaxScaler().fit_transform(step.reshape(-1, 1)).flatten()
                
                
    def extract_step_from_column(self, data_column, contact_column):
        """Extract steps from given data column
        Args:
            data_column (numpy array): The data column to extract steps from.
            contact_column (numpy array): The contact column to determine step segments.
        Returns:
            list: A list of segments, each containing the data for a step.
        """
        # Find indices where the foots are in contact
        contact_idx = np.where(contact_column == 1)[0]
        # Extract the data corresponding to contact data
        contact_data = data_column[contact_idx]
        
        non_zero_pts = np.where(contact_data != 0)[0]
        # Split the array into segments of consecutive non-zero values
        step_segment_index = np.split(non_zero_pts, np.where(np.diff(non_zero_pts) != 1)[0] + 1)
        # print(f"[DataExtractor] Step segment index: {step_segment_index}")

        # Extract the non-zero values for each segment
        step_segments = []
        for segment in step_segment_index:
            start_idx = segment[0] - 1  # Include the zero before the segment
            end_idx = segment[-1] + 1  # Include the zero after the segment
            
            if start_idx >= 0 and contact_data[start_idx] == 0:  # Ensure it's a valid zero
                segment = np.insert(segment, 0, start_idx)
            if end_idx < len(contact_data) and contact_data[end_idx] == 0:  # Ensure it's a valid trailing zero
                segment = np.append(segment, end_idx)
            
            if len(segment) > 15: #to ensure that we have enought data in the step
                step_segments.append(contact_data[segment])

        step_segments = step_segments[1:len(step_segments)-1]  # Remove the first and last segments
        return step_segments

    def plot_original_signal(self, column_name):
        """Plot the original signal.
        Args:
            column_name (str): The name of the column to plot.
        """
        # Check if the column exists in the DataFrame
        if column_name in self.data.columns:
            plt.plot(self.data[column_name])
            plt.title('Original Signal')
            plt.xlabel('Sample Number')
            plt.ylabel('Amplitude')
            plt.show()

    def plot_steps(self, col_name):
        """Plot the steps for the given column name.
        Args:
            col_name (str): The name of the column to plot.
        """
        import math

        if not col_name in self.steps:
            print(f"[DataExtractor] Cannot plot. Column '{col_name}' not found in the steps dictionary.")
            return
        steps = self.steps[col_name]
        num_segments = len(steps)
        grid_size = math.ceil(math.sqrt(num_segments))  # n x n grid

        # Create the subplots
        fig, axes = plt.subplots(grid_size, grid_size, figsize=(10, 10))
        axes = axes.flatten()  # Flatten the 2D array of axes for easy iteration

        # Loop through the segments and plot each one
        for i, segment in enumerate(steps):
            axes[i].plot(segment)
            axes[i].set_title(f"Segment {i + 1}")
            axes[i].grid(True)

        # Hide any unused subplots
        for j in range(num_segments, len(axes)):
            axes[j].axis('off')

        # Adjust layout
        plt.tight_layout()
        plt.show()

def run_data_extractor():
    """Run the data extraction process."""
    # Example usage
    # find path to the parent directory of this file
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    file_path = os.path.join(parent_dir, 'data/sand/trial4.csv')
    # print(f"[DataExtractor] File path: {file_path}")

    data_extractor = DataExtractor()
    data_extractor.load_data(file_path)
    data_extractor.preprocess_data(exclude_colunms=['time'])
    data_extractor.extract_steps(normalize_data=False, legs=['fl', 'rl'], components=['x', 'y', 'z'])
    # data_extractor.plot_original_signal('fl-z')
    data_extractor.plot_steps('fl-x')
    # data_extractor.extract_steps(normalize_data=True, legs=['fl', 'rl'], components=['x', 'y', 'z'])
    # data_extractor.plot_steps('fl-x')
    plt.show()

if __name__ == "__main__":

    run_data_extractor()