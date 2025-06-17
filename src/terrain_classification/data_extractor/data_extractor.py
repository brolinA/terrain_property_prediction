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
        self.steps = {}
        self.data = pd.read_csv(file_path)

        # Strip leading/trailing spaces from column names
        self.data.columns = self.data.columns.str.strip()

        # print data types of the columns
        # print(f"[DataExtractor] Columns in the data: \n{self.data.dtypes}")
        self.preprocess_data(exclude_colunms=['time'])
   
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
   
    def extract_steps(self, normalize_data=False, components:dict=None, pad_length=100, 
                        combine_legs = False):
        """Extract the steps from the data.
        Args:
            legs (list): List of legs to extract steps from.
            components (list): List of components to extract steps from.
        """
        
        if components is None:
            print("Error: 'components' parameter is not provided. Please provide a list of component.")
            return

        for comp_key in components.keys():

            # leg = comp_key[0:2] #extracting leg name from contact name
            contact_col_name = comp_key

            for component in components[comp_key]:
                col_name = component
                
                if not col_name in self.data.columns or not contact_col_name in self.data.columns:
                    print(f"[DataExtractor] Column '{col_name}' or '{contact_col_name}' not found in the DataFrame.")
                    continue
                
                # Extract the data for the specified leg and component
                step = self.extract_step_from_column(self.data[col_name].to_numpy(), self.data[contact_col_name], pad_length)

                if normalize_data:
                    # Normalize the data to be between 0 and 1
                    #Normalize onle the last component if you are combining components
                    for j in range(len(step)):
                        step[j] = pre.MinMaxScaler().fit_transform(np.array(step[j]).reshape(-1, 1)).flatten()
         
                #we need to store the steps for each component
                self.steps[col_name] = step
        
        if combine_legs and not (len(self.steps.items())==0):
            min_stps = min([self.steps[key_].shape[0] for key_ in  self.steps.keys()])

            for key_ in self.steps.keys():
                self.steps[key_] = self.steps[key_][:min_stps]

            #convert dict to numpy array
            stps_array = np.array([self.steps[key_] for key_ in self.steps.keys()])

            #concatenate along columns
            combined_stps = np.column_stack(stps_array)

            self.steps = {}
            self.steps['leg'] = combined_stps
                   
    def pad_or_truncate(self, feature, target_length):
        """Pad or truncate a feature vector to a fixed length."""
        if len(feature) > target_length:
            return feature[:target_length]  # Truncate
        elif len(feature) < target_length:
            return np.pad(feature, (0, target_length - len(feature)), mode='constant')  # Pad with zeros
        return feature

    def extract_step_from_column(self, data_column, contact_column, pad_length):
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
        # step_segment_index = np.split(non_zero_pts, np.where(np.diff(non_zero_pts) != 1)[0] + 1)
        step_segment_index = np.split(contact_idx, np.where(np.diff(contact_idx) != 1)[0] + 1)

        # Extract the non-zero values for each segment
        step_segments = []
        for segment_id in step_segment_index:
            start_idx = segment_id[0] - 1  # Include the zero before the segment
            end_idx = segment_id[-1] + 1  # Include the zero after the segment
            
            # if start_idx >= 0 and contact_data[start_idx] == 0:  # Ensure it's a valid zero
            #     segment_id = np.insert(segment_id, 0, start_idx)
            # if end_idx < len(contact_data) and contact_data[end_idx] == 0:  # Ensure it's a valid trailing zero
            #     segment_id = np.append(segment_id, end_idx)

            seg_values = np.array(data_column[segment_id])
            #clip max values
            # peak_val = max(seg_values)
            # #set values to zero
            # peak_ids = np.where(seg_values > peak_val/2)[0]
            # seg_values[peak_ids] = 0
            if len(seg_values) > 15: #to ensure that we have enought data in the step
                if(not len(seg_values) == pad_length):
                    seg_values = self.pad_or_truncate(seg_values, pad_length)

                step_segments.append(seg_values)

        step_segments = step_segments[2:len(step_segments)-2]  # Remove the first and last segments
        return step_segments

    def plot_original_signal(self, column_name):
        """Plot the original signal.
        Args:
            column_name (str): The name of the column to plot.
        """
        # Check if the column exists in the DataFrame
        if column_name in self.data.columns:
            plt.plot(self.data[column_name])
            plt.title(column_name)
            plt.xlabel('Sample Number')
            plt.ylabel('Amplitude')
            plt.show()

    def plot_steps(self, col_name, no_of_samples=-1):
        """Plot the steps for the given column name.
        Args:
            col_name (str): The name of the column to plot.
        """
        import math

        if not col_name in self.steps:
            print(f"[DataExtractor] Cannot plot. Column '{col_name}' not found in the steps dictionary.")
            return
        if not no_of_samples == -1:
            idx = np.random.choice(range(1, len(self.steps[col_name])), size=no_of_samples, replace=False)
            steps = np.array(self.steps[col_name])[idx.tolist()]
        else:
            steps = self.steps[col_name]
        
        num_segments = len(steps)
        grid_size = math.ceil(math.sqrt(num_segments))  # n x n grid

        # Create the subplots
        if grid_size > 1:
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
        else:
            plt.plot(steps[0])
            # plt.xlabel("samples")

        # Adjust layout
        plt.suptitle(col_name)
        plt.tight_layout()
        plt.show()

        return steps #return the steps ploted

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