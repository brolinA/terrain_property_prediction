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
   
    def extract_steps(self, normalize_data=False, legs=None, components=None, pad_length=100, 
                        combine_components=False, combine_legs = False):
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
            all_components = []
            for component in components:
                col_name = f"{leg}-{component}"
                contact_col_name = f"{leg}-contact"
                
                if not col_name in self.data.columns or not contact_col_name in self.data.columns:
                    print(f"[DataExtractor] Column '{col_name}' or '{contact_col_name}' not found in the DataFrame.")
                    continue
                
                # Extract the data for the specified leg and component
                all_components.append(self.extract_step_from_column(self.data[col_name].to_numpy(), self.data[contact_col_name], pad_length))
                if normalize_data:
                    # Normalize the data to be between 0 and 1
                    #Normalize onle the last component if you are combining components
                    for j in range(len(all_components[-1])):
                        all_components[-1][j] = pre.MinMaxScaler().fit_transform(np.array(all_components[-1][j]).reshape(-1, 1)).flatten()
                
                if not combine_components:
                    # if we are not combining components, then we need to store the steps for each component
                    self.steps[col_name] = np.array(all_components[0])
                    all_components = [] #rest the all_components list
                    
            if combine_components:
                #combine all components colomn wise
                min_len = min([len(x) for x in all_components])
                # Truncate all segments to the minimum length to do column-wise stacking
                for i in range(len(all_components)):                    
                    all_components[i] = all_components[i][:min_len]
                
                all_components = np.column_stack(all_components)
                self.steps[leg] = all_components
        
        if combine_legs:
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
        step_segment_index = np.split(non_zero_pts, np.where(np.diff(non_zero_pts) != 1)[0] + 1)

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
                if(not len(segment) == pad_length):
                    segment = self.pad_or_truncate(segment, pad_length)

                step_segments.append(np.array(contact_data[segment]))

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