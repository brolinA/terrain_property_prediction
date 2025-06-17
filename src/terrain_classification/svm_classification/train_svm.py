# import modules
from copy import deepcopy
import os
from terrain_classification.data_extractor.data_extractor import DataExtractor
from terrain_classification.wavelet_analysis.wavelet_analysis import WaveletAnalysis
from terrain_classification.data_augmentation.data_augmentation import DataAugmentation
#import svm realted modules
import numpy as np
from sklearn import svm
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, accuracy_score
from joblib import dump
from datetime import datetime
import time
import json
from sklearn.metrics import ConfusionMatrixDisplay

class SVMClassification:
    def __init__(self, data_paths):
        self.data_paths = data_paths
        # self.wavelet_type = wavelet_type
        self.data_extractor = DataExtractor()
        self.wavelet_analysis = WaveletAnalysis(wavelet_type='db4')
        self.feature_matrix = []
        self.labels = []
        self.classification_report = {}
        self.report = None
        self.latest_model = None
        self.scaler = None
        self.data_augmentation = DataAugmentation()
        self.original_data = []
    
    def create_feature_matrix_and_label(self, normalize_data=False, components:dict=None, combine_legs = False, 
                                        data_padding_size= 100, use_original_signal=False, 
                                        augmetation_params={'wavelet': None}):
        """Load and preprocess the data."""

        # print(f"Applying the following augmentations: {augmetation_params}")
        if len(augmetation_params) == 0 and not use_original_signal:
            print("No augmentation provided and original signal is not used. So cannot create feature matrix")
            return np.empty([])
        
        for file_path, label in self.data_paths.items():
            # print(f"Loading data from {file_path} with label {label}")
            self.data_extractor.load_data(file_path)  # Load the data
    
            # Extract steps from the data
            self.data_extractor.extract_steps(normalize_data=normalize_data,
                                            components=components,
                                            combine_legs=combine_legs,
                                            pad_length=data_padding_size)
            #run through every setp signal and apply the given augmentations to create the feature matrix
            for component_key_ in components.keys():
                leg_components = components[component_key_]
                for step_number in range (len(leg_components[0])): #iterate through individual step signal 
                    augmented_signal = []  
                    curr_augmentations = deepcopy(augmetation_params)
                    if use_original_signal: #compute the interleaved signal
                        interleave_ = self.data_augmentation.interleave_signal(self.data_extractor.steps, leg_components, step_number )
                        augmented_signal = np.hstack((augmented_signal, interleave_.flatten()))
                        self.original_data.append(augmented_signal) #save the original signal if required later

                    #correlation is computed for the whole signal. So can't be run for individual leg data
                    if "correlation" in curr_augmentations:
                        correlation_ = self.data_augmentation.correlation_matrix(self.data_extractor.steps, leg_components, step_number )
                        augmented_signal = np.hstack((augmented_signal, correlation_.tolist()))
                        del curr_augmentations['correlation'] #remove before proceeding so that this augmentation is not applied for the next step

                    #run though each leg components and compute the other augmentations.
                    for leg_component in leg_components: # for each component
                        for augmentation_type in curr_augmentations.keys(): #apply each augmentation
                            augmentation = []
                            signal = self.data_extractor.steps[leg_component][step_number]
                            
                            if augmentation_type == 'wavelet':
                                augmentation = self.wavelet_analysis.extract_details(signal, curr_augmentations[augmentation_type])
                            # elif augmentation_type == 'derivative':
                            #     #only compute derivates for joint torques
                            #     torque_list = ['hip', 'calf', 'tigh']
                            #     for torque_val in torque_list:
                            #         if torque_val in leg_component: #check if the word ['hip', 'calf', 'tigh'] appear in the list of compoenents
                            #             augmentation = self.data_augmentation.augment_signal(signal, augmentation_type, curr_augmentations[augmentation_type])
                            else:
                                augmentation = self.data_augmentation.augment_signal(signal, augmentation_type, curr_augmentations[augmentation_type])

                            #stack the augmentations horizontally next to each other
                            if not len(augmentation)==0:
                                augmented_signal = np.hstack((augmented_signal, augmentation.flatten()))
                    
                    #once all augmentation are done add it to feature matrix
                    self.feature_matrix.append(augmented_signal)
                    self.labels.append(label) 
    
        self.feature_matrix = np.array(self.feature_matrix)
        self.labels = np.array(self.labels)
        # print(f"Feature matrix shape: {self.feature_matrix.shape}")
        # print(f"Labels shape: {self.labels.shape}")
    
    def save_original_data(self, steps):
        """Save the original data for later use."""
        for step in steps:
            self.original_data.append(step)

    def augment_data(self, signals, augmentation_types, augment_params):
        """Augment the data using the specified augmentation type."""
        augmented_signals = []

        for augmentation_type, param in zip(augmentation_types, augment_params):
            for signal in signals:
                augmented_signal = self.data_augmentation.augment_signal(signal, augmentation_type, param)
                augmented_signals.append(augmented_signal)
        
        #append augmented signal to original signal
        return_signal = np.append(signals, augmented_signals, axis=0)
        return return_signal
    
    def train_classifier(self, C=[1], gamma=[0.1], find_best_parameters=False, save_model=False, model_file_name=None,
                         save_report=False, file_path=None, verbose=False):
        # Split into train/test
        self.classification_report = {}
        X_train, X_test, y_train, y_test = train_test_split(self.feature_matrix, self.labels, test_size=0.2, random_state=42)
        if verbose:
            print(f"[Training] X_train shape: {X_train.shape}, X_test shape: {X_test.shape}")
            #check data distribution
            unique_labels, label_counts = np.unique(y_train, return_counts=True)
            for label, count in zip(unique_labels, label_counts):
                print(f"[Training] Label {label}: {count} samples")
            print(f"[Training] ratio {max(label_counts)/min(label_counts)}")

        # Feature scaling (VERY important for SVM)
        scaler = StandardScaler()
        X_train = scaler.fit_transform(X_train)
        X_test = scaler.transform(X_test)
        self.scaler = scaler # Save the scaler for later use

        # Set up the SVM and parameter grid
        if find_best_parameters:
            if verbose:
                print("\nFinding best parameters...")
            svc = svm.SVC(verbose=False, class_weight='balanced', probability=True)
            param_grid = {
                'C': C,          # Regularization parameter
                'gamma': gamma,  # Kernel coefficient
                'kernel': ['rbf']      # Try both RBF and Linear kernels
            }

            # Grid Search with 5-fold cross-validation
            grid = GridSearchCV(svc, param_grid, refit=True, verbose=0, cv=10, n_jobs=-1, error_score='raise')
            grid.fit(X_train, y_train) # Train
            self.latest_model = grid.best_estimator_ # Save the best model

            print(f"\n[Training] Best Parameters found: {grid.best_params_}")
            
            self.classification_report['C'] = grid.best_params_['C']
            self.classification_report['gamma'] = grid.best_params_['gamma']
            y_pred = grid.predict(X_test) # Predict using the best model

        else:
            print("\n[Training] Training SVM with fixed parameters...")
            svc = svm.SVC(C=C[0], gamma=gamma[0], kernel='rbf', class_weight='balanced', probability=True)
            X_train = self.feature_matrix
            y_train = self.labels
            scaler = StandardScaler()
            X_train = scaler.fit_transform(X_train)
            self.scaler = scaler # Save the scaler for later use

            svc.fit(X_train, y_train) # Train the SVM
            self.latest_model = svc # Save the model

            self.classification_report['C'] = C
            self.classification_report['gamma'] = gamma

        self.classification_report['report'] = classification_report(y_test, y_pred, output_dict=True)
        self.report = classification_report(y_test, y_pred, output_dict=False)
        # ConfusionMatrixDisplay.from_predictions(y_test, y_pred, display_labels=self.latest_model.classes_, cmap='Blues')
        #saving report
        if save_report: #save report
            self.save_report(name=model_file_name, file_path=file_path)
        
        if save_model:
            self.save_model(name=model_file_name, file_path=file_path)

    def print_classification_report(self):
        """Print the classification report."""
        if self.report is not None:
            print(self.report)
        else:
            print("No classification report available. Please run the classifier first.")
    
    def save_report(self, name=None, file_path=None):
        """Save the classification report to a file."""
        if self.classification_report is None:
            print("No classification report available to save. Run the classifier first.")
            return
        
        if name is None:
            name = f"svm_report_{self.classification_report['C']}_{self.classification_report['gamma']}"\
                        f"_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            
        if file_path is None:
            file_path = os.path.abspath(os.path.join(os.path.dirname(__file__)))
            file_path = os.path.join(file_path, "reports", f"{name}.json")
        else:
            file_path = os.path.join(file_path, f"{name}.json")
        # check if path exists
        if not os.path.exists(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        # save report
        with open(file_path, "w") as f:
            json.dump(self.classification_report, f, indent=4)

        print(f"Classification report saved to {file_path}")
    
    def save_model(self, name=None, file_path=None):
        """Save the model to a file."""
        if self.latest_model is None:
            print("No model available to save. Run the classifier first.")
            return
        
        if name is None:
            name = f"svm_model_{self.classification_report['C']}_{self.classification_report['gamma']}"

        if file_path is None:
            parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__)))
            file_path = os.path.join(parent_dir, "models", f"{name}.joblib")
            scaler_path = os.path.join(parent_dir, "models", f"{name}-scaler.pkl")
        else:
            file_path = os.path.join(file_path, f"{name}.joblib")
            scaler_path = os.path.join(file_path, f"{name}-scaler.pkl")

        # check if path exists
        if not os.path.exists(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        # save model
        dump(self.latest_model, file_path)
        dump(self.scaler, scaler_path)
        print(f"Model saved to {file_path}")

def run_classification_test():
    # Sample data paths for testing
    parent_dir = os.path.dirname(__file__)
    data_dir = os.path.abspath(os.path.join(parent_dir, '..'))
    save_report = False
    versbose = False
    
    files_to_use = {
        "sand": ["trial1.csv", "trial2.csv", "trial3.csv", "trial4.csv", "trial5.csv", "trial6.csv", "trial7.csv", "trial8.csv"],
        "concrete": ["trial1.csv", "trial2.csv", "trial3.csv", "trial4.csv", "trial5.csv", "trial6.csv", "trial7.csv", "trial8.csv"],
        "gravel": ["trial1.csv", "trial2.csv", "trial3.csv", "trial4.csv", "trial5.csv", "trial6.csv", "trial7.csv", "trial8.csv"],
    }
    #create a dictionary to hold the data file and its label
    data_labels = {}
    for key_ in files_to_use.keys():
        for file_ in files_to_use[key_]:
            data_labels[os.path.join(data_dir, 'data', key_, file_)] = key_

    # Create an instance of the SVMClassification class
    svm_classifier = SVMClassification(data_labels)

    # Load the data
    svm_classifier.create_feature_matrix_and_label(normalize_data=True,
                                legs=['fl', 'fr', 'rl', 'rr'], 
                                components=['x','y','z'])

    if versbose:
        unique_labels, label_counts = np.unique(svm_classifier.labels, return_counts=True)
        for label, count in zip(unique_labels, label_counts):
            print(f"[Test fun] Label {label}: {count} samples")
        print(f"[Test fun] ratio {max(label_counts)/min(label_counts)}")
    
    # Train the classifier
    st_time = time.time()
    svm_classifier.train_classifier(C=10, gamma=0.01, find_best_parameters=False, 
                                    save_model=False, save_report=save_report, parent_dir=parent_dir, verbose=True)
    end_time = time.time()
    print(f"[Test fun] Training time: {end_time - st_time} seconds")

    if versbose:
        print(f"[Test fun] Classification report: {svm_classifier.classification_report['report']}")

if __name__ == "__main__":
    run_classification_test()