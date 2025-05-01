# import modules
import os
from terrain_classification.data_extractor.data_extractor import DataExtractor
from terrain_classification.wavelet_analysis.wavelet_analysis import WaveletAnalysis
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
    
    def create_feature_matrix_and_label(self, normalize_data=False, legs:list=None, components:list=None, combine_components=False):
        """Load and preprocess the data."""
        target_length = 80  # Define a fixed length for all features

        if legs is None or components is None:
            # Set default values for legs and components
            legs = ['fl']
            components = ['z']
            print(f"Using default legs: {legs} and components: {components}")

        for file_path, label in self.data_paths.items():
            print(f"Loading data from {file_path} with label {label}")
            self.data_extractor.load_data(file_path)  # Load the data
    
            # Extract steps from the data
            self.data_extractor.extract_steps(normalize_data=normalize_data,
                                            legs=legs, 
                                            components=components,
                                            combine_components=combine_components)
    
            for step in self.data_extractor.steps.values():
                wavelet_result = self.wavelet_analysis.perform_analysis(step, level=None)
                for feature in wavelet_result:
                    self.feature_matrix.append(feature.flatten())
                    self.labels.append(label)
    
        self.feature_matrix = np.array(self.feature_matrix)
        self.labels = np.array(self.labels)
        # print(f"Feature matrix shape: {self.feature_matrix.shape}")
        # print(f"Labels shape: {self.labels.shape}")
    
    def train_classifier(self, C=1, gamma=0.1, find_best_parameters=False, save_model=False, model_file_name=None,
                         save_report=False, parent_dir=None, verbose=False):
        # Split into train/test
        self.classification_report = {}
        X_train, X_test, y_train, y_test = train_test_split(self.feature_matrix, self.labels, test_size=0.2, random_state=42)
        
        if verbose:
            #check data distribution
            unique_labels, label_counts = np.unique(y_train, return_counts=True)
            for label, count in zip(unique_labels, label_counts):
                print(f"[Training] Label {label}: {count} samples")
            print(f"[Training] ratio {max(label_counts)/min(label_counts)}")

        # Feature scaling (VERY important for SVM)
        scaler = StandardScaler()
        X_train = scaler.fit_transform(X_train)
        X_test = scaler.transform(X_test)

        # Set up the SVM and parameter grid
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

        if find_best_parameters:
            print("\nFinding best parameters...")
            svc = svm.SVC(verbose=False)
            param_grid = {
                'C': [0.1, 1, 10, 100],          # Regularization parameter
                'gamma': [1, 0.1, 0.01, 0.001],  # Kernel coefficient
                'kernel': ['rbf']      # Try both RBF and Linear kernels
            }

            # Grid Search with 5-fold cross-validation
            grid = GridSearchCV(svc, param_grid, refit=True, verbose=2, cv=5, n_jobs=-1, error_score='raise')
            grid.fit(X_train, y_train) # Train

            print(f"\n[Training] Best Parameters found: {grid.best_params_}")
            
            self.classification_report['C'] = grid.best_params_['C']
            self.classification_report['gamma'] = grid.best_params_['gamma']
            y_pred = grid.predict(X_test) # Predict using the best model

            # Save the best model
            if save_model:
                best_model = grid.best_estimator_
                if model_file_name is None:
                    model_file_name = f"svm_model_c{grid.best_params_['C']}_gamma{grid.best_params_['gamma']}_{current_time}"

                model_path = os.path.join(parent_dir, "models", f"{model_file_name}.joblib")
                dump(best_model, model_path)
                print(f"[Training] Best model saved to {model_path}")

            if save_report:
                print("\n[Training] Saving GridSearchCV results...")
                # Optionally, save results to a CSV file
                import pandas as pd
                grid_results_csv = os.path.join(parent_dir, "reports", f"gridsearch_results_{current_time}.csv")
                pd.DataFrame(grid.cv_results_).to_csv(grid_results_csv, index=False)
                print(f"[Training] GridSearchCV results saved to {grid_results_csv}")

        else:
            print("\n[Training] Training SVM with fixed parameters...")
            svc = svm.SVC(C=C, gamma=gamma, kernel='rbf', class_weight='balanced')
            
            svc.fit(X_train, y_train) # Train the SVM
            y_pred = svc.predict(X_test) # Predict
            
            if save_model:
                # Save the model
                if model_file_name is None:
                    model_file_name = f"models/svm_model_c{C}_{current_time}"
                model_path = os.path.join(parent_dir, "models", f"{model_file_name}.joblib")
                dump(svc, model_path)
                print(f"Model saved to {model_path}")

            self.classification_report['C'] = C
            self.classification_report['gamma'] = gamma

        self.classification_report['report'] = classification_report(y_test, y_pred, output_dict=True)
        self.report = classification_report(y_test, y_pred, output_dict=False)
        #saving report
        if save_report: #save report
            file_name = f"svm_report_{self.classification_report['C']}_{self.classification_report['gamma']}"\
                        f"_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            report_path = os.path.join(parent_dir, "reports", file_name)
            with open(report_path, "w") as f:
                json.dump(self.classification_report, f, indent=4)

            print(f"Classification report saved to {report_path}")
            


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
                                    save_model=False, save_report=False, parent_dir=parent_dir, verbose=True)
    end_time = time.time()
    print(f"[Test fun] Training time: {end_time - st_time} seconds")

    if versbose:
        print(f"[Test fun] Classification report: {svm_classifier.classification_report['report']}")

if __name__ == "__main__":
    run_classification_test()