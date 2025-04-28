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

class SVMClassification:
    def __init__(self, data_paths):
        self.data_paths = data_paths
        # self.wavelet_type = wavelet_type
        self.data_extractor = DataExtractor()
        self.wavelet_analysis = WaveletAnalysis(wavelet_type='db4')
        self.feature_matrix = []
        self.labels = []

    def pad_or_truncate(self, feature, target_length):
        """Pad or truncate a feature vector to a fixed length."""
        if len(feature) > target_length:
            return feature[:target_length]  # Truncate
        elif len(feature) < target_length:
            return np.pad(feature, (0, target_length - len(feature)), mode='constant')  # Pad with zeros
        return feature
    
    def prepare_data(self, normalize_data=False, legs:list=None, components:list=None):
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
            self.data_extractor.preprocess_data()  # Preprocess the data
    
            # Extract steps from the data
            # legs = ['fl', 'fr', 'rl', 'rr']
            # components = ['x', 'y', 'z']
            self.data_extractor.extract_steps(normalize_data=normalize_data,
                                            legs=legs, 
                                            components=components)
    
            for step in self.data_extractor.steps.values():
                wavelet_result = self.wavelet_analysis.perform_analysis(step, level=2)
                for feature in wavelet_result:
                    padded_feature = self.pad_or_truncate(feature.flatten(), target_length)
                    self.feature_matrix.append(padded_feature)
                    self.labels.append(label)
    
        self.feature_matrix = np.array(self.feature_matrix)
        self.labels = np.array(self.labels)
        # print(f"Feature matrix shape: {self.feature_matrix.shape}")
        # print(f"Labels shape: {self.labels.shape}")

def run_classification_test():
    # Sample data paths for testing
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    data_labels = {
        os.path.join(parent_dir,'data/sand/trial1.csv'): 'sand',
        os.path.join(parent_dir,'data/sand/trial2.csv'): 'sand',
        os.path.join(parent_dir,'data/sand/trial3.csv'): 'sand',
        os.path.join(parent_dir,'data/sand/trial4.csv'): 'sand',
        os.path.join(parent_dir,'data/concrete/trial1.csv'): 'concrete',
        os.path.join(parent_dir,'data/concrete/trial2.csv'): 'concrete',
        os.path.join(parent_dir,'data/gravel/trial1.csv'): 'gravel'
    }

    # Create an instance of the SVMClassification class
    svm_classifier = SVMClassification(data_labels)

    # Load the data
    svm_classifier.prepare_data(normalize_data=True,
                                legs=['fl', 'fr', 'rl', 'rr'], 
                                components=['x', 'y', 'z'])
    # svm_classifier.data_extractor.plot_steps('fl-x')  # Plot the steps for the 'fl-z' column

    # Split into train/test
    X_train, X_test, y_train, y_test = train_test_split(svm_classifier.feature_matrix, svm_classifier.labels, test_size=0.2, random_state=42)
    # print(f"X_train shape: {X_train.shape}, y_train shape: {y_train.shape}")
    
    # Feature scaling (VERY important for SVM)
    scaler = StandardScaler()
    X_train = scaler.fit_transform(X_train)
    X_test = scaler.transform(X_test)

    find_best_parameters = False
    # Set up the SVM and parameter grid
    
    if find_best_parameters:
        print("\nFinding best parameters...")
        svc = svm.SVC(verbose=True)
        param_grid = {
            'C': [0.1, 1, 10],          # Regularization parameter
            # 'gamma': [1, 0.1],  # Kernel coefficient
            'gamma': [1, 0.1, 0.01, 0.001],  # Kernel coefficient
            'kernel': ['rbf']      # Try both RBF and Linear kernels
            # 'kernel': ['rbf', 'linear']      # Try both RBF and Linear kernels
        }

        # Grid Search with 5-fold cross-validation
        grid = GridSearchCV(svc, param_grid, refit=True, verbose=2, cv=5, n_jobs=-1, error_score='raise')
        grid.fit(X_train, y_train) # Train

        print(f"\nBest Parameters found: {grid.best_params_}")
        y_pred = grid.predict(X_test) # Predict using the best model

    else:
        print("\nTraining SVM with fixed parameters...")
        svc = svm.SVC(C=10, gamma=0.1, kernel='rbf', verbose=True)
        # Train the SVM
        svc.fit(X_train, y_train)

        # Predict
        y_pred = svc.predict(X_test)

    # Evaluate
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))

if __name__ == "__main__":
    run_classification_test()