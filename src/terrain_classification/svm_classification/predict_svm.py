#### ! /home/isaac/miniconda3/bin/python
import os
import joblib
# import cuml

class SVMPredictor:
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = None
        if os.path.exists(self.model_path):
            self.model = joblib.load(model_path)
        else:
            raise FileNotFoundError(f"Model file not found at {model_path}")

        print(f"Model loaded from {model_path}")

    def predict(self, data):
        return self.model.predict(data)

def test_predictor():
    # Sample data for testing
    # test_data = [
    #     [0.5, 0.2, 0.1],
    #     [0.3, 0.4, 0.6],
    #     [0.8, 0.7, 0.9]
    # ]

    # Path to the saved model
    parent_dir = os.path.dirname(__file__)
    model_name = 'test_model.joblib'
    model_path = os.path.abspath(os.path.join(parent_dir, model_name))
    # model_path = 'path/to/your/saved_model.joblib'

    # Create an instance of SVMPredictor
    predictor = SVMPredictor(model_path)

    # Make predictions
    # predictions = predictor.predict(test_data)
    # print("Predictions:", predictions)

if __name__ == "__main__":
    test_predictor()
    # Add any additional test cases or functionality here