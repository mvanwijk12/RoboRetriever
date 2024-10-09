import torch

# Check if a GPU is available, else use CPU
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


# Linear regression model
class LinearRegressionTorch:
    def __init__(self):
        self.coef_ = torch.tensor(0., requires_grad=True).to(device)
        self.intercept_ = torch.tensor(0., requires_grad=True).to(device)

    def fit(self, X, y):
        X_mean = torch.mean(X)
        y_mean = torch.mean(y)
        self.coef_ = torch.sum((X - X_mean) * (y - y_mean)) / torch.sum((X - X_mean)**2)
        self.intercept_ = y_mean - self.coef_ * X_mean

    def predict(self, X):
        return self.coef_ * X + self.intercept_

# RANSAC implementation with PyTorch
def ransac_fit(X, y, base_model, min_samples, residual_threshold, max_trials=50):
    best_inlier_count = 0
    best_model = None
    best_inlier_mask = None
    
    for _ in range(max_trials):
        # Randomly select min_samples points for fitting
        subset_idx = torch.randperm(len(X))[:min_samples].to(device)
        X_subset, y_subset = X[subset_idx], y[subset_idx]

        # Fit model on the subset
        base_model.fit(X_subset, y_subset)

        # Calculate residuals
        y_pred = base_model.predict(X)
        residuals = torch.abs(y_pred - y)

        # Determine inliers (points with residuals below the threshold)
        inlier_mask = residuals < residual_threshold
        inlier_count = torch.sum(inlier_mask).item()

        # Update the best model if current inliers are better
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_model = LinearRegressionTorch()
            best_model.coef_ = base_model.coef_.clone().detach()
            best_model.intercept_ = base_model.intercept_.clone().detach()
            best_inlier_mask = inlier_mask.clone()

    return best_model, best_inlier_mask

def mad_torch(y):
    median_y = torch.median(y)
    mad = torch.median(torch.abs(y - median_y))
    return mad

# Instantiate the base linear regression model
# base_model = LinearRegressionTorch()

# # Fit RANSAC model
# ransac_model, inlier_mask = ransac_fit(X, y, base_model, min_samples=50, residual_threshold=10.0)

# # Predict values for visualization
# line_X = torch.linspace(X.min(), X.max(), steps=100).view(-1, 1).to(device)
# line_y_ransac = ransac_model.predict(line_X)


# Output coefficients
# print(f"RANSAC slope: {ransac_model.coef_.item()}")
# print(f"RANSAC intercept: {ransac_model.intercept_.item()}")
