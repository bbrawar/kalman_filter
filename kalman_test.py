import numpy as np
import matplotlib.pyplot as plt

# Function to generate test time series data
def generate_data(n=100, process_variance=1, measurement_variance=2):
    """
    Generates a time series with noise.
    Args:
    n (int): Number of time steps.
    process_variance (float): Variance of the process (system) noise.
    measurement_variance (float): Variance of the measurement noise.

    Returns:
    x_true (numpy array): True values (without noise).
    x_obs (numpy array): Observed values (with measurement noise).
    """
    # True values - for example, linear motion with constant velocity
    x_true = np.linspace(0, 100, n)
    
    # Process noise (system dynamics noise)
    process_noise = np.random.normal(0, np.sqrt(process_variance), n)
    
    # Add process noise to the true values to simulate system changes
    x_true += process_noise
    
    # Observed values with measurement noise
    measurement_noise = np.random.normal(0, np.sqrt(measurement_variance), n)
    x_obs = x_true + measurement_noise
    
    return x_true, x_obs

# Kalman Filter implementation
def kalman_filter(z, Q, R):
    """
    Applies Kalman filter to the observed data.
    Args:
    z (numpy array): Observed values.
    Q (float): Process variance.
    R (float): Measurement variance.
    
    Returns:
    x_filtered (numpy array): Kalman filtered values.
    """
    n = len(z)
    x_filtered = np.zeros(n)  # Filtered values
    P = np.zeros(n)  # Estimate error covariance
    
    # Initial guesses
    x_filtered[0] = z[0]  # First estimate is the first measurement
    P[0] = 1  # Initial error covariance
    
    for k in range(1, n):
        # Prediction step
        x_pred = x_filtered[k-1]  # Predicted state estimate (same as previous filtered value)
        P_pred = P[k-1] + Q  # Predicted estimate covariance
        
        # Update step
        K = P_pred / (P_pred + R)  # Kalman Gain
        x_filtered[k] = x_pred + K * (z[k] - x_pred)  # Update estimate with measurement z[k]
        P[k] = (1 - K) * P_pred  # Update the error covariance
    
    return x_filtered

# Generate test data
n = 100  # Number of time steps
process_variance = 1  # Process noise variance
measurement_variance = 2  # Measurement noise variance
x_true, x_obs = generate_data(n, process_variance, measurement_variance)

# Apply Kalman filter
Q = process_variance  # Process variance (system dynamics)
R = measurement_variance  # Measurement variance
x_filtered = kalman_filter(x_obs, Q, R)

# Plot observed vs filtered data
plt.figure(figsize=(10, 6))
plt.plot(x_true, label='True values', linestyle='--', color='k')
plt.plot(x_obs, label='Observed values', color='r', alpha=0.5)
plt.plot(x_filtered, label='Kalman Filtered values', color='b')
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Observed vs Kalman Filtered Data')
plt.legend()
plt.grid(True)
plt.show()
