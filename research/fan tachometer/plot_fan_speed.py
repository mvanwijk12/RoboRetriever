import matplotlib.pyplot as plt

# Function to read RPM data from the file
def read_rpm_data(filename):
    rpm_values = []
    with open(filename, 'r') as file:
        for line in file:
            # Extract the RPM value from each line
            if "RPM:" in line:
                rpm_value = float(line.split(":")[1].strip())
                rpm_values.append(rpm_value)
    return rpm_values

# Function to plot the RPM data
def plot_rpm_data(rpm_values):
    plt.figure(figsize=(10, 6))
    plt.plot(rpm_values, marker='o', linestyle='-', color='b', label='Fan RPM')
    plt.title('Fan Speed Over Time')
    plt.xlabel('Sample Number')
    plt.ylabel('RPM')
    plt.grid(True)
    plt.legend()
    plt.show()

# Main code
if __name__ == "__main__":
    filename = 'fan_speed.log'  # Specify the file path
    rpm_values = read_rpm_data(filename)
    plot_rpm_data(rpm_values)
