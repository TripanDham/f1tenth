import csv
import matplotlib.pyplot as plt

# Initialize lists to store data
timestamps = []
x_positions = []
y_positions = []

# Read the CSV file and extract data
with open('f1tenth/pose_data.csv', mode='r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header if present
    for row in csv_reader:
        timestamp = row[0]
        x = float(row[1])
        y = float(row[2])
        
        # Append to lists
        timestamps.append(timestamp)
        x_positions.append(x)
        y_positions.append(y)

# Plot the data
plt.figure(figsize=(8, 6))
d = 800
c = 1100
# d = 700
# c = 3000
plt.plot(x_positions[d:c], y_positions[d:c], label="Trajectory")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Position Plot")
plt.xlim((-2,5))
plt.ylim((-1,6))
plt.grid(True)
plt.legend()
plt.show()
