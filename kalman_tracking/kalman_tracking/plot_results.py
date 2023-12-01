import matplotlib.pyplot as plt

# Read the file
file_path = "/home/ollie/ros2_ws/src/kalman_tracking/kalman_tracking/Results.txt"
with open(file_path, "r") as file:
    lines = file.readlines()

# Extract the data
x_estimated = []
y_estimated = []
x_actual = []
y_actual = []
xv_estimated = []
yv_estimated = []
xv_actual = []
yv_actual = []
time = []
caught_count = 0
target_lost_count = 0
failed_count = 0
all_tests = []
for line in lines:
    values = line.strip().split()
    if values[0] == "CAUGHT":
        caught_count += 1
        all_tests.append((x_estimated, y_estimated, x_actual, y_actual, xv_estimated, yv_estimated, xv_actual, yv_actual, time))
        x_estimated = []
        y_estimated = []
        x_actual = []
        y_actual = []
        xv_estimated = []
        yv_estimated = []
        xv_actual = []
        yv_actual = []
        time = []
    elif values[0] == "FAILED":
        failed_count += 1
        all_tests.append((x_estimated, y_estimated, x_actual, y_actual, xv_estimated, yv_estimated, xv_actual, yv_actual, time))
        x_estimated = []
        y_estimated = []
        x_actual = []
        y_actual = []
        xv_estimated = []
        yv_estimated = []
        xv_actual = []
        yv_actual = []
        time = []
    elif values[0] == "TARGET":
        target_lost_count += 1
    else:
        x_estimated.append(float(values[0]))
        y_estimated.append(float(values[1]))
        x_actual.append(float(values[2]))
        y_actual.append(float(values[3]))
        xv_estimated.append(float(values[4]))
        yv_estimated.append(float(values[5]))
        xv_actual.append(float(values[6]))
        yv_actual.append(float(values[7]))
        time.append(len(time) * 0.5)  # Each reading is 0.5 seconds apart

def plot_particular_test(test_number):
    # Plot the data
    plt.plot(all_tests[test_number][8], all_tests[test_number][0], label="Estimated", color="blue")
    plt.plot(all_tests[test_number][8], all_tests[test_number][2], label="Actual", color="red")
    plt.xlabel("Time (seconds)")
    plt.ylabel("X Position")
    plt.title("Difference between Estimated and Actual X values over Time")
    plt.legend()
    plt.show()

    # Plot the data
    plt.plot(all_tests[test_number][8], all_tests[test_number][1], label="Estimated", color="blue")
    plt.plot(all_tests[test_number][8], all_tests[test_number][3], label="Actual", color="red")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Y Position")
    plt.title("Difference between Estimated and Actual Y values over Time")
    plt.legend()
    plt.show()

# Plot root mean squared error over all tests
def plot_rmse():
    rmse_x = []
    rmse_y = []
    for test in all_tests:
        rmse_x.append(((sum([(test[0][i] - test[2][i])**2 for i in range(len(test[0]))]) / len(test[0]))**0.5))
        rmse_y.append(((sum([(test[1][i] - test[3][i])**2 for i in range(len(test[1]))]) / len(test[1]))**0.5))
    print(f"Mean RMSE X: {sum(rmse_x) / len(rmse_x)}")
    print(f"Mean RMSE Y: {sum(rmse_y) / len(rmse_y)}")
    plt.plot(rmse_x, label="X")
    plt.plot(rmse_y, label="Y")
    plt.xlabel("Test Number")
    plt.ylabel("Root Mean Squared Error")
    plt.title("Root Mean Squared Error over all Tests")
    plt.legend()
    plt.show()

# Create table of results
def create_table():
    print(f"Caught: {caught_count}")
    print(f"Target Lost: {target_lost_count}")
    print(f"Total: {caught_count + failed_count}")
    print(f"Percentage Caught: {caught_count / (caught_count + failed_count) * 100}")
    print(f"Percentage Target Lost: {target_lost_count / (caught_count + failed_count) * 100}")

#create_table()

#plot_rmse()

plot_particular_test(0)