# open file /logs/rewards.txt take the mean of each episodes rewards denoted by the text EPISODE END then plot the results

import matplotlib.pyplot as plt

# Open the file
file_path = './envs/rewards.txt'
with open(file_path, 'r') as file:
    # Read the contents of the file
    contents = file.read()

# Split the contents into lines
lines = contents.split('\n')

# Initialize an empty list to store the rewards
rewards = []
temp = 0
count = 0
first = True
# Iterate over each line
for line in lines:
    # Check if the line contains "EPISODE END"
    if 'EPISODE END' in line and not first:        
        # Append the reward to the list of rewards
        rewards.append(temp/count)
        temp = 0
        count = 0
    elif line != '' and not first:
        temp += float(line)
        count += 1
    else:
        first = False

# Plot the results
plt.plot(rewards)
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.title('Rewards per Episode')
plt.show()

