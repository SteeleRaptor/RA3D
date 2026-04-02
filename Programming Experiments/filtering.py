
import csv
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def nPointAverage(n, inputData):
    nPointAvg = [] # List to save the averaged data
    prevVals = [inputData[0]] * n # List containing previous values for averaging (start value at initial temperature reading)

    # Iterate through the temperature data
    for i in range(0, len(inputData)):
        # Shift previous values (most recent in index 0)
        for j in range(n - 1, 0, -1):
            prevVals[j] = prevVals[j - 1]
        # Save the most recent
        prevVals[0] = inputData[i]
        # Compute the average
        temp = sum(prevVals) / n
        # Add the averaged data point to the list
        nPointAvg.append(temp)

    return nPointAvg

time = []
temps = []
isOn = []

# Read in the csv data
# log6.csv is hotend measurements without a crosstalk compensator
# log7.csv is hotend measurements with a crosstalk compensator
with open ("Programming Experiments/log6.csv", newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        if row[0] == "time":
            continue
        time.append(round(float(row[0]), 3))
        temps.append(round(float(row[1]), 3))
        isOn.append(row[3])

print("CSV Parsed")

# n-point average
# nPointAvg = [] # List to save the averaged data
pointAvg = 25 # n (Note that larget n's result in more delay)
# prevVals = [temps[0]] * pointAvg # List containing previous values for averaging (start value at initial temperature reading)
nPointAvg = nPointAverage(pointAvg, temps)

# # Iterate through the temperature data
# for i in range(0, len(temps)):
#     # Shift previous values (most recent in index 0)
#     for j in range(0, pointAvg - 1):
#         prevVals[pointAvg - 1 - j] = prevVals[pointAvg - 2 - j]
#     # Save the most recent
#     prevVals[0] = temps[i]
#     # Compute the average
#     temp = sum(prevVals) / pointAvg
#     # Add the averaged data point to the list
#     nPointAvg.append(temp)

# =====| Finding Largest Jumps |=====
# Essentially a crosstalk locator
maxSavedJumps = 10 # How many data points to save for averaging later
largestAbsJumps = [0] * maxSavedJumps # List of all the saved jumps
timeRelation = [0] * maxSavedJumps
# Iterate through temperature data
for i in range(1, len(temps)):
    # Calculate abs jump
    temp = abs(temps[i] - temps[i - 1])
    # Handle saving the largest jump values
    # Check the largest value first (stored in index 0)
    for j in range(0, maxSavedJumps):
        # If the temperature is greater than whatever is at index j...
        if temp > largestAbsJumps[j]:
            # Shift all the data over
            for k in range(maxSavedJumps - 1, j, -1):
                largestAbsJumps[k] = largestAbsJumps[k - 1]
                timeRelation[k] = timeRelation[k - 1]
            # And put the new value in place of j
            largestAbsJumps[j] = temp
            timeRelation[j] = time[i]
            break

# Print all the largest values found
print(largestAbsJumps)
print(timeRelation)
# Print the average of the jumps
avgLargestAbsJump = sum(largestAbsJumps) / maxSavedJumps
print(avgLargestAbsJump)

compensatedTemp = []

for i in range(0, len(temps)):
    if isOn[i] == "True":
        compensatedTemp.append(temps[i] + avgLargestAbsJump)
    else:
        compensatedTemp.append(temps[i])

avgCompensatedTemp = nPointAverage(pointAvg, compensatedTemp)


plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time, isOn, color="blue", label="Raw Data")
plt.title("Heater State Data")
plt.xlabel("Time (s)")
plt.ylabel("Heater On")

plt.subplot(3, 1, 2)
plt.plot(time, temps, color="blue", label="Raw Data")
plt.plot(time, nPointAvg, color="green", label=f"{pointAvg} point average")
plt.title("Temperature Data")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (C)")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, compensatedTemp, color="red", label=f"Compensated Data")
plt.plot(time, avgCompensatedTemp, color="orange", label=f"{pointAvg} avg Compensated Data")
plt.title("Compensated Temperature Data")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (C)")
plt.legend()


plt.show()