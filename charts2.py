import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Patch
import csv

def read_csv_to_2d_array(filename):
    data = []
    with open(filename, mode="r", newline="") as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            data.append([int(x) if x.isdigit() else float(x) for x in row])  # Each row is appended as a list
    return np.array(data)

# Example usage
csv_filename = "Exhaustive_test_results.csv"  # Replace with your actual file name
data_array = read_csv_to_2d_array(csv_filename)

# Print the 2D array
for row in data_array:
    print(row)


# Data extracted from your output
training_set_sizes = ["100%", "80%", "60%", "40%", "20%"]
misdetections = np.array([1, 2, 3])

# Mean accuracy values per training set size (matching order)
means = {
    "100%": [data_array[0,2],data_array[1,2],data_array[2,2]], 
    "80%":  [data_array[5,2],data_array[6,2],data_array[7,2]],
    "60%":  [data_array[10,2],data_array[11,2],data_array[12,2]],
    "40%":  [data_array[15,2],data_array[16,2],data_array[17,2]],
    "20%":  [data_array[20,2],data_array[21,2],data_array[22,2]]
}

# Standard deviation values (capped at 1)
stds = {
    "100%": [min(data_array[0,3], 1), min(data_array[1,3], 1), min(data_array[2,3], 1)],
    "80%":  [min(data_array[5,3], 1), min(data_array[6,3], 1), min(data_array[7,3], 1)],
    "60%":  [min(data_array[10,3], 1), min(data_array[11,3], 1), min(data_array[12,3], 1)],
    "40%":  [min(data_array[15,3], 1), min(data_array[16,3], 1), min(data_array[17,3], 1)],
    "20%":  [min(data_array[20,3], 1), min(data_array[21,3], 1), min(data_array[22,3], 1)]
}

# Error data for different training set percentages (from provided table)
error_B = {
    "100%": [data_array[0,6],data_array[1,6],data_array[2,6]], 
    "80%":  [data_array[5,6],data_array[6,6],data_array[7,6]],
    "60%":  [data_array[10,6],data_array[11,6],data_array[12,6]],
    "40%":  [data_array[15,6],data_array[16,6],data_array[17,6]],
    "20%":  [data_array[20,6],data_array[21,6],data_array[22,6]]
}

error_A = {
    "100%": [data_array[0,7],data_array[1,7],data_array[2,7]], 
    "80%":  [data_array[5,7],data_array[6,7],data_array[7,7]],
    "60%":  [data_array[10,7],data_array[11,7],data_array[12,7]],
    "40%":  [data_array[15,7],data_array[16,7],data_array[17,7]],
    "20%":  [data_array[20,7],data_array[21,7],data_array[22,7]]
}

error_C = {
    "100%": [data_array[0,8],data_array[1,8],data_array[2,8]], 
    "80%":  [data_array[5,8],data_array[6,8],data_array[7,8]],
    "60%":  [data_array[10,8],data_array[11,8],data_array[12,8]],
    "40%":  [data_array[15,8],data_array[16,8],data_array[17,8]],
    "20%":  [data_array[20,8],data_array[21,8],data_array[22,8]]
}

print("Error A:",error_A)
print("Error B:",error_B)
print("Error C:",error_C)
# Bar width and positions
bar_width = 0.15
x = np.arange(len(misdetections))

# Use a colorblind-friendly palette
colorblind_palette = sns.color_palette("colorblind", n_colors=len(training_set_sizes))

# Create plot
fig, ax = plt.subplots(figsize=(10, 6))

# Plot stacked bars for each training percentage
for i, (train_size, color) in enumerate(zip(training_set_sizes, colorblind_palette)):
    ax.bar(x + i * bar_width, means[train_size], bar_width, yerr=stds[train_size],
           label=f"{train_size}", capsize=4, color=color, alpha=0.8, error_kw={'elinewidth': 1.5})

# Configure plot
plt.yticks(fontsize=20)
ax.set_xlabel("Number of Occlusions", fontsize=20)
ax.set_ylabel("Task Completion Rate", fontsize=20)
#ax.set_title("System effectiveness")
ax.set_xticks(x + (len(training_set_sizes) - 1) * bar_width / 2,fontsize=20)
ax.set_xticklabels(misdetections,fontsize=20)
#ax.set_ticklabels(axis='y', fontsize=20)
#ax.set_yticklabels(fontsize=20)
#ax.set_yticks(fontsize=20)
ax.set_ylim(0.05, 1)  # Y-axis limit from 0.2 to 1
ax.legend(fontsize=18, loc='lower left', title="Training set: ", title_fontsize='xx-large')
ax.grid(axis="y", linestyle="--", alpha=0.7)

# Show the plot
plt.show()

error_legend_patches = [
    Patch(color="#5b9bd5", label="A"),
    Patch(color="#8cbf72", label="B"),
    Patch(color="#f4a261", label="C")
]
legend1 = ax.legend(handles=error_legend_patches, title="Error Types", fontsize=12, loc="upper right", frameon=True)


# Define patterns for each training set percentage
hatch_patterns = ['/', '\\', 'x', '-', '|']  # Different patterns

def normalize_errors_per_misdetection(error_A, error_B, error_C, training_set_sizes):
    """
    Normalize errors per misdetection case to obtain percentage values.
    """
    norm_A, norm_B, norm_C = {}, {}, {}

    for train_size in training_set_sizes:
        norm_A[train_size] = []
        norm_B[train_size] = []
        norm_C[train_size] = []

        for i in range(len(error_A[train_size])):  # Loop over misdetections
            total = error_A[train_size][i] + error_B[train_size][i] + error_C[train_size][i]

            if total > 0:  # Avoid division by zero
                norm_A[train_size].append(error_A[train_size][i] / total*100)
                norm_B[train_size].append(error_B[train_size][i] / total*100)
                norm_C[train_size].append(error_C[train_size][i] / total*100)
            else:
                norm_A[train_size].append(0)
                norm_B[train_size].append(0)
                norm_C[train_size].append(0)

    return norm_A, norm_B, norm_C



# Normalize errors
norm_error_A, norm_error_B, norm_error_C = normalize_errors_per_misdetection(
    error_A, error_B, error_C, ["100%", "80%", "60%", "40%", "20%"]
)

# Bar plot settings
bar_width = 0.15
x = np.arange(len(error_A["100%"]))  # Number of misdetections
num_training_sizes = len(norm_error_A.keys())

fig, ax = plt.subplots(figsize=(10, 6))

# Use colorblind-friendly colors
colors = ["#5b9bd5", "#8cbf72", "#f4a261"] #colors = ["b", "g", "r"]
training_set_sizes = ["100%", "80%", "60%", "40%", "20%"]

# Iterate over training sizes
for i, (train_size, hatch) in enumerate(zip(training_set_sizes, hatch_patterns)):
    bottom_B = norm_error_A[train_size]  # Start B above A
    bottom_C = np.array(norm_error_A[train_size]) + np.array(norm_error_B[train_size])  # Start C above A+B

    ax.bar(x + i * bar_width, norm_error_A[train_size], bar_width, label=f"A ({train_size})", color=colors[0], hatch=hatch)
    ax.bar(x + i * bar_width, norm_error_B[train_size], bar_width, bottom=bottom_B, label=f"B ({train_size})", color=colors[1], hatch=hatch)
    ax.bar(x + i * bar_width, norm_error_C[train_size], bar_width, bottom=bottom_C, label=f"C ({train_size})", color=colors[2], hatch=hatch)

hatch_legend_patches = [
    Patch(facecolor='white', edgecolor='black', hatch=hatch, label=f"{train_size}")
    for train_size, hatch in zip(training_set_sizes, hatch_patterns)
]
legend2 = ax.legend(handles=hatch_legend_patches, title="Training Set %", fontsize=12, loc="upper left", frameon=True)

# Add the first legend back (since the second one replaces it)
#ax.add_artist(legend1)
ax.legend(handles=hatch_legend_patches,  fontsize=12, loc="upper right", frameon=True)

# Labels and legend
ax.set_xlabel("Number of Occlusions", fontsize=20)
ax.set_ylabel("Normalized Error Rate Percentage", fontsize=20)
ax.set_xticks(x + (num_training_sizes - 1) * bar_width / 2)
ax.set_xticklabels([str(i+1) for i in range(len(error_A["100%"]))],fontsize=20)  # Labels for misdetections
ax.set_ylim(0, 100)  # Ensure percentages range from 0 to 1
plt.yticks(fontsize=20)

ax.legend(handles=error_legend_patches,  fontsize=18, loc="lower left", title="Error Type: ", title_fontsize='xx-large')
plt.grid(axis="y", linestyle="--", alpha=0.7)

plt.tight_layout()
plt.show()

