import pandas as pd
import matplotlib.pyplot as plt

# Load data from the Excel file
file_path = "cam_modified_sine.xlsx"
data = pd.read_excel(file_path)

# Display the first few rows of the DataFrame to check the data
print(data.head())

# Select the columns you want to plot. For example, let's plot 'x_mm' against 's_mm'
x_column = 'Angle_deg'         # Change according to your column names
y_column = 'j_mm_per_deg3'         # Change according to your column names

# Ensure the selected columns exist in the DataFrame
if x_column in data.columns and y_column in data.columns:
    plt.figure(figsize=(10, 6))

    # Create the plot
    plt.plot(data[x_column], data[y_column], marker='o')

    # Add titles and labels
    plt.title(f'{y_column} vs {x_column}')
    plt.xlabel(x_column)
    plt.ylabel(y_column)

    # Show grid
    plt.grid()

    # Show the plot
    plt.show()
else:
    print(f"Columns {x_column} or {y_column} do not exist in the data.")
