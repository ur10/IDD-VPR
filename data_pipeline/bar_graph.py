import matplotlib.pyplot as plt

# Sample data
categories = ['3 Oct', '7 Oct', '16 Oct', '21 Oct']
values = [100, 10, 8, 13]

# Create a bar graph
plt.bar(categories, values, color='red')

# Add labels to each bar
for i in range(len(categories)):
    plt.text(i, values[i], str(values[i]), ha='center', va='bottom')

# Labeling the axes and title
plt.xlabel('Date')
plt.ylabel('Average GPS drop')
plt.title('GPS drop analysis')

# Display the plot
plt.show()

