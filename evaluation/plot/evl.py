import pandas as pd
import matplotlib.pyplot as plt

# Reading the accuracy history CSV file
accuracy_history_file = "dynamic/RNN_dynamic_accuracy.csv"
accuracy_history = pd.read_csv(accuracy_history_file)
train_acc = accuracy_history['Training Accuracy']
val_acc = accuracy_history['Validation Accuracy']
plt.plot(train_acc, color='blue', label='Training Accuracy')
plt.plot(val_acc, color='green', label='Validation Accuracy')
plt.grid(True)

ax = plt.gca()
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)  
ax.spines['left'].set_visible(False) 
ax.spines['bottom'].set_visible(False) 

# set background color
ax.set_facecolor((0.898, 0.898, 0.898))

# set grid to white
ax.grid(color='white')
plt.show()
