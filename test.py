import csv
import numpy as np

# with open('training_objects_params.txt', 'r') as f:
#     reader = csv.reader(f, delimiter=',')
#     print(reader)


all = np.loadtxt('training_objects_params.txt', delimiter=',', skiprows=1, dtype=np.float64)
first_row =list(all[0,:].flatten())
# .flatten())
print(first_row)
