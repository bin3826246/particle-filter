import matplotlib.pyplot as plt
import numpy as np

result_file = open('result_data.txt', 'r')
gt_file = open('data/gt_data.txt', 'r')
map_file = open('data/map_data.txt', 'r')

result_x = np.array([])
result_y = np.array([])
gt_x = np.array([])
gt_y = np.array([])
map_x = np.array([])
map_y = np.array([])

for line in result_file.readlines():
    line = line.strip('\n')
    data = line.split(' ')
    result_x = np.append(result_x, float(data[0]))
    result_y = np.append(result_y, float(data[1]))

count = 0
for line in gt_file.readlines():
    line = line.strip('\n')
    data = line.split(' ')
    gt_x = np.append(gt_x, float(data[0]))
    gt_y = np.append(gt_y, float(data[1]))
    count += 1
    if count > 900:
        break

for line in map_file.readlines():
    line = line.strip('\n')
    data = line.split('\t')
    map_x = np.append(map_x, float(data[0]))
    map_y = np.append(map_y, float(data[1]))

plt.figure(figsize=(12, 8))
plt.plot(result_x, result_y, 'o', label='best particles')
plt.plot(gt_x, gt_y, '-', label='ground truth')
plt.plot(map_x, map_y, 'o', label='landmarks')


plt.legend()
plt.show()
