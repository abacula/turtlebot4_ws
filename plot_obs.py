import matplotlib.pyplot as plt
import csv

x = []
y = []

with open('obs_locs.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter = ',')
    
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))

plt.scatter(x, y, color = 'g')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Obs locations')
plt.legend()
plt.show()