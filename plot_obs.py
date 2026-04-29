import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv

x = []
y = []


# creating the figure and axes object
fig, ax = plt.subplots()

# update function to update data and plot
def update(frame):
    with open('obs_loc.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        
        for row in plots:
            x.append(float(row[0]))
            y.append(float(row[1]))

    ax.clear()  # clearing the axes
    ax.scatter(x,y, s = y, c = 'b', alpha = 0.5)  # creating new scatter chart with updated data
    fig.canvas.draw()  # forcing the artist to redraw itself

anim = FuncAnimation(fig, update)
plt.show()