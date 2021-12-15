import numpy as np
import matplotlib.pyplot as plt

z_ranger = np.load('z_rangers.npy', allow_pickle=True)

fig, ax = plt.subplots()
ax.plot(np.arange(len(z_ranger)), z_ranger)

ax.set(xlabel='timeline', ylabel='Z ranger [m]',
       title='Z ranger values over time')
ax.grid()

plt.show()
