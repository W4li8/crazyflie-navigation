import numpy as np
import matplotlib.pyplot as plt

z_ranger = np.load('rangers_.npy', allow_pickle=True)

convolved = np.diff(z_ranger, n=2)

plt.plot(convolved)
plt.plot(z_ranger)
plt.show()

# fig, ax = plt.subplots()
# ax.plot(np.arange(len(z_ranger)), z_ranger)

# ax.set(xlabel='timeline', ylabel='Z ranger [m]',
#        title='Z ranger values over time')
# ax.grid()

# plt.show()
