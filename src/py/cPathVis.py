import numpy as np
import matplotlib.pyplot as plt

import matplotlib as mpl
mpl.rcParams['axes3d.mouserotationstyle'] = 'azel'

def main():
    path_data = np.loadtxt("electronPath.txt")
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(projection='3d')
    for i in range(0, path_data.shape[0]):
        ax.plot(path_data[i, 0::3], path_data[i, 2::3], path_data[i, 1::3])
    ax.set_aspect("equal")
    ax.set_title("Electron Path")
    fig.show()
    plt.show()

if __name__ == "__main__":
    main()