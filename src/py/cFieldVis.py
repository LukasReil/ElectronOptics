import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['axes3d.mouserotationstyle'] = 'azel'



# def main():
#     potential_map = np.loadtxt("potential_map.txt", delimiter=",")
#     plt.imshow(potential_map, extent=(-50, 50, -100, 0), origin="lower", cmap="viridis")
#     plt.colorbar(label="Potential (V)")
#     plt.title("Electrostatic Potential Map")
#     plt.xlabel("X (mm)")
#     plt.ylabel("Y (mm)")
#     plt.show()


def main():
    potential_map = np.loadtxt("solvedPotentials.txt")[::50, :]

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(projection='3d')

    potential_map = potential_map[(potential_map[:, 0] < 0.3) & (potential_map[:, 0] > -0.3)]

    ax.scatter(potential_map[:, 2], potential_map[:, 0], potential_map[:, 1], c=potential_map[:, 3], cmap='viridis')
    ax.set_aspect("equal")
    fig.colorbar(ax.collections[0], label='Potential (V)')
    fig.show()
    plt.show()
    

if __name__ == "__main__":
    main()