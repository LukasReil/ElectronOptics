import numpy as np
import matplotlib.pyplot as plt




def main():
    potential_map = np.loadtxt("potential_map.txt", delimiter=",")
    plt.imshow(potential_map, extent=(-50, 50, -100, 0), origin="lower", cmap="viridis")
    plt.colorbar(label="Potential (V)")
    plt.title("Electrostatic Potential Map")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.show()


if __name__ == "__main__":
    main()