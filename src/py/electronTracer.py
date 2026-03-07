import numpy as np
import pickle
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator


q_e = -1.602e-19  # C
m_e = 9.109e-31  # kg

V_acc = 5000  # V
v_e = np.sqrt(2 * -q_e * V_acc / m_e) * 1e3  # mm/s

N_electrons = 25
spread = 0.5  # mm

T = 90 / v_e
dt = T / 10000
N_steps = int(T / dt)

def main():
    with open('field_x.pkl', 'rb') as f:
        field_x = pickle.load(f) * 1e3 # Convert from V/mm to V/m
    with open('field_y.pkl', 'rb') as f:
        field_y = pickle.load(f) * 1e3 # Convert from V/mm to V/m

    with open('xs.pkl', 'rb') as f:
        xs = pickle.load(f)
    with open('ys.pkl', 'rb') as f:
        ys = pickle.load(f)


    field_x_interp = RegularGridInterpolator((ys, xs), field_x, bounds_error=False, fill_value=0)
    field_y_interp = RegularGridInterpolator((ys, xs), field_y, bounds_error=False, fill_value=0)


    plt.subplot(2, 2, 1)
    plt.imshow(field_x, cmap='viridis', extent=(xs[0], xs[-1], ys[-1], ys[0]), origin='upper')
    plt.colorbar()
    plt.title('Field X')

    plt.subplot(2, 2, 2)
    plt.imshow(field_y, cmap='viridis', extent=(xs[0], xs[-1], ys[-1], ys[0]), origin='upper')
    plt.colorbar()
    plt.title('Field Y')

    trajectories = []
    for _ in range(N_electrons):
        print(_)
        x = np.random.normal(0, spread)
        y = 0
        vx = 0
        vy = v_e
        trajectory = [(x, y)]
        for _ in range(N_steps):
            acc_x = q_e * field_x_interp((y, x)) / m_e * 1e3 # Convert from m/s^2 to mm/s^2
            acc_y = q_e * field_y_interp((y, x)) / m_e * 1e3 # Convert from m/s^2 to mm/s^2
            vx += acc_x * dt
            vy += acc_y * dt
            # print(vx * dt)
            x += vx * dt
            y += vy * dt
            trajectory.append((x, y))
        trajectories.append(trajectory)
    
    plt.subplot(2, 2, 3)

    for trajectory in trajectories:
        trajectory = np.array(trajectory)
        plt.imshow(field_x, cmap='viridis', extent=(xs[0], xs[-1], ys[-1], ys[0]), origin='upper', alpha=0.5)
        plt.plot(trajectory[:, 0], trajectory[:, 1], color="blue", alpha=0.25)
    plt.title('Electron Trajectories')


    plt.show()

if __name__ == "__main__":
    main()