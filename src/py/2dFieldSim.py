import numpy as np
import matplotlib.pyplot as plt

x_range = (-10, 10)
y_range = (-10, 10)

xs = np.linspace(x_range[0], x_range[1], 1001)
ys = np.linspace(y_range[0], y_range[1], 1001)

lens_config = (0, 1, 1) # (y, strength, size)

source_config = (0, 8) # (x, y)
source_x_pos_std = 0.1
source_count = 100
sim_dt = 0.001
sim_T = 20
sim_steps = int(sim_T / sim_dt)
source_vel = (0, -1) # (vx, vy)

m_e = 1
q_e = -1

def lens_potential_1d(y, lens_config):
    x_pos, strength, size = lens_config
    dy = y - x_pos
    size_scale_factor = 1 / (2 * size**2)
    gaussian = strength * np.exp(-(dy**2) * size_scale_factor)  # Gaussian in y-direction
    return gaussian

def lens_potential_1d_diff(y, lens_config):
    x_pos, strength, size = lens_config
    dy = y - x_pos
    size_scale_factor = 1 / (2 * size**2)
    gaussian = strength * np.exp(-(dy**2) * size_scale_factor)  # Gaussian in y-direction
    return gaussian * (-2 * dy * size_scale_factor)  # Derivative of Gaussian

def lens_potential_1d_diff2(y, lens_config):
    x_pos, strength, size = lens_config
    dy = y - x_pos
    size_scale_factor = 1 / (2 * size**2)
    gaussian = strength * np.exp(-(dy**2) * size_scale_factor)  # Gaussian in y-direction
    return gaussian * (4 * dy**2 * size_scale_factor**2 -2 * size_scale_factor)  # Second derivative of Gaussian

def lens_potential_1d_diff3(y, lens_config):
    x_pos, strength, size = lens_config
    dy = y - x_pos
    size_scale_factor = 1 / (2 * size**2)
    gaussian = strength * np.exp(-(dy**2) * size_scale_factor)  # Gaussian in y-direction
    return gaussian * (-8 * dy**3 * size_scale_factor**3 + 12 * dy * size_scale_factor**2)  # Third derivative of Gaussian

def lens_field_y(x, y, lens_config):
    return -lens_potential_1d_diff(y, lens_config) + 0.25 * x**2 * lens_potential_1d_diff3(y, lens_config)

def lens_field_x(x, y, lens_config):
    return 0.5 * x * lens_potential_1d_diff2(y, lens_config)

def main():
    X, Y = np.meshgrid(xs, ys)
    Z_x = lens_field_x(X, Y, lens_config)
    Z_y = lens_field_y(X, Y, lens_config)
    plt.subplot(2, 2, 1)
    plt.imshow(Z_x, extent=x_range + y_range, origin='lower', cmap=plt.get_cmap('viridis', 31))
    plt.title('Field in x-direction')
    plt.colorbar()
    plt.subplot(2, 2, 2)
    plt.imshow(Z_y, extent=x_range + y_range, origin='lower', cmap=plt.get_cmap('viridis', 31))
    plt.title('Field in y-direction')

    trajectories = []
    for i in range(source_count):
        x = source_config[0] + np.random.normal(0, source_x_pos_std)
        y = source_config[1]
        vx = source_vel[0]
        vy = source_vel[1]
        trajectory = [(x, y)]
        for _ in range(sim_steps):
            field_x = lens_field_x(x, y, lens_config)
            field_y = lens_field_y(x, y, lens_config)
            vx += (q_e / m_e) * field_x * sim_dt
            vy += (q_e / m_e) * field_y * sim_dt
            x += vx * sim_dt
            y += vy * sim_dt
            trajectory.append((x, y))
        trajectories.append(trajectory)

    for trajectory in trajectories:
        trajectory = np.array(trajectory)
        plt.subplot(2, 2, 3)
        plt.plot(trajectory[:, 0], trajectory[:, 1], color='blue', alpha=0.1)
        plt.xlim(x_range)
        plt.ylim(y_range)
        plt.title('Particle Trajectories')

    plt.colorbar()
    plt.show()

if __name__ == "__main__":
    main()