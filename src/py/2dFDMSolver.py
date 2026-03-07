import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from PIL import Image
import pickle


grid_size_mm = (50, 100)
grid_resolution_mm = 0.1

def set_potential_field_boundary_conditions(field, values, mask):
    field[mask] = values[mask]

def relax_potential_field(field, omega=1.0):
    new_field = convolve2d(field, np.array([[0, 1, 0], [1, 0, 1], [0, 1, 0]]) * 0.25, mode='same', boundary='fill', fillvalue=0)
    return field + (new_field - field) * omega

def sor_potential_field(field, omega=1.98):
    field = field.copy()
    
    rows, cols = np.indices(field.shape)
    red_mask  = ((rows + cols) % 2 == 0)
    black_mask = ((rows + cols) % 2 == 1)


    kernel = np.array([[0, 1, 0],
                       [1, 0, 1],
                       [0, 1, 0]]) * 0.25
    for mask in [red_mask, black_mask]:
        avg = convolve2d(field, kernel, mode='same', boundary='fill', fillvalue=0)
        field = np.where(mask, field + omega * (avg - field), field)
    
    return field


def calculate_field_from_potential(potential_field):
    return convolve2d(potential_field, np.array([[0, 0, 0], [-1, 0, 1], [0, 0, 0]]) * (0.5 / grid_resolution_mm), mode='same', boundary='fill', fillvalue=0), \
           convolve2d(potential_field, np.array([[0,-1, 0], [ 0, 0, 0], [0, 1, 0]]) * (0.5 / grid_resolution_mm), mode='same', boundary='fill', fillvalue=0)

def improve_initial_guess(field, init_field_image, init_field_mask, sigma=5, iterations=10):
    for _ in range(iterations):
         field = gaussian_filter(field, sigma=sigma)
         set_potential_field_boundary_conditions(field, init_field_image, init_field_mask)
    return field


def main():
    x_range = (-grid_size_mm[0] / 2, grid_size_mm[0] / 2)
    y_range = (0, grid_size_mm[1])
    xs = np.arange(x_range[0], x_range[1] + grid_resolution_mm, grid_resolution_mm)
    ys = np.arange(y_range[0], y_range[1] + grid_resolution_mm, grid_resolution_mm)

    field_image = np.copy(np.asarray(Image.open('src/py/lens_test.png').convert('RGB')))
    # field_image = np.flip(field_image, axis=0) # Flip the image vertically to match the coordinate system
    print(field_image.shape)
    init_field_image = np.zeros(field_image.shape[0:2])
    init_field_image[field_image[:,:,0] > 128] = 5000
    init_field_image[field_image[:,:,2] > 128] = -1
    init_field_mask = init_field_image != 0

    potential_field = np.zeros((len(ys), len(xs)))
    set_potential_field_boundary_conditions(potential_field, init_field_image, init_field_mask)
    potential_field = improve_initial_guess(potential_field, init_field_image, init_field_mask, sigma=5, iterations=0)

    delta = float('inf')
    i = 0
    while delta > 0.1:
        new_potential_field = sor_potential_field(potential_field)
        set_potential_field_boundary_conditions(new_potential_field, init_field_image, init_field_mask)
        delta = np.max(np.abs(new_potential_field - potential_field))
        potential_field = new_potential_field
        i += 1
        if i % 10 == 0:
            print(delta)
    print(f'Converged after {i} iterations with delta={delta}')

    plt.subplot(2, 2, 1)
    plt.imshow(potential_field, extent=(x_range[0], x_range[1], y_range[1], y_range[0]), origin='upper', cmap='viridis')
    plt.colorbar()
    plt.title('Potential Field')
    field_x, field_y = calculate_field_from_potential(potential_field)
    with open('field_x.pkl', 'wb') as f:
        pickle.dump(field_x, f)
    with open('field_y.pkl', 'wb') as f:
        pickle.dump(field_y, f)
    with open('xs.pkl', 'wb') as f:
        pickle.dump(xs, f)
    with open('ys.pkl', 'wb') as f:
        pickle.dump(ys, f)

    plt.subplot(2, 2, 3)
    plt.imshow(field_x, extent=(x_range[0], x_range[1], y_range[1], y_range[0]), origin='upper', cmap='viridis')
    plt.colorbar()
    plt.title('Field in x-direction')
    plt.subplot(2, 2, 4)
    plt.imshow(field_y, extent=(x_range[0], x_range[1], y_range[1], y_range[0]), origin='upper', cmap='viridis')
    plt.colorbar()
    plt.title('Field in y-direction')
    plt.show()

if __name__ == "__main__":
    main()