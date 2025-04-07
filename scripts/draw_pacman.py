import numpy as np
import matplotlib.pyplot as plt

def generate_shape(x, y, angle, radius, n=10):
    # x, y is the center of the shape
    # angle is the shape of the mouth
    theta = np.linspace(angle, 2*np.pi-angle, n)
    xs = x + radius * np.cos(theta)
    ys = y + radius * np.sin(theta)
    # add the center of the shape to close the mouth
    xs = np.concatenate(([x], xs, [x]))
    ys = np.concatenate(([y], ys, [y]))
    return xs, ys


def draw_pacman(x, y, angle, radius, n=10):
    xs, ys = generate_shape(x, y, angle, radius, n)
    for (x, y) in zip(xs, ys):
        print(f'<Vertex x="{x}" y="{y}"/>')

    for (x, y) in zip(xs, ys):
        print(f'Vertex<T>({x}, {y}),')

    # plt.fill(xs, ys, 'yellow')
    # plt.plot(xs, ys, 'black')
    # plt.axis('equal')
    # plt.axis('off')
    # plt.show()

    
if __name__ == "__main__":
    draw_pacman(0, 0, np.pi/6, 1)