import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_equilateral_triangle(width):
    """
    Plots an equilateral triangle on the xy-plane, centered at the origin.

    Parameters:
        width (float): The length of one side of the equilateral triangle.
    """
    # Calculate the height of the equilateral triangle
    height = width/np.sqrt(3)  # height of an equilateral triangle
    theta = np.pi/6

    A = np.array([0, height, 0])
    B = np.array([height * np.cos(theta),
                  -height * np.sin(theta),
                  0])
    C = np.array([-height * np.cos(theta),
                  -height * np.sin(theta),
                  0])

    # List of vertices
    vertices = [A, B, C]

    # Create a figure and 3D axis
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the triangle
    triangle = Poly3DCollection([vertices], color='gray', linewidths=1, edgecolors='r', alpha=1.0)#0.5)
    ax.add_collection3d(triangle)

    ax.text(*A, 'A', color='black', fontsize=10)
    ax.text(*B, 'B', color='black', fontsize=10)
    ax.text(*C, 'C', color='black', fontsize=10)

    # Set the limits for the plot
    ax.set_xlim([-width, width])
    ax.set_ylim([-width, width])
    ax.set_zlim([0, 1])

    # Labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title(f'Equilateral Triangle with Width {width} on the XY Plane')

    # Show the plot
    plt.show()

# Call the function to plot the triangle with a width of 5
# plot_equilateral_triangle(4)


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# def plot_cylinder(ax, radius=1, height=1, center=(0, 0), resolution=50, color='b', alpha=0.5):
#     """
#     Plots a solid 3D cylinder with caps on the given Axes3D object.

#     Parameters:
#         ax (Axes3D): The 3D axes to plot the cylinder on.
#         radius (float): The radius of the cylinder.
#         height (float): The height of the cylinder.
#         center (tuple): The (x, y) center of the cylinder's base.
#         resolution (int): The number of segments for the cylinder's surface.
#         color (str): The color of the cylinder.
#         alpha (float): The transparency of the cylinder.
#     """
#     # Generate the data for the cylinder surface
#     x_center, y_center = center
#     z = np.linspace(0, height, resolution)
#     theta = np.linspace(0, 2 * np.pi, resolution)
#     theta_grid, z_grid = np.meshgrid(theta, z)
    
#     x = x_center + radius * np.cos(theta_grid)
#     y = y_center + radius * np.sin(theta_grid)
#     z = z_grid

#     # Plot the surface of the cylinder
#     ax.plot_surface(x, y, z, color=color, alpha=alpha)

#     # Create the top and bottom caps as solid disks
#     circle_theta = np.linspace(0, 2 * np.pi, resolution)
#     circle_x = x_center + radius * np.cos(circle_theta)
#     circle_y = y_center + radius * np.sin(circle_theta)
    
#     # Bottom cap (z=0)
#     vertices_bottom = np.array([circle_x, circle_y, np.zeros_like(circle_x)]).T
#     ax.add_collection3d(Poly3DCollection([vertices_bottom], color=color, alpha=alpha))
    
#     # Top cap (z=height)
#     vertices_top = np.array([circle_x, circle_y, np.ones_like(circle_x) * height]).T
#     ax.add_collection3d(Poly3DCollection([vertices_top], color=color, alpha=alpha))

# # Example usage
# fig = plt.figure(figsize=(8, 6))
# ax = fig.add_subplot(111, projection='3d')

# # Plot a cylinder
# plot_cylinder(ax, radius=1, height=3, center=(0, 0), resolution=50, color='cyan', alpha=1.0)

# # Adjust axes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_box_aspect([1, 1, 3])  # Adjust aspect ratio

# plt.show()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_rectangular_prism(ax, corner=(0, 0, 0), width=1, depth=1, height=1, color='b', alpha=0.5):
    """
    Plots a 3D rectangular prism on the given Axes3D object.

    Parameters:
        ax (Axes3D): The 3D axes to plot the prism on.
        corner (tuple): Coordinates of the corner of the prism (x, y, z).
        width (float): The width of the prism (along the x-axis).
        depth (float): The depth of the prism (along the y-axis).
        height (float): The height of the prism (along the z-axis).
        color (str): The color of the prism.
        alpha (float): The transparency of the prism's surface.
    """
    x, y, z = corner

    # Define vertices of the rectangular prism
    vertices = [
        [x, y, z],
        [x + width, y, z],
        [x + width, y + depth, z],
        [x, y + depth, z],
        [x, y, z + height],
        [x + width, y, z + height],
        [x + width, y + depth, z + height],
        [x, y + depth, z + height]
    ]

    # Define the 6 faces of the prism
    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # Bottom
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # Top
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # Front
        [vertices[4], vertices[5], vertices[6], vertices[7]]   # Back
    ]

    # Plot each face of the prism
    poly3d = Poly3DCollection(faces, alpha=alpha, facecolors=color, edgecolor='k')
    ax.add_collection3d(poly3d)

# Example usage
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot a rectangular prism
plot_rectangular_prism(ax, corner=(0, 0, 0), width=2, depth=1, height=3, color='cyan', alpha=0.7)

# Adjust axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_box_aspect([2, 1, 3])  # Adjust aspect ratio
plt.show()
