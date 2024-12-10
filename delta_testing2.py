import numpy as np
import matplotlib.pyplot as plt

# Define the grid and the function
x = np.linspace(0, 10, 100)
y = np.linspace(0, 5, 50)
X, Y = np.meshgrid(x, y)
Z = np.sin(X) + np.cos(Y)

# Compute the derivatives
# dZ_dx, dZ_dy = np.gradient(Z, x, y)  # Compute gradient with respect to x and y
dZ_dx, dZ_dy = np.gradient(Z, axis=0), np.gradient(Z, axis=1)

# Plot the function and derivatives using quiver
plt.figure(figsize=(10, 6))

# Contour plot of the original function
plt.contourf(X, Y, Z, levels=50, cmap='viridis', alpha=0.8)
plt.colorbar(label='Function Value (Z)')

# Quiver plot for derivatives
plt.quiver(X, Y, dZ_dx, dZ_dy, color='white', scale=50, headwidth=3)

# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Derivatives of $Z = \sin(X) + \cos(Y)$')
plt.grid(True)
plt.show()
