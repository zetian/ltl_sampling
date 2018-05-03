from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("equal")

# draw cube
def rect_prism(x_range, y_range, z_range):
    # TODO: refactor this to use an iterotor
    xx, yy = np.meshgrid(x_range, y_range)
    ax.plot_wireframe(xx, yy, z_range[0], color="r")
    ax.plot_surface(xx, yy, z_range[0], color="r", alpha=0.2)
    ax.plot_wireframe(xx, yy, z_range[1], color="r")
    ax.plot_surface(xx, yy, z_range[1], color="r", alpha=0.2)


    yy, zz = np.meshgrid(y_range, z_range)
    ax.plot_wireframe(x_range[0], yy, zz, color="r")
    ax.plot_surface(x_range[0], yy, zz, color="r", alpha=0.2)
    ax.plot_wireframe(x_range[1], yy, zz, color="r")
    ax.plot_surface(x_range[1], yy, zz, color="r", alpha=0.2)

    xx, zz = np.meshgrid(x_range, z_range)
    ax.plot_wireframe(xx, y_range[0], zz, color="r")
    ax.plot_surface(xx, y_range[0], zz, color="r", alpha=0.2)
    ax.plot_wireframe(xx, y_range[1], zz, color="r")
    ax.plot_surface(xx, y_range[1], zz, color="r", alpha=0.2)


rect_prism(np.array([-1, 1]), np.array([-1, 1]), np.array([-0.5, 0.5]))
plt.show()