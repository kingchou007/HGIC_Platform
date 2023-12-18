import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.axes_grid1 import make_axes_locatable

name = "vc_"
trajectories = np.loadtxt("vc_trajectories.csv", delimiter=",")
fig, axs = plt.subplots(2, 2, figsize=(12, 10))
key_frames = [1, 100, 250, 800]
cmap = plt.get_cmap("summer")

for idx, t in enumerate(key_frames):
    ax = axs[idx // 2, idx % 2]
    for i in range(0, trajectories.shape[1], 2):
        x = trajectories[:t, i]
        y = trajectories[:t, i + 1]
        ax.scatter(x[0], y[0], s=10, zorder=2)
        for j in range(1, len(x)):
            ax.plot(
                x[j - 1 : j + 1], y[j - 1 : j + 1], color=cmap(j / len(x)), linewidth=2
            )

        ax.scatter(x[-1], y[-1], s=10, zorder=2)
        circle_radius = 20
        circle_radius_2 = 7
        ax.scatter(
            x[-1],
            y[-1],
            s=np.pi * (circle_radius**2),
            color="silver",
            alpha=0.5,
            zorder=1,
        )
        ax.scatter(
            x[-1],
            y[-1],
            s=np.pi * (circle_radius_2**2),
            color="lightgreen",
            alpha=0.5,
            zorder=1,
        )
        plt.setp(ax.get_xticklabels(), visible=False)
        plt.setp(ax.get_yticklabels(), visible=False)

    ax.spines[["right", "top"]].set_visible(False)
    norm = mcolors.Normalize(vmin=0, vmax=key_frames[-1])
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = fig.colorbar(sm, cax=cax, orientation="vertical")
    cbar.ax.set_yticklabels([])

plt.tight_layout()
plt.savefig(name + "key_frames.png", dpi=600)
plt.show()
