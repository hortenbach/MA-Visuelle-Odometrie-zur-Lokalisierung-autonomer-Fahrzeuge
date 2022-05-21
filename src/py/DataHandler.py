import cv2 as cv
import math
import numpy as np
import matplotlib.pyplot as plt

# class DataHandler:

#   def __init__(self):
#     self.img_A = []
#     self.img_B = []
#     self.gray_img_A = []
#     self.gray_img_B = []

#     self.DIR = "/home/hortenbach/workspace/code_ws/pictures"
#   def read(self, start, stop):
#     for i in range(start, stop+0):
#       print(i)
#       self.img_A.append(self.__read_img(i, 'left'))
#       self.img_B.append(self.__read_img(i, 'right'))
#       self.gray_img_A.append(self.__read_img_gray(i, 'left'))
#       self.gray_img_B.append(self.__read_img_gray(i, 'right'))

#   def __read_img(self, num, cam):
#     img = cv.imread(f"/home/hortenbach/workspace/MA_v1/py/pic/image_{cam}_{num}.jpg")
#     return img

#   def __read_img_gray(self, num, cam):
#     gray = cv.imread(f"/home/hortenbach/workspace/MA_v1/py/pic/image_{cam}_{num}.jpg", 0)
#     # gray = np.float31(gray)
#     return gray


class DataHandler:
    def __init__(self, start, stop, pic_directory):
        self.gray_img_A = []
        self.gray_img_B = []
        self.depth = []

        # self.DIR = "/home/hortenbach/workspace/code_ws/pictures"
        self.DIR = pic_directory

        for i in range(start, stop + 0):
            # print(i)
            self.gray_img_A.append(self.read_img_gray(i, "left"))
            self.gray_img_B.append(self.read_img_gray(i, "right"))
            self.depth.append(self.read_img(i, "depth"))

    def read_img(self, num, cam):
        img = cv.imread(f"{self.DIR}/{cam}/image_{cam}_{num}.jpg")
        return img

    def read_img_gray(self, num, cam):
        gray = cv.imread(f"{self.DIR}/{cam}/image_{cam}_{num}.jpg", 0)
        # gray = np.float31(gray)
        return gray

    def visualize_trajectory(trajectory):
        # Unpack X Y Z each trajectory point
        locX = []
        locY = []
        locZ = []
        # This values are required for keeping equal scale on each plot.
        # matplotlib equal axis may be somewhat confusing in some situations because of its various scale on
        # different axis on multiple plots
        max = -math.inf
        min = math.inf

        # Needed for better visualisation
        maxY = -math.inf
        minY = math.inf

        for i in range(0, trajectory.shape[1]):
            current_pos = trajectory[:, i]

            locX.append(current_pos.item(0))
            locY.append(current_pos.item(1))
            locZ.append(current_pos.item(2))
            if np.amax(current_pos) > max:
                max = np.amax(current_pos)
            if np.amin(current_pos) < min:
                min = np.amin(current_pos)

            if current_pos.item(1) > maxY:
                maxY = current_pos.item(1)
            if current_pos.item(1) < minY:
                minY = current_pos.item(1)

        auxY_line = locY[0] + locY[-1]
        if max > 0 and min > 0:
            minY = auxY_line - (max - min) / 2
            maxY = auxY_line + (max - min) / 2
        elif max < 0 and min < 0:
            minY = auxY_line + (min - max) / 2
            maxY = auxY_line - (min - max) / 2
        else:
            minY = auxY_line - (max - min) / 2
            maxY = auxY_line + (max - min) / 2

        # Set styles
        mpl.rc("figure", facecolor="white")
        plt.style.use("seaborn-whitegrid")

        # Plot the figure
        fig = plt.figure(figsize=(8, 6), dpi=100)
        gspec = gridspec.GridSpec(3, 3)
        ZY_plt = plt.subplot(gspec[0, 1:])
        YX_plt = plt.subplot(gspec[1:, 0])
        traj_main_plt = plt.subplot(gspec[1:, 1:])
        D3_plt = plt.subplot(gspec[0, 0], projection="3d")

        # Actual trajectory plotting ZX
        toffset = 1.06
        traj_main_plt.set_title("Autonomous vehicle trajectory (Z, X)", y=toffset)
        traj_main_plt.set_title("Trajectory (Z, X)", y=1)
        traj_main_plt.plot(
            locZ, locX, ".-", label="Trajectory", zorder=1, linewidth=1, markersize=4
        )
        traj_main_plt.set_xlabel("Z")
        # traj_main_plt.axes.yaxis.set_ticklabels([])
        # Plot reference lines
        traj_main_plt.plot(
            [locZ[0], locZ[-1]],
            [locX[0], locX[-1]],
            "--",
            label="Auxiliary line",
            zorder=0,
            linewidth=1,
        )
        # Plot camera initial location
        traj_main_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
        traj_main_plt.set_xlim([min, max])
        traj_main_plt.set_ylim([min, max])
        traj_main_plt.legend(
            loc=1, title="Legend", borderaxespad=0.0, fontsize="medium", frameon=True
        )

        # Plot ZY
        # ZY_plt.set_title("Z Y", y=toffset)
        ZY_plt.set_ylabel("Y", labelpad=-4)
        ZY_plt.axes.xaxis.set_ticklabels([])
        ZY_plt.plot(locZ, locY, ".-", linewidth=1, markersize=4, zorder=0)
        ZY_plt.plot(
            [locZ[0], locZ[-1]],
            [(locY[0] + locY[-1]) / 2, (locY[0] + locY[-1]) / 2],
            "--",
            linewidth=1,
            zorder=1,
        )
        ZY_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
        ZY_plt.set_xlim([min, max])
        ZY_plt.set_ylim([minY, maxY])

        # Plot YX
        # YX_plt.set_title("Y X", y=toffset)
        YX_plt.set_ylabel("X")
        YX_plt.set_xlabel("Y")
        YX_plt.plot(locY, locX, ".-", linewidth=1, markersize=4, zorder=0)
        YX_plt.plot(
            [(locY[0] + locY[-1]) / 2, (locY[0] + locY[-1]) / 2],
            [locX[0], locX[-1]],
            "--",
            linewidth=1,
            zorder=1,
        )
        YX_plt.scatter([0], [0], s=8, c="red", label="Start location", zorder=2)
        YX_plt.set_xlim([minY, maxY])
        YX_plt.set_ylim([min, max])

        # Plot 3D
        D3_plt.set_title("3D trajectory", y=toffset)
        D3_plt.plot3D(locX, locZ, locY, zorder=0)
        D3_plt.scatter(0, 0, 0, s=8, c="red", zorder=1)
        D3_plt.set_xlim3d(min, max)
        D3_plt.set_ylim3d(min, max)
        D3_plt.set_zlim3d(min, max)
        D3_plt.tick_params(direction="out", pad=-2)
        D3_plt.set_xlabel("X", labelpad=0)
        D3_plt.set_ylabel("Z", labelpad=0)
        D3_plt.set_zlabel("Y", labelpad=-2)

        # plt.axis('equal')
        D3_plt.view_init(45, azim=30)
        plt.tight_layout()
        plt.grid(False)
        plt.show()
