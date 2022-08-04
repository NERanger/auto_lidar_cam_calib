from cProfile import label
from typing import List, Tuple

import matplotlib.pyplot as plt

import os

trans_plot_lim_margin = 0.4
rot_plot_lim_margin = 0.5

class ErrData(object):
    def __init__(self) -> None:
        self.x = []
        self.err_x_t = []
        self.err_y_t = []
        self.err_z_t = []
        self.err_x_r = []
        self.err_y_r = []
        self.err_z_r = []

    def TransErrLim(self) -> Tuple[float, float]:
        return max([max(self.err_x_t), max(self.err_y_t), max(self.err_z_t)]) + trans_plot_lim_margin, \
               min([min(self.err_x_t), min(self.err_y_t), min(self.err_z_t)]) - trans_plot_lim_margin

    def RotErrLim(self) -> Tuple[float, float]:
        return max([max(self.err_x_r), max(self.err_y_r), max(self.err_z_r)]) + rot_plot_lim_margin, \
               min([min(self.err_x_r), min(self.err_y_r), min(self.err_z_r)]) - rot_plot_lim_margin

    def Plot(self):
        (trans_lower_lim, trans_upper_lim) = self.TransErrLim()
        (rot_lower_lim, rot_upper_lim) = self.RotErrLim()

        plt.subplot(2, 3, 1)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (x-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_x_t)

        plt.subplot(2, 3, 2)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (y-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_y_t)

        plt.subplot(2, 3, 3)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (z-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_z_t)

        plt.subplot(2, 3, 4)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (x-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_x_r)

        plt.subplot(2, 3, 5)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (y-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_y_r)

        plt.subplot(2, 3, 6)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (z-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        plt.plot(self.x, self.err_z_r)

        plt.show()

class SingleRunErr(object):
    def __init__(self, file:str) -> None:
        self.err_data = ErrData()

        self.trans_upper_lim = 0.0
        self.trans_lower_lim = 0.0
        self.rot_upper_lim = 0.0
        self.rot_lower_lim = 0.0

        self.LoadFromFile(file)

    def LoadFromFile(self, file:str):
        with open(file) as f:
            lines = f.readlines()
            for line in lines:
                if line[0] == '#':
                    continue
                data = line.split()
                self.err_data.x.append(int(data[0]))
                self.err_data.err_x_t.append(float(data[1]))
                self.err_data.err_y_t.append(float(data[2]))
                self.err_data.err_z_t.append(float(data[3]))
                self.err_data.err_x_r.append(float(data[4]))
                self.err_data.err_y_r.append(float(data[5]))
                self.err_data.err_z_r.append(float(data[6]))

        self.trans_upper_lim, self.trans_lower_lim = self.err_data.TransErrLim()

        self.rot_upper_lim, self.rot_lower_lim = self.err_data.RotErrLim()

class MultiRunErr(object):
    def __init__(self, files:List[str]) -> None:
        self.err_list = list()
        self.name_list = list()

        for file in files:
            self.err_list.append(SingleRunErr(file))
            self.name_list.append(os.path.basename(file))
    
    def Plot(self):
        trans_lower_lim = -0.3
        trans_upper_lim = 0.3
        rot_lower_lim = -3.0
        rot_upper_lim = 3.0

        plt.subplot(2, 3, 1)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (x-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_x_t, label = name)
        plt.legend()

        plt.subplot(2, 3, 2)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (y-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_y_t, label = name)
        plt.legend()

        plt.subplot(2, 3, 3)
        plt.ylim(trans_lower_lim, trans_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Translation Error (z-axis) in meter')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_z_t, label = name)
        plt.legend()

        plt.subplot(2, 3, 4)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (x-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_x_r, label = name)
        plt.legend()

        plt.subplot(2, 3, 5)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (y-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_y_r, label = name)
        plt.legend()

        plt.subplot(2, 3, 6)
        plt.ylim(rot_lower_lim, rot_upper_lim)
        plt.xlabel('Total Used Frames')
        plt.ylabel('Rotation Error (z-axis) in degree')
        plt.axhline(y=0, color='g', ls='--')
        for i in range(len(self.err_list)):
            err = self.err_list[i]
            name = self.name_list[i]
            plt.plot(err.err_data.x, err.err_data.err_z_r, label = name)
        plt.legend()

        plt.show()