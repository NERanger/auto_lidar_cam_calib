import matplotlib
import matplotlib.pyplot as plt

import glob

from ErrData import ErrData
from ErrData import MultiRunErr

files = glob.glob('E:\\exp_data\\comp_exp\err_*.txt')

if __name__ == '__main__':
    err = MultiRunErr(files)

    err.Plot()