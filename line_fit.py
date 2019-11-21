import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np

# def func(x, a, b, c):
#     return a*np.exp(-b * x) + c
#
# def func_quadratic(x, a, b, c):
#     return a * np.square(x) + b * x + c

# xdata = np.linspace(0, 4, 6)
# xdata = np.array([0, 1, 3, 2, 4, 5])
# y = func(xdata, 2.5, 1.3, 0.5)
# np.random.seed(1729)
# y_noise = 0.2 * np.random.normal(size=xdata.size)
# ydata = np.array([1, 2, 4, 3, 5, 6])
# plt.plot(xdata, ydata, 'b-', label='data')
# popt, pcov = curve_fit(func_quadratic, xdata, ydata)
# new_y = func_quadratic(xdata, *popt)
# print(new_y)
# plt.plot(xdata, new_y, 'g--', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))
# plt.show()

class line_fit:

    def func_quadratic(self, x, a, b, c):
        return a * np.square(x) + b * x + c


    def fit(self, trajectory, point_start=None, point_end=None):
        if point_start is None:
            point_start = trajectory[0][:-1]
            point_end = trajectory[-1][:-1]
        vec = point_end - point_start
        x_vec = np.array([1, 0])
        dot_prod = np.dot(vec, x_vec)
        theta = np.arccos(dot_prod/np.linalg.norm(vec))
        # transform matrix for the new origin
        trans_mat = np.array([[np.cos(theta), -np.sin(theta), point_start[0]],
                              [np.sin(theta), np.cos(theta), point_end[1]],
                              [0,             0,              1]])
        # inverse of new origin transform dot point transform is the transform wrt new origin
        trans_to_local = np.linalg.inv(trans_mat)
        local_points = []
        for i in range(len(trajectory)):
            point_global = np.array([[trajectory[i][0]],
                                    [trajectory[i][1]],
                                    [1]])
            point_local = np.dot(trans_to_local, point_global)
            local_points.append(point_local)
        x_data = np.zeros(len(local_points))
        y_data = np.zeros(len(local_points))
        for i in range(len(local_points)):
            x_data[i] = local_points[i][0]
            y_data[i] = local_points[i][1]

        popt, pcov = curve_fit(self.func_quadratic, x_data, y_data)
        new_y = self.func_quadratic(x_data, *popt)
        # reassigned smooth value
        for i in range(len(local_points)):
            local_points[i][1] = new_y[i]

        # transform local points back to global
        smooth_global = []
        for i in range(len(local_points)):
            global_pos = np.dot(trans_mat, local_points[i])
            row = np.zeros(3)
            row[0] = global_pos[0]
            row[1] = global_pos[1]
            row[2] = trajectory[i][2]
            smooth_global.append(row)


        return smooth_global


        # plt.plot(x_data, self.func_quadratic(x_data, *popt), 'g--', label='fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))
        # plt.plot(x_data, y_data, 'b-', label='data')
        # plt.show()