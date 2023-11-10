import numpy as np


class XYSeries:
    def __init__(self, X, Y):
        assert len(X) == len(Y)
        self.X = np.array(X)
        self.Y = np.array(Y)

    def get_avg_value(self, x0, x1):
        indices = (self.X >= x0) & (self.X <= x1)
        X       = self.X[indices]
        Y       = self.Y[indices]
        if len(X) == 0:
            return None
        if len(X) == 1:
            return Y[0]

        area_2  = 0
        for i in range(len(X) - 1):
            area_2 += (Y[i + 1] + Y[i]) * (X[i + 1] - X[i])

        return area_2 / (2 * (X[-1] - X[0]))

    def append(self, x, y):
        self.X = np.append(self.X, x)
        self.Y = np.append(self.Y, y)
