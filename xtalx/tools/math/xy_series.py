import numpy as np


class XYSeries:
    def __init__(self, X, Y):
        self.X = np.array(X)
        self.Y = np.array(Y)
        assert self.X.size == self.Y.size

    def __len__(self):
        return self.X.size

    def __bool__(self):
        return len(self) != 0

    def _integrate(self, x0, x1):
        assert x0 <= x1

        x0, x1 = coords = np.clip([x0, x1], self.X[0], self.X[-1])
        yL, yH = np.interp(coords, self.X, self.Y)
        iL, iH = np.searchsorted(self.X, coords)
        X      = np.concatenate(([x0], self.X[iL:iH], [x1]))
        Y      = np.concatenate(([yL], self.Y[iL:iH], [yH]))
        A      = np.trapezoid(Y, x=X)

        return A, x0, x1

    def integrate(self, x0, x1):
        '''
        Integrate the data over the range [x0, x1] using the trapezoidal rule.
        The endpoints need not coincide with the sample data; the integration
        will use linear interpolation to integrate just the part of the
        boundary trapezoid that lies within the range.  The data points must be
        in sorted X order.
        '''
        if x0 <= x1:
            return self._integrate(x0, x1)[0]
        return -self._integrate(x1, x0)[0]  # pylint: disable=W1114

    def get_avg_value(self, x0, x1):
        '''
        Compute the average value of the data over the range [x0, x1].  If the
        range extends past the endpoints of the data set, the x0 and x1 values
        will be clipped to the data set endpoints before computing the average
        value.  If the average value is undefined (x0 == x1 after clipping)
        then None is returned.  The data points must be in sorted X order.
        '''
        area, x0, x1 = self._integrate(x0, x1)
        if x0 != x1:
            return area / (x1 - x0)
        return None

    def interpolate(self, x):
        '''
        Given an X value, interpolate what the Y value would be.  The X value
        is clipped to the range of the series.  The data points must be in
        sorted X order, and the X values must all be strictly increasing so
        that there are no vertical line segments in the data.
        '''
        return np.interp(x, self.X, self.Y)

    def interpolate_all(self, x):
        '''
        Given an X value, return a list of Y values (y0, y1, ..., yN) such that
        all the points (X, yi) are on one of the line segments joining points
        in the series.  If a series has vertical segments [(xi == xj) for
        j == i + 1] then the midpoint of the segment is returned.  The data
        points need not be in sorted order so this is appropraite for a data
        set that may loop back over itself.

        Since this must iterate over all points in the data, it is an O(N)
        method.
        '''
        Y = []
        for i in range(len(self) - 1):
            x0 = self.X[i]
            x1 = self.X[i + 1]
            # if x0 < x1:
            #     if x0 <= x < x1:
            #         Y.append(np.interp(x, (x0, x1), (self.Y[i], self.Y[i + 1])))
            # elif x0 > x1:
            #     if x1 <= x < x0:
            #         Y.append(np.interp(x, (x1, x0), (self.Y[i + 1], self.Y[i])))
            # else:
            #     Y.append((self.Y[i] + self.Y[i + 1]) / 2)

            if abs((x0 - x) + (x1 - x)) < abs(x1 - x0):
                y0 = self.Y[i]
                y1 = self.Y[i + 1]
                if x0 == x1:
                    Y.append((y0 + y1) / 2)
                else:
                    Y.append(y0 + (x - x0) * (y1 - y0) / (x1 - x0))
        return Y

    def append(self, x, y):
        '''
        Appends (x, y) to the end of the array.  Makes no attempt to sort
        anything.
        '''
        self.X = np.append(self.X, x)
        self.Y = np.append(self.Y, y)

    def insert(self, x, y):
        '''
        Inserts (x, y) in the array, maintaining X as a sorted list.  This, of
        course, is nonsense if the existing data points aren't already sorted
        by X value.
        '''
        i = np.searchsorted(self.X, x)
        self.X = np.insert(self.X, i, x)
        self.Y = np.insert(self.Y, i, y)

    def delete(self, i):
        '''
        Removes the i-th element from the series.
        '''
        self.X = np.delete(self.X, i)
        self.Y = np.delete(self.Y, i)

    def subseries(self, x0, x1):
        assert x0 <= x1

        x0, x1 = coords = np.clip([x0, x1], self.X[0], self.X[-1])
        iL, iH = np.searchsorted(self.X, coords)
        X      = self.X[iL:iH]
        Y      = self.Y[iL:iH]
        return XYSeries(X, Y)


if __name__ == '__main__':
    s = XYSeries([1, 2, 3], [4, 5, 6])
    assert s.integrate(1, 1) == 0
    assert s.integrate(0, 1) == 0
    assert s.integrate(0, 0.5) == 0
    assert s.integrate(3, 4) == 0
    assert s.integrate(3.5, 4) == 0
    assert s.integrate(0, 4) == 10
    assert s.integrate(1, 2) == 4.5
    assert s.integrate(1, 3) == 10
    assert s.integrate(2, 3) == 5.5
    assert s.integrate(1.5, 2.5) == 5
    assert s.integrate(1.5, 1.5) == 0

    s = XYSeries([0, 2, 3, 4], [2, 3, 5, 4])
    assert s.integrate(-1, 3) == 9
    assert s.integrate(-1, -0.5) == 0
    assert s.integrate(3, 5) == 4.5
    assert s.integrate(4.5, 5) == 0
    assert s.integrate(-1, 5) == 13.5
    assert s.integrate(0, 1) == 2.25
    assert s.integrate(0, 2) == 5
    assert s.integrate(0, 3) == 9
    assert s.integrate(0, 4) == 13.5
    assert s.integrate(0.5, 1.5) == 2.5
    assert s.integrate(0.5, 2.5) == 5.6875
    assert s.integrate(2.5, 0.5) == -5.6875

    print('Tests succeeded.')
