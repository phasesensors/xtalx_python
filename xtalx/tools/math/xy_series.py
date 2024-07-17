import numpy as np


class XYSeries:
    def __init__(self, X, Y):
        assert len(X) == len(Y)
        self.X = np.array(X)
        self.Y = np.array(Y)

    def _integrate(self, x0, x1):
        assert x0 <= x1

        x0, x1 = coords = np.clip([x0, x1], self.X[0], self.X[-1])
        yL, yH = np.interp(coords, self.X, self.Y)
        iL, iH = np.searchsorted(self.X, coords)
        X      = np.concatenate(([x0], self.X[iL:iH], [x1]))
        Y      = np.concatenate(([yL], self.Y[iL:iH], [yH]))
        A      = np.trapz(Y, x=X)

        return A, x0, x1

    def integrate(self, x0, x1):
        '''
        Integrate the data over the range [x0, x1] using the trapezoidal rule.
        The endpoints need not coincide with the sample data; the integration
        will use linear interpolation to integrate just the part of the
        boundary trapezoid that lies within the range.
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
        then None is returned.
        '''
        area, x0, x1 = self._integrate(x0, x1)
        if x0 != x1:
            return area / (x1 - x0)
        return None

    def interpolate(self, x):
        '''
        Given an X value, interpolate what the Y value would be.  The X value
        is clipped to the range of the series.
        '''
        return np.interp(x, self.X, self.Y)

    def append(self, x, y):
        self.X = np.append(self.X, x)
        self.Y = np.append(self.Y, y)


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
