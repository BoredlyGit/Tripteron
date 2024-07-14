import numpy as np


class Plane:
    def __init__(self, normal_vec, pt):
        """
        Represents a plane with the form ax + by + cz = d

        :param normal_vec: normal vector of the plane (a, b, c)
        :param pt: point on the plane (x, y, z)
        """
        self.normal_vec = normal_vec
        self._x = pt[0]
        self._y = pt[1]
        self._z = pt[2]
        self.d = normal_vec[0]*self._x + normal_vec[1]*self._y + normal_vec[2]*self._z

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, val):
        """done so that any assignement to x triggers a recalculation of d"""
        self._x = val
        self.d = self.normal_vec[0]*self._x + self.normal_vec[1] * self._y + self.normal_vec[2] * self._z

    @staticmethod
    def intersect(p1, p2, p3):
        """
        Calculates intersection of 3 planes with:

        inv([a1 b1 c1  *  [d1  =  [x
             a2 b2 c2      d2      y
             a3 b3 c3])    d3]     z]

        :param p1: plane 1
        :param p2: plane 2
        :param p3: plane 3
        :return: (x, y , z) intersect of 3 planes

        (Formula from https://www.youtube.com/watch?v=DAE3EEoFryM)
        """
        mat = np.vstack((p1.normal_vec, p2.normal_vec, p3.normal_vec))
        vec = np.vstack((p1.d, p2.d, p3.d))
        return np.matmul(np.linalg.inv(mat), vec)


# if __name__ == "__main__":
#     # test code for generating the transform matrix
#     p1 = Plane((1, 1, 1), (180, 44.593, 0))
#     p2 = Plane((1, -1, 1), (180, -44.593, 0))
#     p3 = Plane((-1, 0, 1), (-100, 0, -2.5))
#
#     init = Plane.intersect(p1, p2, p3)
#
#     p1 = Plane((1, 1, 1), (181, 44.593, 0))
#     p2 = Plane((1, -1, 1), (180, -44.593, 0))
#     p3 = Plane((-1, 0, 1), (-100, 0, -2.5))
#
#     p1_delta = init - Plane.intersect(p1, p2, p3)
#     print(p1_delta)
#
#     p1 = Plane((1, 1, 1), (180, 44.593, 0))
#     p2 = Plane((1, -1, 1), (181, -44.593, 0))
#     p3 = Plane((-1, 0, 1), (-100, 0, -2.5))
#
#     p2_delta = init - Plane.intersect(p1, p2, p3)
#     print(p2_delta)
#
#     p1 = Plane((1, 1, 1), (180, 44.593, 0))
#     p2 = Plane((1, -1, 1), (180, -44.593, 0))
#     p3 = Plane((-1, 0, 1), (-99, 0, -2.5))
#
#     p3_delta = init - Plane.intersect(p1, p2, p3)
#     print(p3_delta)
#
#     """mat * vec(p1, p2, p3) = vec(x, y ,z)"""
#     mat = np.hstack((p1_delta,  p2_delta, p3_delta))
#     print(mat)
#
#     # print(np.matmul(mat, np.array((-1, -1, 1))))
#     # print(np.matmul(mat, np.array(([-1], [-1], [1]))))
#
#     map = np.linalg.inv(mat)
#     print(map)
#     print(np.matmul(map, np.array((-1, 0, 0))))
#
#
