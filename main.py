from math_utils import Plane
import numpy as np


class Slider(Plane):
    def __init__(self, *args):
        super().__init__(*args)
        self.x_max = 0


class Tripteron:
    def __init__(self, p1: Slider, p2: Slider, p3: Slider, head_z_offset, slider_dist_range):
        """
        is tha robot!!

        :param p1: Slider 1 (typically normal vector (1, 1, 1))
        :param p2: Slider 2 (typically normal vector (1, -1, 1))
        :param p3: Slider 3 (typically normal vector (-1, 0, 1))
        :param head_z_offset: z offset from the intersection of the 3 planes to the head
        :param slider_dist_range: min/max distance to maintain between p1/p1 and p3 (so the arms don't get tangled up)
        """
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.x, self.y, self.z = Plane.intersect(p1, p2, p3).flatten()
        self.z += head_z_offset
        self.slider_dist_range = slider_dist_range
        self.transform_mat = self.generate_transform_matrix()

    def generate_transform_matrix(self):
        """
        Generates a matrix that maps changes in (x, y, z) of the head into x-movements for (p1, p2, p3)

        Steps:
        1. Calculate the effect of a +1 change in x of each plane on the intersection point. --> column vectors of
           change in (x, y, z)

        2. Horizontally stack column vectors to get a matrix that transforms changes in (p1, p2, p3) into changes
           in (x, y, z)
           - The column vectors/horizontal stacking makes sense when doing the matrix multiplication - each row is
             a coefficient of either x/y/z in its respective column "equation"

        3. Invert the matrix to get a matrix that transforms changes in (x, y, z) into changes in (p1, p2, p3)

        :return: matrix that transforms changes in (x, y, z) into changes in (p1, p2, p3)
        """
        init = Plane.intersect(self.p1, self.p2, self.p3)

        self.p1.x += 1
        p1_delta = init - Plane.intersect(self.p1, self.p2, self.p3)
        self.p1.x -= 1

        self.p2.x += 1
        p2_delta = init - Plane.intersect(self.p1, self.p2, self.p3)
        self.p2.x -= 1

        self.p3.x += 1
        p3_delta = init - Plane.intersect(self.p1, self.p2, self.p3)
        self.p3.x -= 1

        mat = np.hstack((p1_delta, p2_delta, p3_delta))
        print(mat)
        inv = np.linalg.inv(mat)
        return inv

    def move(self, x, y, z):
        p1_delta, p2_delta, p3_delta = np.matmul(self.transform_mat, np.array((self.x-x, self.y-y, self.z-z)))

        # TODO: Do checks to make sure the machine doesn't destroy itself
        # TODO: send commands to grbl

        self.p1.x += p1_delta
        self.p2.x += p2_delta
        self.p3.x += p3_delta
        self.x = x
        self.y = y
        self.z = z

        return x, y, z


if __name__ == "__main__":
    robot = Tripteron(Slider((1, 1, 1), (376.5, 44.593, 0)),  # starting position coordinates from cad
                      Slider((1, -1, 1), (376.5, -44.593, 0)),
                      Slider((-1, 0, 1), (66.376, 0, -2.5)),
                      head_z_offset=-56.560,  # calculated by setting to 0 to get intersect, then subtracting cad z
                      slider_dist_range=(100, 1000))

    print(robot.transform_mat)
    print([robot.x, robot.y, robot.z])
    print([robot.p1.x, robot.p2.x, robot.p3.x])

    while True:
        target = (int(n) for n in input("enter coordinates (x y z): ").split(" "))
        robot.move(*target)

        print([robot.x, robot.y, robot.z])
        print([robot.p1.x, robot.p2.x, robot.p3.x])
