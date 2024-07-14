from math_utils import Plane
import numpy as np
import serial
import time


class Slider(Plane):
    def __init__(self, *args, limits=(0, 0)):
        super().__init__(*args)
        self.limits = limits


class Tripteron:
    def __init__(self, p1: Slider, p2: Slider, p3: Slider, head_z_offset, slider_dist_limits, simulation=False):
        """
        is tha robot!!

        :param p1: Slider 1 (typically normal vector (1, 1, 1))
        :param p2: Slider 2 (typically normal vector (1, -1, 1))
        :param p3: Slider 3 (typically normal vector (-1, 0, 1))
        :param head_z_offset: z offset from the intersection of the 3 planes to the head
        :param slider_dist_limits: min/max distance to maintain between p1/p1 and p3 (so the arms don't get tangled up)
        """
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.x, self.y, self.z = Plane.intersect(p1, p2, p3).flatten()
        self.z += head_z_offset
        self.slider_dist_limits = slider_dist_limits
        self.transform_mat = self.generate_transform_matrix()
        self.simulation = simulation
        if not simulation:
            self.serial = serial.Serial('/dev/tty.usbmodem1811', 115200) # TODO: Change to match your laptop/pc
            self.serial.write("\r\n\r\n")  # grbl wake-up
            time.sleep(2)  # Wait for grbl to initialize
            self.serial.flushInput()  # Flush startup text in serial input

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
        inv = np.linalg.inv(mat)
        return inv

    def check_move(self, p1, p2, p3):
        """
        Runs checks to ensure that a planned move won't damage anything.

        :param p1: planned position of p1
        :param p2: planned position of p1
        :param p3: planned position of p1
        :return: True/False of move legality
        """

        print((
                # individual max/min
                self.p1.limits[0] <= p1 <= self.p1.limits[1],
                self.p2.limits[0] <= p2 <= self.p2.limits[1],
                self.p3.limits[0] <= p3 <= self.p3.limits[1],
                # ensure that p1/p2 isn't too close/too far from p3 (risk of collisions)
                self.slider_dist_limits[0] <= abs(p1-p3) <= self.slider_dist_limits[1],
                self.slider_dist_limits[0] <= abs(p2-p3) <= self.slider_dist_limits[1]
                ))

        return (
                # individual max/min
                self.p1.limits[0] <= p1 <= self.p1.limits[1] and
                self.p2.limits[0] <= p2 <= self.p2.limits[1] and
                self.p3.limits[0] <= p3 <= self.p3.limits[1] and
                # ensure that p1/p2 isn't too close/too far from p3 (risk of collisions)
                self.slider_dist_limits[0] <= abs(p1-p3) <= self.slider_dist_limits[1] and
                self.slider_dist_limits[0] <= abs(p2-p3) <= self.slider_dist_limits[1]
                )

    def move(self, x, y, z):
        """
        Moves the robot. yay.

        :param x: desired head x position
        :param y: desired head y position
        :param z: desired head z position
        :return: True/False of whether the move succeeded
        """
        p1_delta, p2_delta, p3_delta = np.matmul(self.transform_mat, np.array((self.x-x, self.y-y, self.z-z)))

        if not self.check_move(self.p1.x + p1_delta, self.p2.x + p2_delta, self.p3.x + p3_delta):
            return False

        # see https://github.com/michaelfranzl/grbl-streamer if this gets too complicated
        if not self.simulation:
            # REMEMBER: wire the robot so that x, y, z correspond to the correct sliders
            self.serial.write(f"G0 X{self.p1.x + p1_delta} Y{self.p2.x + p2_delta} Z{self.p3.x + p3_delta}\n")
            grbl_out = self.serial.readline()  # Wait for grbl response with carriage return
            print(grbl_out.strip())

        self.p1.x += p1_delta
        self.p2.x += p2_delta
        self.p3.x += p3_delta
        self.x = x
        self.y = y
        self.z = z

        return True


"""
Limits (from CAD): 
    - p1/p2 max = 376.5
    - p1/p2 min = 155

    - p3 max = 235
    - p3 min = 10

    - p1/p2 to p3 min = 140
    - p1/p2 to p3 max = 305

"""

if __name__ == "__main__":
    robot = Tripteron(Slider((1, 1, 1), (376.5, 44.593, 0), limits=(155, 376.5)),  # starting position coordinates from CAD
                      Slider((1, -1, 1), (376.5, -44.593, 0), limits=(155, 376.5)),
                      Slider((-1, 0, 1), (66.376, 0, -2.5), limits=(10, 325)),
                      head_z_offset=-56.560,  # calculated by setting to 0 to get intersect, then subtracting cad z
                      slider_dist_limits=(140, 305),
                      simulation=True)

    # print(robot.transform_mat)
    print(f"Head pos: {[robot.x, robot.y, robot.z]}")
    print(f"Motors pos: {[robot.p1.x, robot.p2.x, robot.p3.x]}")

    while True:
        target = (float(n) for n in input("enter coordinates (x y z): ").split(" "))
        ret = robot.move(*target)

        if ret:
            print(f"Head pos: {[robot.x, robot.y, robot.z]}")
            print(f"Motors pos: {[robot.p1.x, robot.p2.x, robot.p3.x]}")
        else:
            print("ILLEGAL MOVE. WOMP WOMP TRY AGAIN")
        print("---------------------------------------------------------------------")
