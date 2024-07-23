from main import Tripteron, Slider
import numpy as np

if __name__ == "__main__":
    robot = Tripteron(Slider((1, 1, 1), (376.5, 44.593, 0), limits=(155, 376.5)),
                      # starting position coordinates from CAD
                      Slider((1, -1, 1), (376.5, -44.593, 0), limits=(155, 376.5)),
                      Slider((-1, 0, 1), (66.376, 0, -2.5), limits=(10, 325)),
                      head_z_offset=-56.560,  # calculated by setting to 0 to get intersect, then subtracting cad z
                      slider_dist_limits=(140, 305),
                      simulation=True)

    for z in range(0, robot.z): # Tripteron starting pos is at max height
        arr = np.zeros((robot.p1.limits[1]))

# Given a Tripteron object, brute-force to estimate the work envelope