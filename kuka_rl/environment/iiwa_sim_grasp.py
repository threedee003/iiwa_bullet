import time
import numpy as np
import math

iiwa_eef_idx = 6
iiwa_num_dof = 7

# lower limit
# upper limit
ll = [-3.14]*iiwa_num_dof
ul = [3.14]*iiwa_num_dof


class iiwaSim(object):
    def __init__(self,
                bullet_client,
                offset) -> None:
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualthreshold = 0)
        self.offset = np.array(offset)

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHIC_SHAPES
        self.legos = []

        