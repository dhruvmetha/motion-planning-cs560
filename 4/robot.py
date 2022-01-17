import numpy as np
from collision import isCollisionFree 

class Robot():
    def __init__(self, width, height):
        self.height = height
        self.width = width
        self.translation = (0, 0)
        self.rotation = 0
        self.rotation_matrix = np.array([[1, 0], [0, 1]])

    def set_pose(self, pose):
        x, y, z = pose
        self.translation = (x, y) 
        self.rotation = z
        self.rotation_matrix = np.array([[np.cos(z), -np.sin(z)], [np.sin(z), np.cos(z)]])

    def transform(self):
        pts = np.array([[-self.width/2, -self.height/2], [-self.width/2, self.height/2], [self.width/2, self.height/2], [self.width/2, -self.height/2]])
        return [(i[0] + self.translation[0], i[1] + self.translation[1]) for i in np.matmul(self.rotation_matrix, pts.T).T]

    def kinematics(self, state, control):
        q = state
        u = control
        q_dot = [u[0] * np.cos(q[2] + np.pi/2), u[0] * np.sin(q[2] + np.pi/2), u[1]]
        return np.array(q_dot)

    def dynamics(self, state, control):
        q = state
        u = control
        q_dot = [q[3], q[4], q[5], u[0] * np.cos(q[2] + np.pi/2), u[0] * np.sin(q[2] + np.pi/2), u[1]]
        return np.array(q_dot)

    def propagate(self, state, controls, durations, dt, q_dot=None):
        q = np.array(state)
        n = 0
        u = None
        trajectory = []
        # print('durations', len(durations))
        while len(durations) > 0:
            # if u is None:
            trajectory.append(q.tolist())
            n = durations.pop(0)
            u = controls.pop(0)
            # print('here before', q, n, u)
            # if q_dot is None:
            q_dot = self.dynamics(q, u)
            # print(q_dot)
            q = q + (n * dt * q_dot)
            # print('here after', q, n, u)
            
        # print('returning', q)
        return q, q