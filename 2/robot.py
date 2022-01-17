import numpy as np

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

        
