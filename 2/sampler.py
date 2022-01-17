import numpy as np

def sample():
    return (round(np.random.uniform(0, 10, 1)[0], 1), round(np.random.uniform(0, 10, 1)[0], 1), round(np.random.uniform(-np.pi, np.pi, 1)[0], 2)) 
