import numpy as np

def sample():
    return (round(np.random.uniform(0, 10, 1)[0], 1), round(np.random.uniform(0, 10, 1)[0], 1), round(np.random.uniform(-np.pi, np.pi, 1)[0], 2)) 

def sample_between(n1, n2, size):
    return np.random.uniform(n1, n2, size).round(2)

def sample_integer(n1, n2):
    return np.random.randint(n1, n2)