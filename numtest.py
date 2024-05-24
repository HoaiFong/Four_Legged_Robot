import numpy as np
a = np.array([  [1, 2, 3],
                [4, 5, 6],
                [7, 8, 9]])
b = np.array([  [2],
                [4],
                [6]])
c = np.array([11,22,33])
print(a.dot(b)) # Output: [1, 2, 3]print(type(a)) # Output: <class 'numpy.ndarray'>
print(c)
print(b[0,0])
print(np.transpose(c))