#!/usr/bin/env python

import numpy as np

start = np.array([[1, 2, 3, 4]])
end = np.array([[10, 20, 30, 40]])
L = np.concatenate([start.T, end.T], 1)
step = 5
coef = np.linspace(1, 0, step).reshape(1, step)
R =  np.concatenate([coef, coef[:, ::-1]], 0)
print(L)
print(R)
A = np.dot(L, R).T
print(A)

for i in range(step):
  print(A[0, :].tolist())
  if (A.shape[0] - 1):
    A = np.delete(A, 0, 0)
  # print(A[i])
print(A.shape)
print(A[0, :])


