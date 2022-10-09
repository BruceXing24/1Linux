# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/3 18:39
import numpy as np

body = 1.31844704893266
LF1 = 0.128744524052319
LF2 = 0.323696829552903
LF3 = 0.13341

mass = body+   4*(LF1+LF2+LF3)
print(mass)
print(mass*9.8)

R = np.array([
                [1,0,0,],
                [0,1,0,],
                [0,0,1,],
              ])
D = np.array([
    [0,0,0],
    [0,0,-0.2236],
    [0,0.2236,0]
])
matrix = R + np.dot(D,R)

matrix_1 = np.linalg.inv(matrix)
f3=np.array([[1.8269],
         [0.0248],
         [-9.6398]])
# (-0.016918666204606934, -2.4765200999228836, -3.60073194448647)

f4 = np.dot(matrix_1,f3)
print(f4)
print(mass*9.8/4)