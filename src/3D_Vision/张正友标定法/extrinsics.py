#!usr/bin/env/ python
# _*_ coding:utf-8 _*_
 
import numpy as np
 
#返回每一幅图的外参矩阵[R|t]
def get_extrinsics_param(H, intrinsics_param):
    extrinsics_param = []
 
    inv_intrinsics_param = np.linalg.inv(intrinsics_param)
    for i in range(len(H)):
        h0 = (H[i].reshape(3, 3))[:, 0]
        h1 = (H[i].reshape(3, 3))[:, 1]
        h2 = (H[i].reshape(3, 3))[:, 2]
 
        scale_factor = 1 / np.linalg.norm(np.dot(inv_intrinsics_param, h0))
 
        r0 = scale_factor * np.dot(inv_intrinsics_param, h0)
        r1 = scale_factor * np.dot(inv_intrinsics_param, h1)
        t = scale_factor * np.dot(inv_intrinsics_param, h2)
        r2 = np.cross(r0, r1)
 
        R = np.array([r0, r1, r2, t]).transpose()
        extrinsics_param.append(R)
 
    return extrinsics_param
