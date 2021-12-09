####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math
import sympy as sym
from sympy import sin,cos


####################################################################
# 设置字体
####################################################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


####################################################################
# 自定义函数
####################################################################
I = sym.Matrix([[1,0,0], [0,1,0], [0,0,1]])
deg2rad = math.pi/180
rad2deg = 180/math.pi
# 机械臂构型描述
w2 = sym.Matrix([0,-1,0]);  v2 = sym.Matrix([734.95, 0, 94.65])
w3 = sym.Matrix([0,-1,0]);  v3 = sym.Matrix([309.95, 0, 94.65]  )
w4 = sym.Matrix([0,-1,0]);  v4 = sym.Matrix([-82.3, 0, 94.65] )
M = sym.Matrix([[0,0,-1,734.95], [0,-1,0,109.15], [-1,0,0,-5.191], [0,0,0,1]])
### 旋量转换为齐次变换矩阵
def twist2tran(w, vel, q):
    #  global I
    lw = li(w)
    v = sym.Matrix([[vel[0]],[vel[1]],[vel[2]]])
    rot = I + sin(q)*lw + (1-cos(q))*lw*lw
    pos = (I*q + (1-cos(q))*lw + (q-sin(q))*lw*lw) * v
    tran = sym.Matrix([[rot, pos],[0,0,0,1]])
    tran = sym.simplify(tran)
    return tran

# 角速度转换为李代数
def li(w):
    lw = sym.Matrix([[0,-w[2],w[1]], [w[2],0,-w[0]], [-w[1],w[0],0]])
    return lw

# 齐次转换矩阵对应的邻接转换矩阵
def adj(tran):
    R = tran[0:3,0:3]
    p = tran[0:3,3]
    pR = li(p)*R
    O33 = sym.Matrix([[0,0,0], [0,0,0], [0,0,0]])
    adjMat = sym.Matrix([[R,O33], [pR,R]])
    adjMat = sym.simplify(adjMat)
    return adjMat

# 正运动学
def kinematic(q):
    tran2 = twist2tran(w2, v2, q[0])
    tran3 = twist2tran(w3, v3, q[1])
    tran4 = twist2tran(w4, v4, q[2])
    T = M*tran2*tran3*tran4
    return T

# 雅克比
def jacobian(twist, q):
    c4 = cos(q[2]); s4 = sin(q[2])
    c34 = cos(q[1]+q[2]); s34 = sin(q[1]+q[2])
    jb4 = sym.Matrix([-1, -82.3, 94.65])
    jb3 = sym.Matrix([-1, jb4[1]+392.25*c4, jb4[2]-392.25*s4])
    #  jb3 = sym.Matrix([0, -1, 0, 392.25*c4-82.3, 0, 94.65-392.25*s4])
    jb2 = sym.Matrix([-1, jb3[1]+425*c34, jb3[2]-425*s34])
    #  jb2 = sym.Matrix([0, -1, 0, 392.25*c4-82.3+425*c34, 0, 94.65-392.25*s4-425*s34])
    jcb = sym.Matrix([[jb2, jb3, jb4]])**(-1)
    sym.print_latex(jcb*twist)
    return q

if __name__ == '__main__':
    w = sym.symbols('w_1:4')
    v = sym.symbols('v_1:4')
    q = sym.symbols('q_2:7')
    #  q = np.array([0, -98.9, 117.80, -108.9, -90, 90])*deg2rad
    q = np.array([-98.9, 117.80, -108.9])*deg2rad
    #  T = kinematic(q)
    #  sym.print_latex(sym.simplify(T))
    twist = sym.Matrix([-1,0,0])
    jacobian(twist, q)


