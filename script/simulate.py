####################################################################
# 导入库
####################################################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math


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
# Global varibles
c1 = c2 = c3 = c4 = c5 = c6 = c23 = c234 = 0
s1 = s2 = s3 = s4 = s5 = s6 = s23 = s234 = 0
d1, a2, a3, d4, d5, d6 = 89.459, 425, 392.25, 109.15, 94.65, 82.3
R = np.zeros((3,3)); P = np.zeros((3,1)); Tran = np.zeros((4,4))
jcb = ijcb = tjcb = np.zeros((6,6))

cm = 1/2.54
r2d = 180/math.pi
d2r = math.pi/180

### RPY角转旋转矩阵
def rpy2Rot(rpy):
    [gamma, beta, alpha] = rpy
    rot = np.zeros((3,3))

    # X
    rot[0][0] = math.cos(alpha)*math.cos(beta)
    rot[1][0] = math.sin(alpha)*math.cos(beta)
    rot[2][0] = -1*math.sin(beta)
    # Y
    rot[0][1] = math.cos(alpha)*math.sin(beta)*math.sin(gamma) - math.sin(alpha)*math.cos(gamma)
    rot[1][1] = math.sin(alpha)*math.sin(beta)*math.sin(gamma) + math.cos(alpha)*math.cos(gamma)
    rot[2][1] = math.cos(beta)*math.sin(gamma)
    # Z
    rot[0][2] = math.cos(alpha)*math.sin(beta)*math.cos(gamma) + math.sin(alpha)*math.sin(gamma)
    rot[1][2] = math.sin(alpha)*math.sin(beta)*math.cos(gamma) - math.cos(alpha)*math.sin(gamma)
    rot[2][2] = math.cos(beta)*math.cos(gamma)

    return rot

# 更新关节角
def ur_calcJnt(q):
  global c1, c2, c3, c4, c5, c6, c23, c234, s1, s2, s3, s4, s5, s6, s23, s234
  c1 = math.cos(q[0]);  s1 = math.sin(q[0])
  c2 = math.cos(q[1]);  s2 = math.sin(q[1])
  c3 = math.cos(q[2]);  s3 = math.sin(q[2])
  c4 = math.cos(q[3]);  s4 = math.sin(q[3])
  c5 = math.cos(q[4]);  s5 = math.sin(q[4])
  c6 = math.cos(q[5]);  s6 = math.sin(q[5])
  c23 = math.cos(q[1]+q[2]); s23 = math.sin(q[1]+q[2])
  c234 = math.cos(q[1]+q[2]+q[3]); s234 = math.sin(q[1]+q[2]+q[3])
  return [c1, c2, c3, c4, c5, c6, c23, c234, s1, s2, s3, s4, s5, s6, s23, s234]

# 正运动学
def ur_kinematics(q):
  global R, P, Tran

  ur_calcJnt(q)
  hori = d4 + d6*c5
  vert = a2*c2 + a3*c23 - d5*s234 + d6*c234*s5

  P[0][0] = -s1*hori + c1*vert
  P[1][0] = c1*hori + s1*vert
  P[2][0] = d1 - a2*s2 - a3*s23 - d5*c234 - d6*s234*s5

  R[0][0] = c6*(s1*s5 + c1*c234*c5) - s234*c1*s6
  R[0][1] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6
  R[0][2] = c234*c1*s5 - c5*s1
  R[1][0] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6
  R[1][1] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1
  R[1][2] = c1*c5 + c234*s1*s5
  R[2][0] = -c234*s6 - s234*c5*c6
  R[2][1] = s234*c5*s6 - c234*c6
  R[2][2] = -s234*s5

  Tran = np.concatenate((R,P), axis=1)
  Tran = np.concatenate((Tran,np.array([[0,0,0,1]])), axis=0)

  return Tran

# 逆运动学
def inv_kinematics(Tran, q):
    global d1, a2, a3, d4, d5, d6
    q_jnt = np.zeros(6)
    nx = Tran[0][0]; ox = Tran[0][1]; ax = Tran[0][2]; px = Tran[0][3]
    ny = Tran[1][0]; oy = Tran[1][1]; ay = Tran[1][2]; py = Tran[1][3]
    nz = Tran[2][0]; oz = Tran[2][1]; az = Tran[2][2]; pz = Tran[2][3]
    # 关节角1 (t15_24)
    A1 = py - ay*d6; B1 = -px + ax*d6
    temp1 = math.atan2(d4,(A1*A1+B1*B1-d4*d4)**(1/2)) - math.atan2(A1,B1)
    temp2 = math.atan2(d4,-(A1*A1+B1*B1-d4*d4)**(1/2)) - math.atan2(A1,B1)
    q_jnt[0] = temp1 if abs(temp1-q[0])<abs(temp2-q[0]) else temp2
    s1 = math.sin(q_jnt[0]); c1 = math.cos(q_jnt[0])
    # 关节角5 (t15_22)
    temp1 = math.acos(-ax*s1+ay*c1)
    temp2 = -math.acos(-ax*s1+ay*c1)
    q_jnt[4] = temp1 if abs(temp1-q[4])<abs(temp2-q[4]) else temp2
    s5 = math.sin(q_jnt[4])
    # 关节角6 (t15_21)
    A6 = ny*c1-nx*s1; B6 = -oy*c1+ox*s1
    q_jnt[5] = math.atan2(-s5,0) - math.atan2(A6,B6)
    #  q_jnt[5] = math.atan2(-B6/s5,-A6/s5)
    s6 = math.sin(q_jnt[5]); c6 = math.cos(q_jnt[5])
    # 关节角3 (t14_14, t14_34)
    a2 = px*c1 +py*s1 -d5*(nx*c1*s6+ox*c1*c6+ny*s1*s6+oy*s1*c6) -d6*(ay*s1+ax*c1)
    B3 = pz -d1 -az*d6 -d5*(oz*c6+nz*s6)
    temp1 = math.acos((a2*a2+B3*B3-a2*a2-a3*a3)/(2*a2*a3))
    temp2 = -math.acos((a2*a2+B3*B3-a2*a2-a3*a3)/(2*a2*a3))
    q_jnt[2] = temp1 if abs(temp1-q[2])<abs(temp2-q[2]) else temp2
    s3 = math.sin(q_jnt[2]); c3 = math.cos(q_jnt[2])
    # 关节角2 (t14_14, t14_34)
    A2 = -(a3*a3 +2*a2*a3*c3 +a2*a2)
    B2 = -(a3*c3+a2)*a2 +a3*s3*B3
    C2 = (a3*c3+a2)*B3 +a3*s3*a2
    q_jnt[1] = math.atan2(C2/A2, B2/A2)
    # 关节角4 (t15_13, t15_33)
    a3 = -(oz*c6 + nz*s6)
    B4 = -c6*(ox*c1+oy*s1) -s6*(nx*c1+ny*s1)
    q_jnt[3] = math.atan2(B4, a3) - q_jnt[1] - q_jnt[2]
    if (q_jnt[3] > math.pi):
        q_jnt[3] = q_jnt[3] -2*math.pi
    elif (q_jnt[3] < -math.pi):
        q_jnt[3] = q_jnt[3] +2*math.pi
    return q_jnt

def ur_jacobian(q):
  global jcb, tjcb, ijcb

  ur_calcJnt(q)

  jcb[:,[0]] = np.array([-(a2*c2+a3*c23-d5*s234+d6*c234*s5)*s1 - (d4+d6*c5)*c1,
    -(d4+d6*c5)*s1+(a2*c2+a3*c23-d5*s234+d6*c234*s5)*c1, 0, 0, 0, 1]).reshape((6,1))
  jcb[:,[1]] = np.array([-c1*(a2*s2+a3*s23+d5*c234+d6*s234*s5),
    -s1*(a2*s2+a3*s23+d5*c234+d6*s234*s5),
    -a2*c2-a3*c23+d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape(6,1)
  jcb[:,[2]] = np.array([-c1*(a3*s23+d5*c234+d6*s234*s5),
    -s1*(a3*s23+d5*c234+d6*s234*s5),
    -a3*c23+d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape((6,1))
  jcb[:,[3]] = np.array([-c1*(d5*c234+d6*s234*s5),
    -s1*(d5*c234+d6*s234*s5),
    d5*s234-d6*c234*s5,
    -s1, c1, 0]).reshape((6,1))
  jcb[:,[4]] = np.array([d6*(s1*s5 + c1*c234*c5),
    d6*(-c1*s5 + s1*c234*c5),
    -d6*s234*c5, -c1*s234, -s1*s234, -c234]).reshape((6,1))
  jcb[:,[5]] = np.array([0, 0, 0,
    -s1*c5 + c1*c234*s5, c1*c5 + s1*c234*s5, -s234*s5]).reshape((6,1))

  tjcb = np.transpose(jcb)
  ijcb = np.linalg.inv(jcb)


dt = 8     # 采样间隔 (ms)
T  = 500  # 运行时间 (ms)
w  = 40*d2r # 角速度 (rad/s)
v  = 0      # 线速度 (mm/s)
nn = int(T/dt+1) # 采样点数
#  q0 = np.array([0, -90, 90, -90, -90, 0]).reshape((6,1))*d2r  # 初始关节角
#  q0 = np.array([0, 0, 0, 0, -90, 90]).reshape((6,1))*d2r  # 初始关节角
q0 = np.array([0, -98.9, 117.80, -108.9, -90, 90])*d2r
pos = np.zeros((3,nn))
q_jnt = np.zeros((6,nn))
dx = np.array([v/(T/dt), 0, 0, w/(T/dt), 0, 0]).reshape((6,1))  # 每个周期的状态变化
time = np.linspace(0, int(T), int(T/dt+1)) # 时间

ur_kinematics(q0)
#  pos[:,0] = Tran[0:3,3]
#  ur_jacobian(q0)
#  print(ijcb)

# 循环开始
q = q0
for tt in time:
  i = int(tt/dt)
  ur_kinematics(q)
  pos[:,i] = Tran[0:3,3]
  ur_jacobian(q)
  dq = ijcb@dx
  q = q + dq
  q_jnt[:,[i]] = q

# 初始状态
#  q_jnt[:,0] = q0.reshape(1,6)
#  q0 = [0, -98.9, 117.79, -108.88]
#  ur_kinematics(q)
#  pos[:,0] = Tran[0:3,3]
#  print(Tran)
# 终止状态
#  Tran = [[1,0,0,300],[0,1,0,300],[0,0,1,300],[0,0,0,1]]
#  qq = inv_kinematics(Tran, q0)
#  print("\n")
#  print(q0.reshape(1,6)*r2d)
#  print(qq*r2d)
#  ur_kinematics(qq)
#  print(Tran)
#  pos[:,-1] = Tran[0:3,3]
#  q_jnt[:,-1] = qq

####################################################################
# 创建图框
####################################################################
# fig:
# 1 inch == 2.54 cm, 单栏8.3 cm(3.27 inch), 双栏17.6 cm(6.9 inch)
# figsize=(x inch, y inch), int dpi,
# facecolor=(r,g,b), edgecolor=(r,g,b),
# bool frameon.
fig = plt.figure(figsize=(17.6*cm, 20*cm))


####################################################################
# 绘制子图1: 关节角度变化曲线
####################################################################
ax1 = fig.add_subplot(221)
# plot: c(color), marker, linewidth
ax1.plot(time, q_jnt[0,:]*r2d, 'r--', label=r'$\theta_1$')
ax1.plot(time, q_jnt[1,:]*r2d, 'g:',  label=r'$\theta_2$')
ax1.plot(time, q_jnt[2,:]*r2d, 'c',   label=r'$\theta_3$')
ax1.plot(time, q_jnt[3,:]*r2d, 'y--', label=r'$\theta_4$')
ax1.plot(time, q_jnt[4,:]*r2d, 'm:',  label=r'$\theta_5$')
ax1.plot(time, q_jnt[5,:]*r2d, 'b',   label=r'$\theta_6$')

# legend:
# loc = upper/lower/center, right/left/center, best(default)
ax1.legend(loc='best')
ax1.grid(True)
# set:
# ax.set_foo(bar) == ax.set(foo=bar)
# title, xlabel, xlim, xticks, xticklabels
ax1.set(title  = '关节角变化曲线',
        xlabel = r'时间$(t\rm{/ms})$',
        ylabel = r'角度$\rm{(deg/^o)}$')


####################################################################
# 绘制子图2: 末端位姿变化曲线
####################################################################
ax2 = fig.add_subplot(222,projection='3d')

# 数据准备
Tran_beg = ur_kinematics(q_jnt[:,[0]])
Tran_end = ur_kinematics(q_jnt[:,[-1]])
# 末端位置
hnd_x = pos[0,:]
hnd_y = pos[1,:]
hnd_z = pos[2,:]
# 方向向量长度
vec_len = 200

# 起点、终点、原点坐标
ax2.scatter(hnd_x[0], hnd_y[0], hnd_z[0], marker='*', color='darkorange',
        linewidth=2.5, label='Start point')
ax2.scatter(hnd_x[-1], hnd_y[-1], hnd_z[-1], marker='*', color='m',
        linewidth=2.5, label='End point')
ax2.scatter(0,0,0, marker='d', color='k', linewidth=5, label='Base')
# 原点坐标系
ax2.quiver(0,0,0,1,0,0,length=vec_len,normalize=False)
ax2.quiver(0,0,0,0,1,0,length=vec_len, color='g' , normalize=False)
ax2.quiver(0,0,0,0,0,1,length=vec_len, color='r' , normalize=False)
# 起点姿态
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],Tran_beg[0][0], Tran_beg[1][0], Tran_beg[2][0],
        length=vec_len,normalize=False,alpha=0.5)
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],Tran_beg[0][1], Tran_beg[1][1], Tran_beg[2][1],
        length=vec_len, color='g' , normalize=False,alpha=0.5)
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],Tran_beg[0][2], Tran_beg[1][2], Tran_beg[2][2],
        length=vec_len, color='r' , normalize=False,alpha=0.5)
# 终点姿态
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],Tran_end[0][0], Tran_end[1][0], Tran_end[2][0],
        length=vec_len, normalize=False)
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],Tran_end[0][1], Tran_end[1][1], Tran_end[2][1],
        length=vec_len, color='g' , normalize=False)
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],Tran_end[0][2], Tran_end[1][2], Tran_end[2][2],
        length=vec_len, color='r' , normalize=False)


# 图片设置
ax2.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
        title = '末端位姿轨迹图')
ax2.legend(loc='best')
ax2.set_xlim(0, 800)
ax2.set_ylim(-400,400)
ax2.set_zlim(0, 800)


####################################################################
# 绘制子图2: 末端位姿变化曲线
####################################################################
ax3 = fig.add_subplot(223,projection='3d')

ax3.scatter(hnd_x[0], hnd_y[0], hnd_z[0], marker='*', color='darkorange',
        linewidth=2.5, label='Start point')
ax3.scatter(hnd_x[-1], hnd_y[-1], hnd_z[-1], marker='*', color='m',
        linewidth=2.5, label='End point')
ax3.ticklabel_format(useOffset=False, style='plain')
xmin, xmax = ax3.get_xlim()
ymin, ymax = ax3.get_ylim()
zmin, zmax = ax3.get_zlim()
ax3.plot(hnd_x, hnd_y, hnd_z, label='Displacement in 3D space')
ax3.plot(hnd_y, hnd_z, zs=xmin, zdir='x', linestyle=':', label='Displacement in (y, z)')
ax3.plot(hnd_x, hnd_z, zs=ymax, zdir='y', linestyle=':', label='Displacement in (x, z)')
ax3.plot(hnd_x, hnd_y, zs=zmin, zdir='z', linestyle=':', label='Displacement in (x, y)')

ax3.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
        title = '末端位置轨迹图')
ax3.legend(loc='best')

####################################################################
# 后处理
####################################################################
# make sure that the plots fit nicely in your figure
plt.tight_layout()
#  plt.savefig('../foo.svg')
#  plt.show()


