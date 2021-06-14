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
# 读入数据
####################################################################
data_theta  = np.loadtxt("../data/data.curtheta", dtype=float)
data_curPos = np.loadtxt("../data/data.curpos", dtype=float)
time = data_theta[:,0]


####################################################################
# 修改配置文件 | Change default rc settings
####################################################################
#  print(mpl.matplotlib_fname())
#  mpl.rcParams['lines.linewidth'] = 1.5
# X、Y轴标签字体大小
#  mpl.rcParams['xtick.labelsize'] = 10.5
#  mpl.rcParams['ytick.labelsize'] = 10.5
# X、Y轴刻度标签字体大小
#  mpl.rcParams['axes.labelsize'] = 10.5

####################################################################
# 自定义函数
####################################################################
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


####################################################################
# 创建图框
####################################################################
# fig:
# 1 inch == 2.54 cm, 单栏8.3 cm(3.27 inch), 双栏17.6 cm(6.9 inch)
# figsize=(x inch, y inch), int dpi,
# facecolor=(r,g,b), edgecolor=(r,g,b),
# bool frameon.
cm = 1/2.54
r2d = 180/math.pi
d2r = math.pi/180
fig = plt.figure(figsize=(17.6*cm, 20*cm))
data = np.linspace(0,10,100)


####################################################################
# 绘制子图
####################################################################
### 子图1: 二维曲线图
theta_1 = data_theta[:,1]
theta_2 = data_theta[:,2]
theta_3 = data_theta[:,3]
theta_4 = data_theta[:,4]
theta_5 = data_theta[:,5]
theta_6 = data_theta[:,6]

ax1 = fig.add_subplot(221)
# plot: c(color), marker, linewidth
ax1.plot(time, theta_1, 'r--', label=r'$\theta_1$')
ax1.plot(time, theta_2, 'g:', label=r'$\theta_2$')
ax1.plot(time, theta_3, 'c',   label=r'$\theta_3$')
ax1.plot(time, theta_4, 'y--',   label=r'$\theta_4$')
ax1.plot(time, theta_5, 'm:',   label=r'$\theta_5$')
ax1.plot(time, theta_6, 'b',   label=r'$\theta_6$')

# legend: 
# loc = upper/lower/center, right/left/center, best(default)
ax1.legend(loc='lower right')
ax1.grid(True)
# set:
# ax.set_foo(bar) == ax.set(foo=bar)
# title, xlabel, xlim, xticks, xticklabels
ax1.set(title  = '关节角变化曲线',
        xlabel = r'时间$\rm{(t/s)}$',
        ylabel = r'角度$\rm{(deg/^o)}$')

####################################################################
### 子图2: 末端位置轨迹图
hnd_x = data_curPos[:,1]
hnd_y = data_curPos[:,2]
hnd_z = data_curPos[:,3]
# r-gama; p-beta; y-alpha;
ori_r = data_curPos[:,6]*d2r
ori_p = data_curPos[:,5]*d2r
ori_y = data_curPos[:,4]*d2r
# 方向向量长度
vec_len = 200

# 起点坐标系 [XYZ]
rot_beg = rpy2Rot([ori_r[0], ori_p[0], ori_y[0]])
# 起点坐标系 [XYZ]
rot_end = rpy2Rot([ori_r[-1], ori_p[-1], ori_y[-1]])

ax2 = fig.add_subplot(222,projection='3d')
# 末端点轨迹
ax2.plot(hnd_x, hnd_y, hnd_z, 'r:')
# 特殊点
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
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][0], rot_beg[1][0], rot_beg[2][0],
        length=vec_len,normalize=False,alpha=0.5)
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][1], rot_beg[1][1], rot_beg[2][1],
        length=vec_len, color='g' , normalize=False,alpha=0.5)
ax2.quiver(hnd_x[0], hnd_y[0], hnd_z[0],rot_beg[0][2], rot_beg[1][2], rot_beg[2][2],
        length=vec_len, color='r' , normalize=False,alpha=0.5)
# 终点姿态
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][0], rot_end[1][0], rot_end[2][0],
        length=vec_len,normalize=False)
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][1], rot_end[1][1], rot_end[2][1],
        length=vec_len, color='g' , normalize=False)
ax2.quiver(hnd_x[-1], hnd_y[-1], hnd_z[-1],rot_end[0][2], rot_end[1][2], rot_end[2][2],
        length=vec_len, color='r' , normalize=False)

ax2.set(xlabel = r'X/mm', ylabel = r'Y/mm', zlabel = r'Z/mm',
        title = '末端位置轨迹图')
ax2.legend(loc='best')
ax2.set_xlim(0, 800)
ax2.set_ylim(-400,400)
ax2.set_zlim(0, 800)

####################################################################
### 子图3: 末端坐标系姿态
ax3 = fig.add_subplot(223,projection='3d')
ax3.quiver(0,0,0,1,0,0,length=1,normalize=False)
ax3.quiver(0,0,0,0,1,0,length=1, color='g' , normalize=False)
ax3.quiver(0,0,0,0,0,1,length=1, color='r' , normalize=False)

ax3.set_xlim(-1,1)
ax3.set_ylim(-1,1)
ax3.set_zlim(-1,1)


####################################################################
# 后处理
####################################################################
# make sure that the plots fit nicely in your figure
plt.tight_layout()
#  plt.savefig('../foo.svg')
plt.show()

