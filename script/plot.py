##################################
# Reference
##################################
# 1. [Gallery-matplotlib documentation](https://matplotlib.org/stable/gallery/index.html)
# 1. [Matplotlib Cheat Sheet: Plotting in Python](https://www.datacamp.com/community/blog/python-matplotlib-cheat-sheet)
# 1. [Matplotlib Tutorial: Python Plotting](https://www.datacamp.com/community/tutorials/matplotlib-tutorial-python)

##################################
# 安装库
##################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

##################################
# 设置字体
##################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


##################################
# 修改配置文件 | Change default rc settings
##################################
#  print(mpl.matplotlib_fname())
#  mpl.rcParams['lines.linewidth'] = 1.5
# X、Y轴标签字体大小
#  mpl.rcParams['xtick.labelsize'] = 10.5
#  mpl.rcParams['ytick.labelsize'] = 10.5
# X、Y轴刻度标签字体大小
#  mpl.rcParams['axes.labelsize'] = 10.5


##################################
# 创建图框
##################################
# fig:
# 1 inch == 2.54 cm, 单栏8.3 cm(3.27 inch), 双栏17.6 cm(6.9 inch)
# figsize=(x inch, y inch), int dpi,
# facecolor=(r,g,b), edgecolor=(r,g,b),
# bool frameon.
cm = 1/2.54
fig = plt.figure(figsize=(17.6*cm, 20*cm))
data = np.linspace(0,10,100)


##################################
# 绘制子图
##################################
# 注意: 每一个子图是独立的，如是否显示图例，需要单独在子图函数下面增加
### 子图1: 二维曲线图
data_1 = np.linspace(0,10,100)

ax1 = fig.add_subplot(111)
# plot: c(color), marker, linewidth
ax1.plot(data_1, np.sin(data_1), 'r--', label='$sin(x)$')
ax1.plot(data_1, np.cos(data_1), 'g-.', label='$cos(x)$')
ax1.plot(data_1, np.sin(data_1)+np.cos(data_1), 'c', label='$sin(x)+cos(x)$')

# legend: 
# loc = upper/lower/center, right/left/center, best(default)
# bbox_to_anchor = (x, y)
ax1.legend(loc='lower right')
ax1.annotate('$sin(0)=0$', xy=(0,0), xytext=(0.2,-1), arrowprops={'arrowstyle': '->'})
ax1.text(0.2, 1.5, r'Demo Equation: $y=\rm{sin}(x)$')
ax1.grid(True)
# set:
# ax.set_foo(bar) == ax.set(foo=bar)
# title, xlabel, xlim, xticks, xticklabels
ax1.set(title  = '正余弦函数图像',
        xlabel = r'时间$\rm{(t/s)}$',
        ylabel = r'位移$\rm{(x/m)}$')
ax1.set_xlim(0, 10)
ax1.set_ylim(-1.5, 1.9)

##################################
# 后处理
##################################
# make sure that the plots fit nicely in your figure
plt.tight_layout()
#  plt.savefig('../foo.svg')
plt.show()


