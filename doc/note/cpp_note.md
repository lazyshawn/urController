## ✨ 目录
1. [C++中的重要知识点](#c%2B%2B中的重要知识点)
    * [在构造函数中调用构造函数](#在构造函数中调用构造函数)

## ✨ C++中的重要知识点
### 在构造函数中调用构造函数
假设我们有一个写好的构造函数 `A(int,int)` ,我们现在想要重载一个构造函数
`A(int,int,int)`,但是新的构造函数除了部分操作外，其他基本和原来写好的构造函数
一致，这就出现了极大的代码重复。
代码重复是令我们恶心的一点，但是我们又不能直接在新写的构造函数中调原构造函数，
这样只是在构造函数中构建了一个临时变量，并没有用构造函数来初始化自己。
正确的方法是使用 `placement new`:
```cpp
#include <iostream>
class A{
  private:
    int a;
  public:
    A(int);
    A(int,int);
};

// 构造1
A::A(int _a):a(_a){
}

// 构造2
A::A(int _a,int){
  new(this) A(_a);
  cout << a << endl;
}

int main(){
  A a(12,12);
}
```
**Remark**: 但是在初始化 Realsense 相机时，使用 `placement new` 会报错 
`what():  xioctl(VIDIOC_S_FMT) failed Last Error: Device or resource busy`。
(1) 可能是重复激活同一个相机导致的。

1. [C++ 如何在构造函数中调用构造函数, bradypbai, CSDN](
https://blog.csdn.net/weixin_43406295/article/details/98101439)。
1. [C++ 中如何在一个构造函数中调用另一个构造函数, qingfenghao, ChinaUnix](
http://blog.chinaunix.net/uid-23741326-id-3385581.html)

