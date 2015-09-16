# WovenCloth
基于[SIGGRAPH2014 Yarn-Level Woven Cloth](http://dl.acm.org/citation.cfm?doid=2661229.2661279)完成了基本的纺线针织布料模拟。视频结果由Mitsuba渲染器光线跟踪完成。

#截图
[!screenshot](https://raw.githubusercontent.com/BentleyBlanks/WovenCloth/master/screenshot/1.png)
 
#构建说明
1.算法部分由OpenFramework+Eigen完成

2.离线渲染部分由SIGGRAPH常见的Mitsuba完成渲染

#使用说明
1.使用Eigen中精确度最高的jacobiSvd方法完成8*8的线性微分方程组求解

2.本布料模拟支持弯曲，剪切，拉伸等基本纺线属性

3.世纪程序中只使用了半径为30的纺线，总长宽高为20*20。过多纺线目前会因误差原因导致程序直接出现不稳定。

4.OpenFrameworks程序支持离线渲染和实时渲染，离线渲染支持将结果全部导出成obj，实时渲染效率较低。只需要更改顶部的```宏的搭配```即可。
```cpp
#define TEST 0
#ifndef TEST
#define ONCE 1
#endif // !TEST

//#define STRETCH 2
//#define BEND 3
//#define SHEAR 4
//#define FRICTION 5
#define OFFLINE_RENDERING 6
```

5.程序较为不稳定，只实现了其原论文50%的效果。已向作者寻求问题求解，希望我能够理解并及时更新。


##关于作者
```cpp
int 官某某 = "Bingo";

char[] 个人博客 = "http://bentleyblanks.github.io";
```

