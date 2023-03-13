# SNL
想要写一个用于导航领域的C++标准库，~~不清楚要不要使用模板~~

## ***.gitignore***

```.ignore
# 忽略所有的 .a 文件
*.a
# 但跟踪所有的 lib.a，即便你在前面忽略了 .a 文件
!lib.a
# 只忽略当前目录下的 TODO 文件，而不忽略 subdir/TODO
/TODO
# 忽略任何目录下名为 build 的文件夹
build/
# 忽略 doc/notes.txt，但不忽略 doc/server/arch.txt
doc/*.txt
# 忽略 doc/ 目录及其所有子目录下的 .pdf 文件
doc/**/*.pdf
```

## *git*

```
#git提交代码

git init #可能不需要
git add 文件/文件夹名
git commit -m '提交描述'
git push 

#删除某一文件 参考：https://www.jianshu.com/p/b4f93946a27d
git rm 要删除的文件名
git rm -r 要删除的文件夹
```



## _git_error_

```
Updates were rejected because the remote contains work that you do  not have locally. This is usually caused by another repository pushing to the same ref. You may want to first integrate the remote changes  (e.g., 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
原因在于：远程仓库存在本地仓库不存在的提交
git pull
```



## _markdown_

```
https://www.runoob.com/markdown/md-block.html
```



## 2023年03月04日



```none
#### 17:40:51

​	思考良久，决定还是记录时间来描述当前的想法。目前困惑于该如何用一个无序容器来容纳不同的ROS消息体类型。
正在考虑使用tuple来解决这个问题，或者用可变参数模板。C++这些部分都没有深入学习，难以下手，用时方恨少。

#### 19:51:31

​	突然意识我可以写一个消息类来封装ros的这些消息体。

#### 23:46:44

​	经过差不多一个晚上的调试成功将代码调试完成，目前可以提供的接口API（除去基本的添加元素，删除元素，etc）可以做到如下的一个功能：

​	对于两个任意带有header头并有时间戳的话题消息类型，可以将单个为基准的消息数据变量与另外一组消息类型的数据容器的元素进行时
间戳比对，输出容器中满足在一定时间范围阈值（可自行设置）的消息数据。这个功能主要用于多传感器时间同步。代码解决单容器不能存放不
同消息体的手段是采用std::any，采用模板编程。模板编程yyds!
```



## 2023年3月5日

```

这种不能创建的类型，是不是可以用于模板参数，并在模板函数里调用它们的静态方法;
类的继承?(o゜▽゜)o☆

```



## 2023年3月7日

```none
😍图像滤波😍

滤波器指的是一种由一副图像$I(x, y)$根据像素点$x, y$附件的区域计算得到一副新图像$I^{'}(x, y)$ 的算法。

均值滤波

方框滤波 $\rightarrow$归一化$\rightarrow$均值滤波

高斯滤波

双边滤波
```



## 2023年3月8日

```none
但是对于双目摄像头，当场景中的物体与摄像头的距离远大于两个摄像头基线的距离时，无人机的双目视觉就退化成了单目视觉问题🐴。

《学习OPenCV3》

特征点：关键子（特征点的像素坐标），描述子（是一个向量，描述了该关键点周围像素的信息）

FAST关键子：根据光度亮暗即灰度变化明显的地方。改进型的角点检测：ORB特征点：Oriented FAST关键字 + BRIF描述子

过程：
1、选取一像素点p，假设他的亮度为Ip。
2、设置阈值T（比如Ip的20%）
3、以像素p为中心，选取半径为3的圆上的16个像素点
4、假如选取的圆上有连续的N个点的亮度大于Tp + T 或小于Ip - T，那么像素点p可以被认为是特征点。N取12则为FAST-12，以此类推。
   循环以上4步，对每一个像素执行相同的操作。

特征点扎堆现象：需要非极大值抑制（YOLOV5类似）；存在尺度问题——远处看起来像是角点的东西，接近后可能不是角点了：图像金字塔
,金字塔的每一层都是一张缩放或者放大的图片。


欧式距离：两点之间的直线距离。汉明距离：两个二进制串之间的汉明距离，指的是其不同位数的个数。

特征点匹配：运算两幅图中的特征点之间的描述子的距离（欧式距离、汉明距离），描述子距离代表了两个特征点之间的相似程度。使用快速近
邻（FLANN）算法
```

<img decoding="async" src="./images/pyramid.png" width="30%" title = "图像金字塔" align = "center">

<center style="font-size:14px;color:#C0C0C0;text-decoration:underline">图1.图像金字塔</center> 

## 2023年3月12日

```c++
int a = NULL;
int a1 = 0;
new(&a) int(0); //a的内存空间在栈区
int* p = &a; 
int* p1 = &a1;
int* p2 = new int();
int* p3 = new int();
```

## 2023年3月13日

```
😍点扩散函数（英語：point spread function，简称PSF）😍
是描述光学系统对点源解析能力的函数。因为点源在经过任何光学系统后都会由于衍射而形成一个扩大的像点，通过测量系统的点扩展函数，能
够更准确地提取图像信息。
```

