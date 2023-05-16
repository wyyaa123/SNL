# SNL
想要写一个用于导航领域的C++标准库，~~不清楚要不要使用模板~~

## ***.gitignore***

```
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

```powershell
#git提交代码

git init #可能不需要
git add #文件/文件夹名
git commit -m '提交描述'
git push 

#删除某一文件 参考：https://www.jianshu.com/p/b4f93946a27d
git rm '+ 要删除的文件名'
git rm -r '+ 要删除的文件夹'
```



## _git_error_

```powershell
Updates were rejected because the remote contains work that you do  not have locally. This is usually caused by another repository pushing to the same ref. You may want to first integrate the remote changes  (e.g., 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
#原因在于：远程仓库存在本地仓库不存在的提交
git pull
```



## _markdown_

```c++
//https://www.runoob.com/markdown/md-block.html
```



## 2023年03月04日



```c++
/*
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
*/
```



## 2023年3月5日

```c++
/*
这种不能创建的类型，是不是可以用于模板参数，并在模板函数里调用它们的静态方法;
类的继承?(o゜▽゜)o☆
*/
```



## 2023年3月7日

```c++
/*
😍图像滤波😍

滤波器指的是一种由一副图像$I(x, y)$根据像素点$x, y$附件的区域计算得到一副新图像$I^{'}(x, y)$ 的算法。

均值滤波

方框滤波 $\rightarrow$归一化$\rightarrow$均值滤波

高斯滤波

双边滤波
*/
```



## 2023年3月8日

```c++
/*
但是对于双目摄像头，当场景中的物体与摄像头的距离远大于两个摄像头基线的距离时，无人机的双目视觉就退化成了单目视觉问题🐴。

《学习OpenCV3》

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

特征点匹配：运算两幅图中的特征点之间的描述子的距离（欧式距离、汉明距离），描述子距离代表了两个特征点之间的相似程度。
使用快速近邻（FLANN）算法
*/
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

:star:相机成像原理：https://www.bilibili.com/video/BV1yE411o7kJ/?spm_id_from=333.1007.top_right_bar_window_history.content.click&vd_source=3487a7a535cad2041999f1ffa1007a8f

```c++
/*
💥曝光时间：曝光时间是指在摄影过程中，感光介质（如胶片或相机传感器）接收到光线的时间长度。简单来说，曝光时间就是快门打开的时
间，用于控制感光介质受到光线的多少。

如果物体的运动速度飞快，曝光时间就无法捕捉到物体的轮廓导致图像运动失真（欠采样失真?）

💥帧率：帧率是指每秒钟播放的帧数，常用单位是fps（Frames Per Second）


💥相机ISO是衡量相机感光度的单位，表示相机可以接收到多少光线。
ISO的值越高，相机的感光度就越高，可以在较暗的环境下拍摄照片，但同时也会增加照片噪点的可能性。

💥当图像中某个像素点的光线强度超过相机感受范围时，就会发生像素点饱和，
这意味着该像素点记录的颜色值达到了相机能够记录的最大值或最小值，无法再继续增加或减少。

当一个像素点饱和时，它的颜色值就会丢失一部分信息，这可能会导致图像失真或过曝。
因此，当进行摄影时，需要避免过度曝光，以防止像素点饱和。

通常，相机的ISO范围是从ISO 100到ISO 6400，但现代相机的ISO范围可能更广。
在选择ISO时，需要根据实际拍摄环境进行调整，以获得合适的曝光和图像质量。
一般来说，在光线充足的情况下，选择较低的ISO值可以获得更清晰、更准确的图像；
而在光线较暗的情况下，选择较高的ISO值可以获得更明亮的图像，但可能会产生噪点。

😍点扩散函数（英語：point spread function，简称PSF）😍
是描述光学系统对点源解析能力的函数。
因为点源在经过任何光学系统后都会由于衍射而形成一个扩大的像点，通过测量系统的点扩展函数，能够更准确地提取图像信息。
由于光的波动性及其与它所通过光学系统的相互作用，入射光存在衍射或散射现象，产生了衍射极限，
即物面上的理想点在像面上不再为理想点。
在数学上点光源可用δ函数（脉冲）表示，输出像的光强分布叫做脉冲响应，
所以点扩散函数就是光学系统的脉冲响应函数，是一种光学系统成像性能客观定量的评价指标。

一个复杂物体的成像过程可以被看作是一个真实物体与PSF的卷积。
*/
```

![img1](https://nwzimg.wezhan.cn/contents/sitefiles2044/10221217/images/20330377.png?)



## 2023年3月14日

```c++
/*
💥我们平常的精度都是到像素级别，坐标都是整数值。
简单来将,亚像素就是0.5个像素,即我的定位不是1,也不是2, 是1.5, 这样会更精确。
*/

cv::TermCriteria(int type, int maxCount, double epsilon);

//type为 cv::TermCriteria::MAX_ITER = cv::TermCriteria::COUNT (有限迭代次数) 
//或者 cv::TermCriteria::EPS (误差参数， 接近此程度就可以退出)

//如果终止条件包含cv::TermCriteria::MAX_ITER，就是告诉算法迭代maxCount后终止；
//同理终止条件包含cv::TermCriteria::EPS，就是告诉算法在与算法收敛相关的某些度量降到epsilon以下后终止
```

## 2023年3月15日

:star:光流

```c++
/*
光流可以用于场景中的物体的运动估计，甚至用于相机相对于整个场景的自运动估计。

光流算法的理想输出是两幅图像中每个像素的速度的估计关联，或者等效的，
一幅图像中的每个像素的位移矢量，指示该像素在另外一副图像中的相对位置

图像中的每个像素都使用这种方法，则通常将其称为“稠密光流”；仅仅跟踪图像中某些点的子集则称为稀疏光流算法。

lucas-Kanade(LK)稀疏光流算法：不能跟踪运动幅度大的像素运动
“金字塔”Lk算法：允许跟踪大幅度运动像素点

三个前提：
	1、亮度恒定：场景中的目标图像像素点在帧到帧移动时不会发生改变，对于灰度图则是要求像素灰度不会随帧的跟踪改变。
	2、时间的持续性或者“微小移动”：目标像素点的运动幅度不大。或者说运动相对于帧速率（曝光时间）较慢。
	3、空间一致性：场景中属于相同表面的相邻点，具有相似的运动，并且其投影到图像平面上的点距离也比较近。
*/

/*
相机的帧率决定着设备的测量效率，如相机的帧率是30FPS，则每秒钟最多拍摄30次。 
而如果相机的速度是120FPS，如果算法够快，那么每秒钟最多检测120个产品。

快门是照相机用来控制感光片有效曝光时间的机构。
是照相机的一个重要组成部分，它的结构、形式及功能是衡量照相机档次的一个重要因素。

全局式快门：通过整幅场景在同一时间曝光实现的。Sensor所有像素点同时收集光线，同时曝光。
即在曝光开始的时候，Sensor开始收集光线；在曝光结束的时候，光线收集电路被切断。然后Sensor值读出即为一幅照片。
CCD就是Global shutter工作方式。可以通过减少曝光时间来做到提高相机在拍摄高速运动物体的图像质量，减少模糊。

卷帘式快门：与Global shutter不同，它是通过Sensor逐行曝光的方式实现的。
在曝光开始的时候，Sensor逐行扫描逐行进行曝光，直至所有像素点都被曝光。
当然，所有的动作在极短的时间内完成。不同行像元的曝光时间不同。
因为进行A/D的数据量变小故相机的帧速变高，噪声也会更小。适用于静止或者运动变化较慢的物体。

全局快门优缺点：所有像元同时曝光，曝光时间更短，但是会增加噪声。成本较于卷帘式快门更高，适合于快速运动的场景；
卷帘式快门优缺点：逐行曝光，相较于全局快门其拍摄高速物体的变形会更加明显，拍摄高速运动物体时性能不佳，会产生果冻效应，适用于静止或者运动变化较慢的物体。
*/



```

某一点的光强大小可由如下公式给出：

​	$f(x, t) = I(x(t), t) = I(x(t + dt), t+ dt) \tag{1} $

​	1、亮度恒定：

​			$\frac{\partial f}{\partial t} = 0 \tag{1.1}$

​	2、时间持续性：

​			$I_x \cdot v + I_t = \frac{\partial I}{\partial t}|_t (\frac{\partial x}{\partial t}) + \frac{\partial I}{\partial t}|_{x(t)} = 0 \rightarrow v = -\frac{I_t}{I_x}$ 



## 2023年3月16日

```c++
/*
光圈是相机镜头的一个重要参数，它控制着相机镜头的光线进入量，也就是光通量大小。
在相机镜头中，光圈是指镜头的光圈开口大小，通常用f数值来表示(镜头焦距与光圈直径之比)，也称为f-stop。
f越小光圈越大进光量越大，相反进光量越小。

光圈不仅影响相机的曝光，还会影响照片的景深。
景深是指照片中被认为是清晰的距离范围，它与光圈大小、镜头焦距和相机与被摄物体的距离有关。
当光圈变大时，景深也会变浅，被摄物体前后的景深范围减小，背景和前景都会模糊化；
反之，当光圈变小时，景深也会变深，被摄物体前后的景深范围增加，背景和前景会更清晰。

关键点主要用于三个任务：跟踪、目标识别、立体视觉
跟踪是随着场景在顺序图像流中演变而跟踪某个目标的运动的任务，其有两个类别：
1、第一个是在静态场景中跟踪对象；2、另一个是跟踪场景本身，以估计摄像机的运动。术语上跟踪指前者，后者称为视觉测距。

特征点的查找匹配：
	1、查找两幅图像的关键点
	2、计算关键点的描述符
	3、对特征点进行匹配
	4、计算最大最小描述符距离
	5、剔除不良匹配：如果两描述符之间的距离大于两倍最小描述符距离则认为是不佳匹配，同时设置最小下限30

关于cv:DMatch，该数据类型中有以下几个数据成员：queryIndx, trainIndx, imgIdx, distance. queryIndx表示的是前一副图像的特征点序号i，trainIndx表示后一副图像与前一幅图像相匹配的序号j, imgInx忽略, distance表示两特征点之间的匹配质量。

*/
```

## 2023年3月17日

```c++
/*
在OpenCV库中，很多函数的形参中包含了一个名为mask的参数。这个参数通常是一个二进制图像，用于指定操作的区域。

具体来说，mask参数通常被用来限制某些操作的范围，比如对某个图像区域进行像素操作，或者对某个轮廓区域进行绘制等。在这些操作中，只有mask参数所对应的像素值为非零时，才会对对应区域进行操作，否则对应区域的像素值不会发生任何改变。

mask参数也可以被用来过滤掉不需要处理的像素点，或者保留某个区域的像素点等。这种方式在图像分割、图像融合等任务中经常被使用。

需要注意的是，不是所有的OpenCV函数都会包含mask参数，只有在需要限制某个操作的范围时才会使用该参数。

cv::Mat cv::findFundamentalMat(cv::InputArray points1, 
							   cv::InputArray points2, 
							   int method = 8, 
							   double ransacReprojThreshold = 3.0, 
							   double confidence = 0.99, 
							   cv::OutputArray mask = noArray());//计算基础矩阵

points1, points2是特征点数组；

method为以下方法：
	cv::FM_7POINT 使用该方法可能会返回三个不同的矩阵（9X3矩阵，所有三个返回都需要考虑），对异常值敏感.点数要求大于等于8。
	cv::FM_8POINT 通过线性方程组对F进行求解，对异常值敏感。点数要求大于等于8
	cv::FM_LMEDS和cv::FM_RANSAC 点数要求大于8，对异常值不敏感，具有辨别和剔除异常值的能力。
	
当使用cv::FM_LMEDS和cv::FM_RANSAC方法时，就要使用ransacReprojThreshold和confidence参数。ransacReprojThreshold被RANSAC所用，意为从点到极线（像素）的最大距离。超过此值则认为是异常值。第二个参数被RANSAC和LMedS所用，意指期望置信度，实际上是算法的迭代次数。
*/
```

## 2023年3月18日

```c++
/*
相机主点是指摄像机光心在成像平面上的投影点，也称为光心点或成像中心。
在针孔相机模型中，光线通过针孔投射到成像平面上，而相机主点则是针孔在成像平面上的投影点，
它是成像平面的中心点，与成像平面的法向量垂直。
在相机标定和摄影测量中，相机主点通常需要被确定或估计出来，以便进行相应的计算。
*/
```

## 2023年3月20日

```c++
/*卡尔曼滤波器的实现过程是根据上一次的最优估计结果预测当前的值(先验估计值)，同时使用预测值修正当前值(先验估计值)，得到最优估计结果*/

/*
	卡尔曼滤波的使用：
	(1)选择状态量、观测量    (2)构建方程
	(3)初始化参数           (4)代入公式迭代
	(5)调节超参数(Q, R)
*/
```

## 2023年3月23日

```
图像去模糊主要分为两点：

图像非盲去模糊(通过惯导可以知道飞机的运动方向也就可以知道图像在二维平面上的移动方向，距离)

图像盲去模糊(在高速运动下，相机会和无人机刚体之间产生一个位移，旋转)

```

## 2023年3月24日

```matlab
a=[1,1,1;2,2,2;3,3,3]
b=[4,4,4;5,5,5;6,6,6]
a =

     1     1     1
     2     2     2
     3     3     3
a(:) = 
	 1
     2
     3
     1
     2
     3
     1
     2
     3
 a.^2 = 
 	1	1	1
 	4	4	4
 	9	9	9
 a.b = 
 	4	4	4
 	10	10	10
 	18	18	18
 	
```

## 2023年3月27日

```c++
/*
梯度的方向是函数变化最快的方向，所以当函数中存在边缘时，一定有较大的梯度值，相反，当图像中有比较平滑的部分时，灰度值变化较小，则相应的梯度也较小，

边缘是图像像素值快速变化的地方
*/
```

## 2023年3月28日

```
如上图所示，fftshift实现的效果就是将低频信息迁移到中心，高频信息在四周，此时用滤波器相乘，频谱图中心的信息保留，即保留了低频成分，四周的信息归0，即去除了高频成分，这就实现了我们常说的低通滤波
```

## 2023年4月23日

```
加速度计测量的是载体相对于惯性空间的绝对加速度和引力加速度之差，称作“比力”（specific force），而不是载体的运动加速度。

模糊核空间不变意为，在图像的各个像素点上，或者某块像素块上模糊核不
变，换一句话说就是模糊核不受图像深度的不同而变化。相反，模糊核空间变化意为像素点或者像素块的模糊核是不一致的，主要受图像深度影响
```

## 2023年4月24日

```
从IMU测量中重建的模糊估计的偏差来自很多方面，包括陀螺仪和加速度计误差随时间的累积，相机曝光和IMU启动时间之间的未知延迟，IMU随机游走噪声，相机内参矩阵的不准确校准，以及恒定重力假设的无效性。由于模糊估计中的小误差也会导致去模糊的假象，基于IMU的模糊核估计对于(10)中的非盲目去模糊是不够准确的
```

## 2023年4月27日

```
雅可比矩阵：在向量分析中，雅可比矩阵是函数的一阶偏导数以一定方式排列成的矩阵，其行列式称为雅可比行列式。
```

## 2023年5月6日

 - 群：只有一个良好的运算的集合称为群
   - 群的四个属性
     1. 封闭性:  $\forall a_1, a_2 \in A, a_1 \cdot a_2 \in A;$ 
     2. 结合律:  $\forall a_1, a_2, a_3 \in A, (a_1 \cdot a_2) \cdot a_3 = a_1 \cdot ( a_2 \cdot a_3 );$
     3. 幺元:  $\exists a_0 \in A, s.t. \forall a \in A, a_0 \cdot a = a \cdot a_0 = a;$
     4. 逆: $\forall a \in A, \exists a^{-1} \in A \ \ \ s.t. \ a \cdot a^{-1} = a_0.$
- 李群$so(3)$指的是由旋转矩阵组成的集合，经过对数映射后就会映射到$\mathfrak {so}(3)$ 该过程描述为$log(\boldsymbol{R})=\frac{\varphi \cdot (\boldsymbol{R} - \boldsymbol{R}^T)}{2\sin(\varphi)}$，$\mathfrak {so}(3)$ 上的矩阵是由旋转向量对应的反对称矩阵组成的。相反$\mathfrak {so}(3)$上的矩阵通过指数映射后会映射到$so(3)$上，该过程描述为$Exp(\vec{\phi}) = exp(\vec{\phi}^{\wedge})=\boldsymbol{I} + \frac{\sin{\lVert \vec{\phi} \rVert}}{\lVert \vec{\phi} \rVert} \vec \phi ^ {\wedge} + \frac{1-\cos{\lVert \vec \phi \rVert}}{\lVert \vec \phi \rVert ^ {2}}(\vec \phi ^ {\wedge})^2$  

## 2023年5月7日

- 克罗因内积：指两个不满足矩阵乘法维数的矩阵的运算, 例：

    $\boldsymbol{B}_{pq},\boldsymbol{A}_{mn}=\left(\begin{array}{1} a_{11} & \cdots & a_{1n} \\ \vdots & \ddots & \vdots \\ a_{m1} & \cdots & a_{mn}\end{array}\right), \boldsymbol{A}_{mn} \otimes \boldsymbol{B}_{pq} = \left(\begin{array}{1} a_{11} \cdot \boldsymbol{B} & \cdots & a_{1n} \cdot \boldsymbol{B} \\ \vdots & \ddots & \vdots \\ a_{m1} \cdot \boldsymbol{B} & \cdots & a_{mn} \cdot \boldsymbol{B} \end{array}\right)$

​		所得到的矩阵是一个$mq \times nq$的矩阵

## 2023年5月16日

- 傅里叶变换处理非平稳信号有天生缺陷。它只能获取**一段信号总体上包含哪些频率的成分**，但是**对各成分出现的时刻并无所知**。因此时域相差很大的两个信号，可能频谱图一样。
