# SNL
想要写一个用于导航领域的C++标准库，~~不清楚要不要使用模板~~

# ***.gitignore***

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

## 2023年03月04日

#### 17:40:51

​	思考良久，决定还是记录时间来描述当前的想法。目前困惑于该如何用一个无序容器来容纳不同的ROS消息体类型。
正在考虑使用tuple来解决这个问题，或者用可变参数模板。C++这些部分都没有深入学习，难以下手，用时方恨少。

#### 19:51:31

​	突然意识我可以写一个消息类来封装ros的这些消息体。

