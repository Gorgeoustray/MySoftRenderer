# MySoftRenderer
基于 C++ 和 Win32 API 实现了简单的软渲染器，可用 Visual Studio 2019 编译运行。

实现的主要功能包括：
+ 3D 数学库：三维向量、四维向量、四维矩阵
+ 渲染管线：顶点 MVP 变换、背面剔除、裁剪、光栅化、透视矫正、纹理过滤、漫反射光照计算等
+ 相机控制：类似 FPS 游戏的控制方式，鼠标旋转镜头，WASD 平移，QE 上下
+ 帧率显示
+ 支持读入 obj 格式的模型
+ 两种渲染模式：线框、纹理贴图

线框模式：
![线框模式](https://img-blog.csdnimg.cn/6cd9a4b9cfec4f1c8482d187f6fd6a5b.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAZ29yZ2VvdXN0cmF5,size_20,color_FFFFFF,t_70,g_se,x_16)
纹理贴图模式：
![在这里插入图片描述](https://img-blog.csdnimg.cn/5c61c60bc8844427abd0e6beb6d85099.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAZ29yZ2VvdXN0cmF5,size_20,color_FFFFFF,t_70,g_se,x_16)
我在学习了 [GAMES101](http://games-cn.org/intro-graphics/) 之后，对图形学有了初步的了解。在写这个软渲染器的过程中复习了图形学的知识（当然大部分时间还是在处理 bug）。并且，用代码来展现出三维空间的模型，对我这样的初学者来说成就感爆棚。但是我没有能力直接徒手写一个出来，写的过程中参考了以下资料：
+ [GAMES101](http://games-cn.org/intro-graphics/)：图形学基础知识，包括 3D 数学、MVP 矩阵、光栅化、着色等等
+ [mini3d]()：其实写的时候参考了很多别人的软渲染器，这个对我的帮助最大。首先是为我提供了 Win32 API 框架，让我不用处理 Win32 API 相关的细节，只需要专注于用 C++ 实现数学库和渲染出每一帧的图片；其次是代码可读性很强，提供了精炼的渲染管线参考，我把这个项目读懂之后，渲染器主要的设计都是仿照他写的，在此基础上继续添加了新的功能
+ [RenderHelp](https://github.com/skywind3000/RenderHelp)：和上面的项目是同一个作者，参考了 obj 格式文件的读取
+ [3D游戏编程大师技巧](https://book.douban.com/subject/1321769/)：项目中部分功能的实现参考了这本书，包括剪裁、纹理过滤、mipmap 等等。这本书最大的优点是几乎所有提到的功能都有完整的代码示例，讲解非常细致
+ [Windows 程序设计：第5版](https://book.douban.com/subject/1088168/)：虽然不用过于关注 Win32 API 的细节，但是还是要能看懂 mini3d 项目中的代码大概做了些什么事情，所以这本书也看了几章
+ [tinyrenderer](https://github.com/ssloy/tinyrenderer)：使用了项目中的模型

代码分成了 3 个头文件和 1 个源文件，后面有时间的话梳理一下代码，分几篇文章整理一下具体的实现，其实很多功能都还有需要改进的地方。
