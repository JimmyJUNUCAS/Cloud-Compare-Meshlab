# Cloud-Compare-Meshlab
The project about the plug-in transform of Cloud Compare and Meshlab 

目录：

文件夹： filter_qRANSACSD
        基于meshlab框架，尺寸检测模块代码实现。
        
文件夹： qRANSAC_SD
        基于Cloud Compare框架，尺寸检测模块代码实现。
        
文件夹： references_of_Meshlab_filters
        Meshlab框架中插件模板，代表性插件学习示例。
        
 尺寸检测原理参考论文：http://pan.baidu.com/s/1jIQf7BC
 
 尺寸检测原理：
 
 meshlab 插件模板分析
 
 尺寸检测代码分析

-----------------------------------------------------------------------------------------
2017/7/7
更新点云选择方式，确保输出最恰当输出结果

2017/7/12
完成尺度检测模块，效果图如下：<br>

如图所示，显示为检测球形半径效果图，可以定位出球形半径以及中心点坐标

 ![Sphere](https://github.com/JimmyJUNUCAS/Cloud-Compare-Meshlab/blob/master/pic/Sphere.png)
 
如图所示，显示为检测圆柱效果图，可以定位出圆柱半径以及高
 
 ![Cylinder](https://github.com/JimmyJUNUCAS/Cloud-Compare-Meshlab/blob/master/pic/Cylinder.png)

如图所示，显示为检测圆台效果图，可以定位出圆台上下半径以及高 
 
 ![Cone](https://github.com/JimmyJUNUCAS/Cloud-Compare-Meshlab/blob/master/pic/Cone.png)
 
如图所示，显示为检测圆环效果图，可以定位出圆环内外圆半径以及高 
 
 ![Torus](https://github.com/JimmyJUNUCAS/Cloud-Compare-Meshlab/blob/master/pic/Torus.png)
