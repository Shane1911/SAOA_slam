 * @Date: 2023-01-20 22:56:13
 * @LastEditTime: 2023-01-20 23:21:24
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
###  &emsp; 当再次踏上阿根廷的土地时，那个曾经写下这些文字的人已经远去，而重新整理和润色这些日记的我，不再是曾经的那个我了。-----------《让我们彼此理解》-切.格瓦拉。
&emsp; 这是一个关于激光slam的问题，不复杂，但绝对精彩。此处给初学者介绍一种利用第三方库实现的建图功能，以便很好的了解这项技术。
### Dependencies：
PCL
Eigen
OpenMP
VGICP
CUDA (optional)
### RUN:
'''
mkdir build && cd build
cmake .. 
make -j8
'''
### instruction:
建图效果：
<div align=center><img width = '850' height ='300' src =map.png></div> 
位姿协方差：
<div align=center><img width = '850' height ='300' src =cov.png></div> 
