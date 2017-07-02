#pragma once

const double pi=3.1415926535897932384626433;
const double eps=1e-4;
const double eps_edge=1e-2;
const double INF=1e10;
const int Linear_Scan_Threshold=5;//物体数量低于此值时用线性扫描
extern int Image_W,Image_H;
extern int Max_Depth=5;
extern double attenuation[3]={1,0,0};//衰减参数
extern int Object_Num=0;
extern long long Ray_Object_Intersection_Count=0;
const double Diff_Threshold=0.8;//相邻格子色差(颜色向量之差的模长)超过这个值时将开启超采样
const int Sampling_Quality=3;//超采样将每个像素分成几乘几的格子
const int Progress_Base=20;//以几分之一显示进度
const int Sampling_Progress_Base=20;//以几分之一显示超采样进度
const bool Super_Sampling_Enable=false;//是否打开超采样