#pragma once

const double pi=3.1415926535897932384626433;
const double eps=1e-4;
const double eps_edge=1e-2;
const double INF=1e10;
const int Linear_Scan_Threshold=5;//�����������ڴ�ֵʱ������ɨ��
extern int Image_W,Image_H;
extern int Max_Depth=5;
extern double attenuation[3]={1,0,0};//˥������
extern int Object_Num=0;
extern long long Ray_Object_Intersection_Count=0;
const double Diff_Threshold=0.8;//���ڸ���ɫ��(��ɫ����֮���ģ��)�������ֵʱ������������
const int Sampling_Quality=3;//��������ÿ�����طֳɼ��˼��ĸ���
const int Progress_Base=20;//�Լ���֮һ��ʾ����
const int Sampling_Progress_Base=20;//�Լ���֮һ��ʾ����������
const bool Super_Sampling_Enable=false;//�Ƿ�򿪳�����