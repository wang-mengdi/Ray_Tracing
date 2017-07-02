#pragma once

/*
定义了若干向量和矩阵,及其基本运算
它们的元素类型均为double
这里的矩阵是行优先的
*/

#include<iostream>

//矩阵和向量的定义

//3*3矩阵
class Mat3{
public:
	double s[3][3];
	Mat3(void){
		memset(s,0,sizeof(s));
	}
	Mat3(double x){
		memset(s,0,sizeof(s));
		for(int i=0;i<3;i++) s[i][i]=x;
	}
	Mat3(double x00,double x01,double x02,double x10,double x11,double x12,double x20,double x21,double x22){
		s[0][0]=x00,s[0][1]=x01,s[0][2]=x02;
		s[1][0]=x10,s[1][1]=x11,s[1][2]=x12;
		s[2][0]=x20,s[2][1]=x21,s[2][2]=x22;
	}
	double* operator [] (int k){
		return s[k];
	}
};

//4*4矩阵
class Mat4{
public:
	double s[4][4];
	Mat4(void){
		memset(s,0,sizeof(s));
	}
	Mat4(double x){
		memset(s,0,sizeof(s));
		for(int i=0;i<4;i++) s[i][i]=x;
	}
	Mat4(double x00,double x01,double x02,double x03,double x10,double x11,double x12,double x13,double x20,double x21,double x22,double x23,double x30,double x31,double x32,double x33){
		s[0][0]=x00,s[0][1]=x01,s[0][2]=x02,s[0][3]=x03;
		s[1][0]=x10,s[1][1]=x11,s[1][2]=x12,s[1][3]=x13;
		s[2][0]=x20,s[2][1]=x21,s[2][2]=x22,s[2][3]=x23;
		s[3][0]=x30,s[3][1]=x31,s[3][2]=x32,s[3][3]=x33;
	}
	Mat4(Mat3 a){
		//保持齐次坐标不变
		memset(s,0,sizeof(s));
		for(int i=0;i<3;i++) for(int j=0;j<3;j++) s[i][j]=a[i][j];
		s[3][3]=1;
	}
	double* operator [] (int k){
		return s[k];
	}
	void left_multiple(const Mat4&);
	void right_multiple(const Mat4&);
};

//3*1向量
class Vec3{
public:
	double s[3];
	Vec3(double x0=0,double x1=0,double x2=0){
		s[0]=x0,s[1]=x1,s[2]=x2;
	}
	double& operator [] (int k){
		return s[k];
	}
	void operator += (const Vec3 &b){
		s[0]+=b.s[0];
		s[1]+=b.s[1];
		s[2]+=b.s[2];
	}
	void left_multiple(const Mat4&);
};

//4*1向量
class Vec4{
public:
	double s[4];
	Vec4(double x0=0,double x1=0,double x2=0,double x3=0){
		s[0]=x0,s[1]=x1,s[2]=x2,s[3]=x3;
	}
	Vec4(Vec3 a,double x3){
		for(int i=0;i<3;i++) s[i]=a[i];
		s[3]=x3;
	}
	double& operator [] (int k){
		return s[k];
	}
	Vec3 xyz(void)const{
		return Vec3(s[0],s[1],s[2]);
	}
	void left_multiple(const Mat4&);
};

//重载向量的数乘(包括乘除)
Vec3 operator * (double a,Vec3 b){
	for(int i=0;i<3;i++) b[i]*=a;
	return b;
}
Vec3 operator * (Vec3 b,double a){
	for(int i=0;i<3;i++) b[i]*=a;
	return b;
}
Vec3 operator / (Vec3 a,double b){
	for(int i=0;i<3;i++) a[i]/=b;
	return a;
}
Vec4 operator * (double a,Vec4 b){
	for(int i=0;i<4;i++) b[i]*=a;
	return b;
}
Vec4 operator * (Vec4 b,double a){
	for(int i=0;i<4;i++) b[i]*=a;
	return b;
}
Vec4 operator / (Vec4 a,double b){
	for(int i=0;i<4;i++) a[i]/=b;
	return a;
}

//重载向量加减
Vec3 operator + (Vec3 a,Vec3 b){
	for(int i=0;i<3;i++) a[i]+=b[i];
	return a;
}
Vec3 operator - (Vec3 a,Vec3 b){
	for(int i=0;i<3;i++) a[i]-=b[i];
	return a;
}
Vec4 operator + (Vec4 a,Vec4 b){
	for(int i=0;i<4;i++) a[i]+=b[i];
	return a;
}
Vec4 operator - (Vec4 a,Vec4 b){
	for(int i=0;i<4;i++) a[i]-=b[i];
	return a;
}

//重载"向量*向量"为向量之间的点积
double operator * (const Vec3 &a,const Vec3 &b){
	double ret=0;
	for(int i=0;i<3;i++) ret+=a.s[i]*b.s[i];
	return ret;
}
double operator * (const Vec4 &a,const Vec4 &b){
	double ret=0;
	for(int i=0;i<4;i++) ret+=a.s[i]*b.s[i];
	return ret;
}

//重载"矩阵*向量"为矩阵乘法
Vec3 operator * (Mat3 a,Vec3 b){
	Vec3 c;
	for(int i=0;i<3;i++){
		c[i]=0;
		for(int k=0;k<3;k++){
			c[i]+=a[i][k]*b[k];
		}
	}
	return c;
}
Vec4 operator * (Mat4 a,Vec4 b){
	Vec4 c;
	for(int i=0;i<4;i++){
		c[i]=0;
		for(int k=0;k<4;k++){
			c[i]+=a[i][k]*b[k];
		}
	}
	return c;
}

//重载"数*矩阵"与"矩阵*数"为矩阵的数量积
Mat3 operator * (double a,Mat3 b){
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++) b[i][j]*=a;
	}
	return b;
}
Mat3 operator * (Mat3 b,double a){
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++) b[i][j]*=a;
	}
	return b;
}
Mat4 operator * (double a,Mat4 b){
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++) b[i][j]*=a;
	}
	return b;
}
Mat4 operator * (Mat4 b,double a){
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++) b[i][j]*=a;
	}
	return b;
}

//重载矩阵加法
Mat3 operator + (Mat3 a,Mat3 b){
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++) a[i][j]+=b[i][j];
	}
	return a;
}

Mat4 operator + (Mat4 a,Mat4 b){
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++) a[i][j]+=b[i][j];
	}
	return a;
}

//重载"矩阵*矩阵"为矩阵乘法
Mat3 operator * (Mat3 a,Mat3 b){
	Mat3 c;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			c[i][j]=0;
			for(int k=0;k<3;k++){
				c[i][j]+=a[i][k]*b[k][j];
			}
		}
	}
	return c;
}
Mat4 operator * (Mat4 a,Mat4 b){
	Mat4 c;
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			c[i][j]=0;
			for(int k=0;k<4;k++){
				c[i][j]+=a[i][k]*b[k][j];
			}
		}
	}
	return c;
}

//定义"矩阵左乘矩阵"和"矩阵右乘矩阵"

//左乘
void Mat4::left_multiple(const Mat4 &b){
	static double s1[4][4];
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			s1[i][j]=0;
			for(int k=0;k<4;k++){
				s1[i][j]+=b.s[i][k]*s[k][j];
			}
		}
	}
	memcpy(s,s1,sizeof(s));
}
//右乘
void Mat4::right_multiple (const Mat4 &b){
	static double s1[4][4];
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			s1[i][j]=0;
			for(int k=0;k<4;k++){
				s1[i][j]+=s[i][k]*b.s[k][j];
			}
		}
	}
	memcpy(s,s1,sizeof(s));
}

//定义"向量左乘矩阵"
void Vec4::left_multiple(const Mat4 &b){
	static double s1[4];
	for(int i=0;i<4;i++){
		s1[i]=0;
		for(int k=0;k<4;k++){
			s1[i]+=b.s[i][k]*s[k];
		}
	}
	memcpy(s,s1,sizeof(s));
}

void Vec3::left_multiple(const Mat4 &b){//只操作左上的3*3,也是够强行的
	static double s1[3];
	for(int i=0;i<3;i++){
		s1[i]=0;
		for(int k=0;k<3;k++){
			s1[i]+=b.s[i][k]*s[k];
		}
	}
	memcpy(s,s1,sizeof(s));
}