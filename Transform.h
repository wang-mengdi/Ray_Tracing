#pragma once

#include"Matrix.h"
#include<cmath>

double Length(Vec3 a){
	return sqrt(a*a);
}

//向量的单位化
Vec3 Normalize(Vec3 a){
	double L=sqrt(a*a);
	return a/L;
}
/*Vec4 normalize(Vec4 a){
	double L=sqrt(a*a);
	for(int i=0;i<4;i++) a[i]/=L;
	return a;
}*/

Mat4 Transpose(Mat4 a){
	for(int i=0;i<4;i++){
		for(int j=0;j<i;j++){
			std::swap(a[i][j],a[j][i]);
		}
	}
	return a;
}

//叉积
Vec3 Cross_Product(Vec3 a,Vec3 b){
	return Vec3(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}

//投影矩阵
Mat3 Projection_Matrix(Vec3 axis){
	Mat3 ans(0);
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			ans[i][j]=axis[i]*axis[j];
		}
	}
	return ans;
}

//叉积矩阵
Mat3 Cross_Production_Matrix(Vec3 a){
	return Mat3(0,-a[2],a[1],a[2],0,-a[0],-a[1],a[0],0);
}

//旋转矩阵
Mat3 Rotation_Matrix(double degrees,Vec3 axis_){
	//绕轴axis_旋转degrees角度,axis_长度任意
	Vec3 axis=Normalize(axis_);
	double theta=degrees/180.0*pi;
	Mat3 ans(0);
	ans=ans+Mat3(1.0)*cos(theta);
	ans=ans+Projection_Matrix(axis)*(1.0-cos(theta));
	ans=ans+Cross_Production_Matrix(axis)*sin(theta);
	return ans;
}
//旋转矩阵的逆
Mat3 Inverse_Rotation_Matrix(double degrees,Vec3 axis){
	return Rotation_Matrix(-degrees,axis);
}

//缩放矩阵
Mat4 Scale_Matrix(double sx,double sy,double sz){
	return Mat4(sx,0,0,0, 0,sy,0,0, 0,0,sz,0, 0,0,0,1);
}
//缩放矩阵的逆
Mat4 Inverse_Scale_Matrix(double sx,double sy,double sz){
	return Scale_Matrix(1/sx,1/sy,1/sz);
}

//平移矩阵
Mat4 Translation_Matrix(double tx,double ty,double tz){
	return Mat4(1,0,0,tx, 0,1,0,ty, 0,0,1,tz, 0,0,0,1);
}
//平移矩阵的逆
Mat4 Inverse_Translation_Matrix(double tx,double ty,double tz){
	return Translation_Matrix(-tx,-ty,-tz);
}

class Transformation_Group{
public:
	Mat4 trans;//变换
	Mat4 invtrans;//变换的逆
	Mat4 normaltrans;//法向变换
	Transformation_Group(void){//初始化成不作任何变换
		trans=Mat4(1.0);
		invtrans=Mat4(1.0);
		normaltrans=Mat4(1.0);
	}
	//重载"变换组*=变换组"为变换的叠加
	void operator *= (const Transformation_Group &B){
		trans=trans*B.trans;
		invtrans=B.invtrans*invtrans;//注意inv的顺序
		normaltrans=normaltrans*B.normaltrans;
	}
};
//重载"变换组*变换组"为变换的叠加
Transformation_Group operator * (Transformation_Group A,Transformation_Group B){
	A*=B;
	return A;
}

//平移对应的变换组
Transformation_Group Translation_Group(double tx,double ty,double tz){
	Transformation_Group ret;
	ret.trans=Translation_Matrix(tx,ty,tz);
	ret.invtrans=Inverse_Translation_Matrix(tx,ty,tz);
	//ret.normaltrans=Transpose(ret.invtrans);
	ret.normaltrans = Mat4(1.0);
	return ret;
}

//缩放对应的变换组
Transformation_Group Scale_Group(double sx,double sy,double sz){
	Transformation_Group ret;
	ret.trans=Scale_Matrix(sx,sy,sz);
	ret.invtrans=Inverse_Scale_Matrix(sx,sy,sz);
	ret.normaltrans=Transpose(ret.invtrans);
	return ret;
}

//旋转对应的变换组
Transformation_Group Rotation_Group(double degrees,Vec3 axis){
	Transformation_Group ret;
	ret.trans=Mat4(Rotation_Matrix(degrees,axis));
	ret.invtrans=Mat4(Inverse_Rotation_Matrix(degrees,axis));
	ret.normaltrans=Transpose(ret.invtrans);
	return ret;
}