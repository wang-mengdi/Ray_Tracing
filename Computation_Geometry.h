#pragma once

#include"Matrix.h"
#include"Transform.h"
#include"Variables.h"
#include<cmath>
#include<iostream>
#include<algorithm>
using namespace std;

//定义几何体,以及各种几何运算

//射线
class Ray{
public:
	Vec4 pos;
	Vec4 dir;
	/*Ray(Vec4 a,Vec4 b){
		pos=a;
		dir=b;
	}*/
	Ray(Vec3 a,Vec3 b){
		pos=Vec4(a,1.0);
		//方向向量为单位向量且齐次坐标为零
		b=Normalize(b);
		dir=Vec4(b,0.0);
	}
	Ray(){}
	void apply_transformation(const Transformation_Group &T){
		pos.left_multiple(T.trans);
		dir.left_multiple(T.trans);
	}
	void apply_inverse_transformation(const Transformation_Group &T){
		pos.left_multiple(T.invtrans);
		dir.left_multiple(T.invtrans);
	}
};

Vec4 Calc_Point(const Ray &R,double t){
	return R.pos+t*R.dir;
}

//几何体
class Geometry{
public:
	int type;//2是球,3是三角形
	int n;
	Vec4 *s;
	//重载"几何体*=变换组"为:对几何体中的每个顶点应用变换矩阵
	void apply_transformation(const Transformation_Group &T){
		for(int i=0;i<n;i++) s[i].left_multiple(T.trans);
	}
};

Geometry Sphere(double x,double y,double z,double r){
	Geometry ret;
	ret.type=2;
	ret.n=2;
	ret.s=new Vec4[2];
	ret.s[0]=Vec4(x,y,z,1);
	ret.s[1]=Vec4(r,0,0,0);
	return ret;
}

Geometry Triangle(Vec3 a,Vec3 b,Vec3 c){
	Geometry ret;
	ret.type=3;
	ret.n=3;
	ret.s=new Vec4[3];
	ret.s[0]=Vec4(a,1.0);
	ret.s[1]=Vec4(b,1.0);
	ret.s[2]=Vec4(c,1.0);
	return ret;
}

class Limit_Cube{
public:
	Vec4 low;
	Vec4 high;
	Limit_Cube(){
		low=Vec4(INF,INF,INF,1);
		high=Vec4(-INF,-INF,-INF,1);
	}
	Limit_Cube(const Geometry &G){
		if(G.type==2){//球
			low=Vec4(G.s[0].xyz()-Vec3(G.s[1].s[0],G.s[1].s[0],G.s[1].s[0]),1);
			high=Vec4(G.s[0].xyz()+Vec3(G.s[1].s[0],G.s[1].s[0],G.s[1].s[0]),1);
		}
		else if(G.type==3){//三角形
			for(int i=0;i<3;i++){
				low[i]=INF,high[i]=-INF;
				for(int j=0;j<3;j++){
					low[i]=min(low[i],G.s[j].s[i]);
					high[i]=max(high[i],G.s[j].s[i]);
				}
			}
			low[3]=high[3]=1;
		}
	}
	void update(const Limit_Cube &C){
		for(int i=0;i<3;i++){
			low[i]=min(low[i],C.low.s[i]);
			high[i]=max(high[i],C.high.s[i]);
		}
	}
	void apply_transformation(const Transformation_Group &G){
		low.left_multiple(G.trans);
		high.left_multiple(G.trans);
	}
};

bool Ray_Limit_Intersection(const Ray &R,const Limit_Cube &C){
	//return true;
	static double l,r,t1,t2;
	l=-INF,r=INF;
	for(int i=0;i<3;i++){
		if(fabs(R.dir.s[i])<eps){
			if(C.low.s[i]-eps<=R.pos.s[i]&&R.pos.s[i]<=C.high.s[i]+eps){
				t1=-INF;
				t2=INF;
			}
			else return false;
		}
		else{
			t1=(C.low.s[i]-R.pos.s[i])/R.dir.s[i];
			t2=(C.high.s[i]-R.pos.s[i])/R.dir.s[i];
			if(t1>=t2) swap(t1,t2);
		}
		l=max(l,t1);
		r=min(r,t2);
	}
	if(l>r+eps) return false;
	return true;
}

//解一元二次方程
bool Solve_Quadratic(const double &a,const double &b,const double &c,double &x1,double &x2){
	static double dlt;dlt=b*b-4*a*c;
	if(dlt<0) return false;
	dlt=sqrt(dlt);
	x1=(-b-dlt)/(a*2);
	x2=(-b+dlt)/(a*2);
	if(x1>x2) std::swap(x1,x2);
	return true;
}
//射线和圆求交
bool Ray_Sphere_Intersection(const Vec3 &P0,const Vec3 &P1,const Vec3 &C,double sqr,double &ret,Vec4 &normal){
	//射线方程为P=P0+t*P1,圆心为C,圆半径的平方为sqr
	static double a;a=P1*P1;
	static double b;b=P1*(P0-C)*2;
	static double c;c=(P0-C)*(P0-C)-sqr;
	static double t1,t2;
	if(!Solve_Quadratic(a,b,c,t1,t2)) return false;
	if(t1<eps){
		if(t2<eps) return false;
		else ret=t2;
	}
	else ret=t1;
	normal=Vec4(Normalize(P0+ret*P1-C),1.0);
	return true;
}
/*
//射线和平面求交
bool Ray_Plane_Intersection(Vec3 P0,Vec3 P1,Vec3 A,Vec3 n,double &t){
	//射线方程为P=P0+t*P1,平面上一点为A,平面法向为n
	double a=P1*n;
	if(fabs(a)<eps) return false;
	double b=A*n-P0*n;
	t=b/a;
	return true;
}*/
//射线和三角形求交,同时作重心分解
bool Ray_Triangle_Intersection(const Vec3 &P0,const Vec3 &P1,const Vec3 &a,const Vec3 &b,const Vec3 &c,double &t,Vec4 &normal){
	//射线方程为P=P0+t*P1,三角形为abc,交点重心分解为alpha*a+beta*b+gamma*c
	Vec3 E1=b-a,E2=c-a,T=P0-a;
	Vec3 X=Cross_Product(P1,E2),Y=Cross_Product(T,E1);
	static double k;k=X*E1;
	if(fabs(k)<eps) return false;
	t=(Y*E2)/k;
	static double beta;beta=(X*T)/k;
	static double gamma;gamma=(Y*P1)/k;
	static double alpha;alpha=1-beta-gamma;
	if(t<eps) return false;
	if(alpha<-eps) return false;
	if(beta<-eps) return false;
	if(gamma<-eps) return false;
	normal=Vec4(Normalize(Cross_Product(b-a,c-a)),1.0);
	return true;
}

//射线和几何体求交
bool Ray_Geometry_Intersection(const Ray &R,const Geometry &G,double &t,Vec4 &normal){//计算距离和交点法向
	if(G.type==2){
		return Ray_Sphere_Intersection(R.pos.xyz(),R.dir.xyz(),G.s[0].xyz(),G.s[1]*G.s[1],t,normal);
	}
	else if(G.type==3){
		return Ray_Triangle_Intersection(R.pos.xyz(),R.dir.xyz(),G.s[0].xyz(),G.s[1].xyz(),G.s[2].xyz(),t,normal);
	}
	return false;
}
