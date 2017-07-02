#pragma once

#include"Computation_Geometry.h"
#include"Matrix.h"
#include"Variables.h"
#include"Transform.h"
#include<vector>
#include<algorithm>
#include<iostream>
using namespace std;

ostream& operator << (ostream &os, const Vec3 &x) {
	os << "(" << x.s[0] << " " << x.s[1] << " " << x.s[2] << ")";
	return os;
}

class Camera{//相机
public:
	Vec3 eye;
	Vec3 center;
	Vec3 up;
	double x_len;
	double y_len;

	//屏幕上某一坐标所对应的射线
	Ray Ray_Through_Pixel(double i,double j){
		//先弄个新的坐标系
		//在新的坐标系中,相机位于原点看向-z方向
		Vec3 z=Normalize(eye-center);
		Vec3 x=Normalize(Cross_Product(up,z));
		Vec3 y=Cross_Product(z,x);
		double hw=(Image_W-1)/2.0,hh=(Image_H-1)/2.0;
		double alpha=(i-hw)/hw;
		double beta=(j-hh)/hh;
		Vec3 look=-1.0*z;
		look=look+alpha*x_len*x;
		look=look+beta*y_len*y;
		return Ray(eye,look);
	}
};

class Object{//一个物体
public:
	Geometry G;//几何体
	Transformation_Group T;
	Vec3 ambient;//环境光
	Vec3 diffuse;//表面的漫射光
	Vec3 specular;//表面的反射光
	double shininess;//表面的光泽度
	Vec3 emission;//表面的辐射光
	Limit_Cube C;
	vector<Vec4> s;
	void Set_Transformation(const Transformation_Group &T_){//设置变换
		if(G.type==2){//如果是球,储存变换
			T=T_;
			//考虑"旋转的椭球",这个比较奇怪......
			if (s.empty()) {
				s = vector<Vec4>(8);
				for (int i = 0; i<8; i++) {
					s[i][0] = (i & 1) ? C.high[0] : C.low[0];
					s[i][1] = (i & 2) ? C.high[0] : C.low[0];
					s[i][2] = (i & 4) ? C.high[0] : C.low[0];
					s[i][3] = 1;
				}
			}
			for (int i = 0; i < 8; i++) {
				s[i].left_multiple(T_.trans);

			}			
			C.low=Vec4(INF,INF,INF,1);
			C.high=Vec4(-INF,-INF,-INF,1);
			for(int i=0;i<8;i++){
				for(int j=0;j<3;j++){
					C.low[j]=min(C.low[j],s[i][j]);
					C.high[j]=max(C.high[j],s[i][j]);
				}
			}
		}
		else if(G.type==3){//如果是三角形,直接应用变换
			G.apply_transformation(T_);
			C=Limit_Cube(G);
			T=Transformation_Group();//记下单位变换
		}
	}
};
Object *Object_Null;

bool Ray_Object_Intersection(const Ray &R,const Object &O,double &t,Vec4 &normal){
	Ray_Object_Intersection_Count++;
	Ray R1=R;
	R1.apply_inverse_transformation(O.T);
	if(!Ray_Geometry_Intersection(R1,O.G,t,normal)) return false;
	normal.left_multiple(O.T.normaltrans);
	return true;
}

//传说中的KD树!
class KD_Tree_Node{
public:
	KD_Tree_Node *lchild,*rchild;
	Limit_Cube C;
	int n;//个数
	Object* *s;//在区域内的
};
KD_Tree_Node *Null;
Object* *Scene;
KD_Tree_Node *Scene_Root;

int Now_Cut_Dim;
bool Compare_Object(Object *a,Object *b){//按最小点比较
	return a->C.low[Now_Cut_Dim]<b->C.low[Now_Cut_Dim];
}
KD_Tree_Node* Build_KD_Tree_Node(int n,Object* *s,int dim){
	KD_Tree_Node *p=new KD_Tree_Node;
	p->n=n;
	p->s=s;
	for(int i=0;i<n;i++){
		p->C.update(s[i]->C);
	}
	if(n>1){
		Now_Cut_Dim=dim;
		sort(s,s+n,Compare_Object);
		int hf=n/2;
		p->lchild=Build_KD_Tree_Node(hf,s,(dim+1)%3);
		p->rchild=Build_KD_Tree_Node(n-hf,s+hf,(dim+1)%3);
	}
	else{
		p->lchild=p->rchild=Null;
	}
	return p;
}

void Build_KD_Tree(vector<Object> &O){
	Scene=new Object*[O.size()];
	for(int i=0;i<O.size();i++){
		Scene[i]=&O[i];
	}
	Scene_Root=Build_KD_Tree_Node(O.size(),Scene,0);
}

bool Ray_Node_Intersection(const Ray &R,KD_Tree_Node *p,double &t,Vec4 &normal,Object* &hit){
	if(!Ray_Limit_Intersection(R,p->C)) return false;
	/*if(p->n==1){
		if(Ray_Object_Intersection(R,*(p->s[0]),t,normal)){
			hit=p->s[0];
			return true;
		}
		else return false;
	}*/
	if(p->n<=Linear_Scan_Threshold){
		t=INF;
		double t1;
		Vec4 n1;
		for(int i=0;i<p->n;i++){
			if(Ray_Object_Intersection(R,*(p->s[i]),t1,n1)){
				if(t1<t){
					t=t1;
					normal=n1;
					hit=p->s[i];
				}
			}
		}
		if(t<INF) return true;
		else return false;
	}
	else{
		t=INF;
		double t1;
		Vec4 n1;
		Object* h1;
		if(Ray_Node_Intersection(R,p->lchild,t1,n1,h1)){
			if(t1<t){
				t=t1;
				normal=n1;
				hit=h1;
			}
		}
		if(Ray_Node_Intersection(R,p->rchild,t1,n1,h1)){
			if(t1<t){
				t=t1;
				normal=n1;
				hit=h1;
			}
		}
		if(t<INF) return true;
		else return false;
	}
}

bool Ray_Scene_Intersection(const Ray &R,double &t,Vec4 &normal,Object* &hit){
	return Ray_Node_Intersection(R,Scene_Root,t,normal,hit);
}

class Illumination{//光源
public:
	int type;//1是方向光,2是点光源
	Vec4 pos;
	Vec3 col;
	void Set_Transformation(const Transformation_Group &T){
		pos.left_multiple(T.trans);
	}
};

Illumination Directional_Light(Vec3 p,Vec3 c){
	Illumination ret;
	ret.type=1;
	ret.pos=Vec4(p,0);
	ret.col=c;
	return ret;
}

Illumination Point_Light(Vec3 p,Vec3 c){
	Illumination ret;
	ret.type=2;
	ret.pos=Vec4(p,1);
	ret.col=c;
	return ret;
}

bool Light_Visible(const Vec3 &p,const Illumination &L){
	Ray R;
	double tmx;
	if(L.type==1){//方向光
		R=Ray(p,L.pos.xyz()*-1);
		tmx=INF;
	}
	else if(L.type==2){//点光源
		R=Ray(p,L.pos.xyz()-p);
		tmx=Length(L.pos.xyz()-p);
	}
	R.pos=Calc_Point(R,eps);
	double t;
	Vec4 normal;
	Object *hit;
	if(Ray_Scene_Intersection(R,t,normal,hit)){
		if(t+eps<tmx) return false;
	}
	return true;
}

double Calc_Attenuation(double r){
	return 1.0/(attenuation[0]+attenuation[1]*r+attenuation[2]*r*r);
}

Vec3 Blinn_Phong(const Vec3 &look,const Vec3 &pos,const Vec3 &normal_0,const Object &O,const Illumination &light){
	//位置pos,位置向眼睛的方向是look(look已单位化),法向normal(已单位化),物体O,光源light
	Vec3 L;//光源方向
	double atten;//衰减率
	Vec3 normal = Normalize(normal_0);
	if(light.type==1){
		L=Normalize((-1*light.pos).xyz());
		atten=1;//方向光不衰减
	}
	else{
		L=light.pos.xyz()-pos;
		atten=Calc_Attenuation(Length(L));
		L=Normalize(L);
	}
	Vec3 lambert,reflect;//漫射光和反射光
	Vec3 H;H=Normalize(L+look);//半角
	double NL=normal*L,NH=normal*H;
	
	for(int i=0;i<3;i++){
		lambert[i]=atten*O.diffuse.s[i]*light.col.s[i]*max(NL,0.0);
		reflect[i]=atten*O.specular.s[i]*light.col.s[i]*pow(max(NH,0.0),O.shininess);
	}
	return lambert+reflect;
}

Vec3 Calc_Light(const Vec3 &look,const Vec3 &pos,const Vec3 &n,Object *hit,vector<Illumination> &L){
	Vec3 ret;ret=Vec3(0,0,0);
	ret=ret+hit->ambient;//环境光
	ret=ret+hit->emission;//自发光
	for(int i=0;i<L.size();i++){
		if(Light_Visible(pos,L[i])){
			ret=ret+Blinn_Phong(look,pos,n,*hit,L[i]);
		}
	}
	return ret;
}

//寻找颜色
Vec3 Get_Color(int nowdep,const Ray &R,vector<Illumination> &L,Object* &hit){
	if(nowdep>Max_Depth) return Vec3(0,0,0);
	//射线由R定义,要求R已经过单位化,O中存放所有物体
	double t=INF;
	Vec4 normal;
	hit=Object_Null;
	if(Ray_Scene_Intersection(R,t,normal,hit)){
		Vec3 pos;pos=Calc_Point(R,t).xyz();
		Vec3 look;look=Normalize(-1.0*R.dir.xyz());
		Vec3 n;n=normal.xyz();
		Vec3 lt;lt=Calc_Light(look,pos,n,hit,L);
		Vec3 rf;rf=n*2-look;
		Ray R1(pos,rf);
		R1.pos=Calc_Point(R1,eps);
		Object *h1;
		Vec3 ref=Get_Color(nowdep+1,R1,L,h1);
		for(int i=0;i<3;i++) ref[i]*=hit->specular[i];
		return lt+ref;
	}
	else return Vec3(0,0,0);
}

