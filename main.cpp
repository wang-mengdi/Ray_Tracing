#include<iostream>
#include<cstdio>
#include<algorithm>
#include<cstring>
#include<cmath>
#include<sstream>
#include<vector>
#include<stack>
#include<string>
#include<fstream>
#include<FreeImage.h>
#include"Matrix.h"
#include"Variables.h"
#include"Ray_Tracing.h"
using namespace std;
int Image_W,Image_H;
Camera Cam;
vector<Object> Objects;
string Output_Filename="output.png";
vector<Vec3> Vertices;
vector<Illumination> Lights;
stack<Transformation_Group> Current_Trans;
Object ***Hit_Object;
Vec3 **Color;

//将颜色加载到Byte数组中
BYTE* Load_Byte(void){
	BYTE *pixels;
	pixels = (BYTE*)malloc(Image_W*Image_H * 3);
	int tot=0;
	for(int j=0;j<Image_H;j++){
		for(int i=0;i<Image_W;i++){
			Vec3 col=Color[i][j];
			for(int k=2;k>=0;k--){
				double c = col[k];
				c *= 255;
				c=max(c,0.0);
				c=min(c,255.0);
				pixels[tot++]=int(c+0.5);
			}
		}
	}
	return pixels;
}

//首次计算颜色,无超采样
void First_Calc_Color(void){
	long long cnt=-1,tot=0;//按1/Progress_Base显示进度
	for(int j=0;j<Image_H;j++){
		for(int i=0;i<Image_W;i++){
			Ray R=Cam.Ray_Through_Pixel(i,j);
			Color[i][j]=Get_Color(0,R,Lights,Hit_Object[i][j]);
			tot++;
			long long k=tot*Progress_Base/(Image_H*Image_W);
			if(k>cnt){
				cnt=k;
				cout<<"渲染进度:"<<cnt<<"/"<<Progress_Base<<endl;
			}
		}
	}
}

//超采样
void Super_Sample(void){
	long long cnt=-1,tot=0,all=0;//按1/Sampling_Progress_Base显示进度
	bool **flag;
	flag=new bool*[Image_W];
	for(int i=0;i<Image_W;i++) flag[i]=new bool[Image_H];
	for(int i=0;i<Image_W;i++){
		for(int j=0;j<Image_H;j++){
			flag[i][j]=false;
			for(int k1=-1;k1<=1;k1++){
				int i1=i+k1;
				if(i1<0||i1>=Image_W) continue;
				for(int k2=-1;k2<=1;k2++){
					int j1=j+k2;
					if(j1<0||j1>=Image_H) continue;
					if(Length(Color[i][j]-Color[i1][j1])>Diff_Threshold){
						flag[i][j]=true;
						all++;
						goto OUT;
					}
				}
			}
			OUT:;
		}
	}
	double half=(Sampling_Quality-1)/2.0;
	for(int i=0;i<Image_W;i++){
		for(int j=0;j<Image_H;j++){
			Vec3 col(0,0,0);
			Object *hit;
			if(!flag[i][j]) continue;
			for(int k1=0;k1<Sampling_Quality;k1++){
				for(int k2=0;k2<Sampling_Quality;k2++){
					double x=i+(k1-half)/half,y=j+(k2-half)/half;
					Ray R=Cam.Ray_Through_Pixel(x,y);
					col=col+Get_Color(0,R,Lights,hit);
				}
			}
			Color[i][j]=col/(Sampling_Quality*Sampling_Quality);
			tot++;
			long long k=tot*Sampling_Progress_Base/all;
			if(k>cnt){
				cnt=k;
				cout<<"超采样进度:"<<cnt<<"/"<<Progress_Base<<endl;
			}
		}
	}
}

//渲染
void Render(string output){
	First_Calc_Color();
	if(Super_Sampling_Enable) Super_Sample();
	BYTE *pixels=Load_Byte();
	FreeImage_Initialise();
	FIBITMAP *img = FreeImage_ConvertFromRawBits(pixels, Image_W, Image_H, Image_W * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);
	FreeImage_Save(FIF_PNG, img, output.c_str(), 1);
	FreeImage_DeInitialise();
	free(pixels);
}

//从文件里读一堆数
bool Read_Vals(stringstream &s, const int numvals, double* values){
    for (int i = 0; i < numvals; i++) {
        s >> values[i]; 
        if (s.fail()) {
            cout << "Failed reading value " << i << " will skip\n"; 
            return false;
        }
    }
    return true; 
}

Vec3 now_ambient(0.2,0.2,0.2);
Vec3 now_diffuse;
Vec3 now_specular;
double now_shininess;
Vec3 now_emission;
void Set_Material(Object &O){
	O.ambient=now_ambient;
	O.diffuse=now_diffuse;
	O.specular=now_specular;
	O.shininess=now_shininess;
	O.emission=now_emission;
}

//读文件
void Read_File(string input){
	cout<<"加载场景文件..."<<endl;
	Current_Trans.push(Transformation_Group());
	ifstream fin;
	fin.open(input.c_str());
	string buff,cmd;
	double values[10];
	while(getline(fin,buff)){
		if((buff.find_first_not_of(" \t\r\n") != string::npos) && (buff[0] != '#')) {
			stringstream s(buff);
			s>>cmd;

			//一般命令:
			if(cmd=="size"){//设置图像大小
				if(Read_Vals(s,2,values)){
					Image_W=values[0];
					Image_H=values[1];
					Hit_Object=new Object**[Image_W];
					Color=new Vec3*[Image_W];
					for(int i=0;i<Image_W;i++){
						Hit_Object[i]=new Object*[Image_H];
						Color[i]=new Vec3[Image_H];
					}
				}
			}
			else if(cmd=="maxdepth"){//设置最大弹射次数
				if(Read_Vals(s,1,values)){
					Max_Depth=values[0];
				}
			}
			else if(cmd=="output"){//设置输出文件名
				s>>Output_Filename;
			}

			//摄像机命令:
			else if(cmd=="camera"){//设置相机位置
				if(Read_Vals(s,10,values)){
					Cam.eye=Vec3(values[0],values[1],values[2]);
					Cam.center=Vec3(values[3],values[4],values[5]);
					Cam.up=Vec3(values[6],values[7],values[8]);
					Cam.y_len=tan(values[9]/180*pi/2);
					Cam.x_len=Cam.y_len*Image_W/Image_H;
				}
			}

			//几何命令:
			else if(cmd=="vertex"){//添加一个顶点
				if(Read_Vals(s,3,values)){
					Vertices.push_back(Vec3(values[0],values[1],values[2]));
				}
			}
			else if(cmd=="sphere"){//添加一个球
				if(Read_Vals(s,4,values)){
					Object now;
					now.G=Sphere(values[0],values[1],values[2],fabs(values[3]));
					now.C=Limit_Cube(now.G);
					now.Set_Transformation(Current_Trans.top());
					Set_Material(now);
					Objects.push_back(now);
				}
			}
			else if(cmd=="tri"){//用已有的顶点添加一个三角形
				if(Read_Vals(s,3,values)){
					Object now;
					now.G=Triangle(Vertices[values[0]],Vertices[values[1]],Vertices[values[2]]);
					now.C=Limit_Cube(now.G);
					now.Set_Transformation(Current_Trans.top());
					Set_Material(now);
					Objects.push_back(now);
				}
			}

			//变换命令:
			else if(cmd=="translate"){//增加平移变换
				if(Read_Vals(s,3,values)){
					Current_Trans.top()*=Translation_Group(values[0],values[1],values[2]);
				}
			}
			else if(cmd=="rotate"){//增加旋转变换
				if(Read_Vals(s,4,values)){
					Current_Trans.top()*=Rotation_Group(values[3],Vec3(values[0],values[1],values[2]));
				}
			}
			else if(cmd=="scale"){//增加缩放变换
				if(Read_Vals(s,3,values)){
					Current_Trans.top()*=Scale_Group(values[0],values[1],values[2]);
				}
			}
			else if(cmd=="pushTransform"){//向变换栈顶加入一个单位变换
				Current_Trans.push(Transformation_Group());
			}
			else if(cmd=="popTransform"){//将栈顶变换退栈
				Current_Trans.pop();
			}

			//灯光命令:
			else if(cmd=="directional"){//增加方向光源
				if(Read_Vals(s,6,values)){
					Illumination now=Directional_Light(Vec3(-values[0],-values[1],-values[2]),Vec3(values[3],values[4],values[5]));
					now.Set_Transformation(Current_Trans.top());
					Lights.push_back(now);
				}
			}
			else if(cmd=="point"){//增加点光源
				if(Read_Vals(s,6,values)){
					Illumination now=Point_Light(Vec3(values[0],values[1],values[2]),Vec3(values[3],values[4],values[5]));
					now.Set_Transformation(Current_Trans.top());
					Lights.push_back(now);
				}
			}
			else if(cmd=="attenuation"){//设置光衰减系数
				if(Read_Vals(s,3,values)){
					for(int i=0;i<3;i++) attenuation[i]=values[i];
				}
			}
			else if(cmd=="ambient"){//设置环境光照
				if(Read_Vals(s,3,values)){
					now_ambient=Vec3(values[0],values[1],values[2]);
				}
			}

			//材质命令:
			else if(cmd=="diffuse"){//设置材质漫射光
				if(Read_Vals(s,3,values)){
					now_diffuse=Vec3(values[0],values[1],values[2]);
				}
			}
			else if(cmd=="specular"){//设置材质反射光
				if(Read_Vals(s,3,values)){
					now_specular=Vec3(values[0],values[1],values[2]);
				}
			}
			else if(cmd=="shininess"){//设置材质光泽度
				if(Read_Vals(s,1,values)){
					now_shininess=values[0];
				}
			}
			else if(cmd=="emission"){//设置表面辐射光
				if(Read_Vals(s,3,values)){
					now_emission=Vec3(values[0],values[1],values[2]);
				}
			}
		}
	}
	cout<<"场景文件加载完毕"<<endl;
	cout<<"建树中..."<<endl;
	Build_KD_Tree(Objects);
	cout<<"建树完毕,开始渲染"<<endl;
}


int main(){
	//freopen("pixels.txt", "w", stdout);
	char str[100];
	cout << "请输入场景文件名：";
	scanf("%s", str);
	Read_File(str);
	Render(Output_Filename);
	getchar();
	return 0;
}
