#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include<librealsense2/rsutil.h>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include "cv-helpers.hpp"
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
//#include <GL/glew.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/SVD"
using namespace cv;
using namespace std;
using namespace rs2;

//const int width = 1280;
//const int height = 720;
const int width = 640;
const int height = 480;
const int fps = 30;
GLint xangle = 0, yangle = 0, zangle = 0;
void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
	Mat hh;
	hh = *(Mat*)userdata;
	Point p(x, y);
	switch (EVENT)
	{
	case EVENT_LBUTTONDOWN:
	{
		printf("b=%d\t", hh.at<Vec3b>(p)[0]);
		printf("g=%d\t", hh.at<Vec3b>(p)[1]);
		printf("r=%d\n", hh.at<Vec3b>(p)[2]);
		printf("x=%d\t", x);
		printf("y=%d\n", y);
		//		std::cout << "x:" << x << ";" <<y ;
	}
	break;

	}
}
int U = 0, V = 0;
bool infraredflag = false;
void on_mouse2(int EVENT, int x, int y, int flags, void* userdata)
{
	Mat hh;
	hh = *(Mat*)userdata;
	Point p(x, y);
	switch (EVENT)
	{
	case EVENT_LBUTTONDOWN:
	{
		//		printf("b=%d\t", hh.at<Vec3b>(p)[0]);
		//		printf("g=%d\t", hh.at<Vec3b>(p)[1]);
		//		printf("r=%d\n", hh.at<Vec3b>(p)[2]);
		printf("x=%d\t", x);
		printf("y=%d\n", y);
		U = x; V = y;
		infraredflag = true;
		//		std::cout << "x:" << x << ";" <<y ;
	}
	break;

	}
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS)
		return;
	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		glfwSetWindowShouldClose(window, GL_TRUE);

		break;
	case GLFW_KEY_LEFT:
		yangle = (yangle + 5) % 360;
		//		glRotatef(zangle, 0.0f, 0.0f, 1.0f);
		std::cout << "KEY_LEFT pressed!" << yangle << "zangle now \r";
		break;
	case GLFW_KEY_RIGHT:
		yangle = (yangle - 5) % 360;
		//		glRotatef(zangle, 0.0f, 0.0f, 1.0f);
		std::cout << "KEY_RIGHT pressed!" << yangle << "zangle now \r";
		break;
	case GLFW_KEY_UP:
		xangle = (xangle + 5) % 360;
		std::cout << "KEY_UP pressed!" << xangle << "yangle now \r";
		break;
	case GLFW_KEY_DOWN:
		xangle = (xangle - 5) % 360;
		std::cout << "KEY_DOWN pressed!" << xangle << "yangle now \r";
		break;
	default:
		break;
	}
}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS) switch (button)
	{
	case GLFW_MOUSE_BUTTON_LEFT:
		std::cout << "Mosue left button clicked!" << zangle << "zangle now \r";
		zangle = (zangle + 15) % 360;
		break;
	case GLFW_MOUSE_BUTTON_MIDDLE:
		std::cout << "Mosue middle button clicked!" << zangle << "zangle now \r";
		break;
	case GLFW_MOUSE_BUTTON_RIGHT:
		std::cout << "Mosue right button clicked!" << zangle << "yangle now \r";
		zangle = (zangle + 15) % 360;
		break;
	default:
		return;
	}
	return;
}
void cursor_position_callback(GLFWwindow* window, double x, double y)
{
	//sprintf(msg, "Mouse position move to [%d:%d]", int(x), int(y));
//	std::cout << "Mouse position move to" << int(x) << "," << int(y) << " \r";
	GLfloat xpos, ypos;
	xpos = float((x - width / 2) / width) * 2;
	ypos = float(0 - (y - height / 2) / height) * 2;
//	std::cout << "Mouse position move to" << xpos << "," << ypos << " \r";
	return;
}
void pose_estimation_3d3d(
	const vector<Point3f>& pts1,
	const vector<Point3f>& pts2,
	Mat& R, Mat& t
);
int main()
{
	bool monitorflag = true;
	bool flag = false;
	bool posecomputer_enable_flag = true;//只允许计算一次R和t就够了
	float x = 0, y = 0, z = 0;
	float xout = 0, yout = 0, zout = 0;
	float Mtranslatex=0.0, Mtranslatey = 0.0, Mtranslatez = 0.0;
	float x1 = 0.0, y1 = 0.0, z1 = 0.0;//标记点1初始位置
	float x2 = 0.0, y2 = 0.0, z2 = 0.0;//标记点2初始位置
	float x3 = 0.0, y3 = 0.0, z3 = 0.0;//标记点3初始位置
	float xo1 = 0.0, yo1 = 0.0, zo1 = 0.0;//标记点1末位置
	float xo2 = 0.0, yo2 = 0.0, zo2 = 0.0;//标记点2末位置
	float xo3 = 0.0, yo3 = 0.0, zo3 = 0.0;//标记点3末位置
	float Arotationx = 0.0, Arotationy = 0.0, Arotationz = 0.0;
	float A0 = 0.0, A1 = 0.0, A2 = 0.0;//相机初始旋转角度
//	float alpha=0.0, beta=0.0, gamma=0.0;
	float a = 0.0, b = 0.0, g = 0.0;
//	float R[3][3] = {0.0};//旋转矩阵
	vector<Point3f> pts1, pts2;
	Mat R = (Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0), t = (Mat_<double>(3, 1) << 0, 0, 0);
	float Rcolor = 0.0, Gcolor = 0, Bcolor = 0;
	char pstr, cstr;
	int counter = 0, counterN=0, counterQ=0;//扫描的位置计数，第一个很重要，目前设置数据文件中第一个出现M的位置为初始位置。回头再用此程序读取初始化初始位置。
	FILE *fp = fopen("pointcloud.txt", "r");//想用FILE指针，需要在C/C++预编译器中的定义栏加上：_CRT_SECURE_NO_WARNINGS
	FILE *fp2 = fopen("color.txt", "r");//
	FILE *fp3 = fopen("monitor.txt", "w");
	GLFWwindow* window;//Initialization
	if (!glfwInit())/* Initialize the library */
		return -1;
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);

	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);

	//	FILE *fp;//想用FILE指针，需要在C/C++预编译器中的定义栏加上：_CRT_SECURE_NO_WARNINGS
	//	fp = fopen("ball.txt", "w");
	rs2::align align_to(RS2_STREAM_COLOR);
	Mat  result;
	Mat ElementStructopen = getStructuringElement(MORPH_ELLIPSE, Size(18, 18));
	Mat ElementStructclose = getStructuringElement(MORPH_ELLIPSE, Size(18, 18));

	//	const char* left_win = "left_Image";
	//	namedWindow(left_win, WINDOW_AUTOSIZE);

	char LName[100];//保存图片用的名字字符串 

	rs2::pipeline pipe;//Pipeline
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
	pipe_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	//	pipe_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
	pipe_config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

	rs2::pipeline_profile profile = pipe.start(pipe_config);
	glfwSwapInterval(1);//前后缓冲区显示间隔
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	//while (cvGetWindowHandle(depth_win) && cvGetWindowHandle(right_win) && cvGetWindowHandle(left_win) && cvGetWindowHandle(color_win)) // Application still alive?
	while (!glfwWindowShouldClose(window))
	{
		//堵塞程序直到新的一帧捕获
		rs2::frameset frameset = pipe.wait_for_frames();
		// Make sure the frameset is spatialy aligned (each pixel in depth image corresponds to the same pixel in the color image)
//		rs2::frameset aligned_set = align_to.process(frameset);//经测试，此句一定不能要，深度与彩色对齐后，导致定位出现明显偏差
		//原因可能是深度图扩散后贴合到彩色图上，导致坐标对应点会产生空洞（彩色图不受影响，每个坐标都有深度）。
        
        //取红外1图、深度图和彩色图																								
		video_frame ir_frame_left = frameset.get_infrared_frame(1);
		depth_frame depth = frameset.get_depth_frame();


		//获取内参
		auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();		
		//		rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
		const auto intrinDepth = depth_stream.get_intrinsics();
		const auto intrinColor = color_stream.get_intrinsics();
		//直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
		rs2_extrinsics  extrinDepth2Color;
		rs2_error *error;
		rs2_get_extrinsics(depth_stream, color_stream, &extrinDepth2Color, &error);

		Mat dMat_left(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
		//Mat dMat_right(Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());
		//Mat depth_image(Size(width, height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
		//Mat color_image(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		frame color_frame = frameset.get_color_frame();
		auto color_image = frame_to_mat(color_frame);//这两句与前一句效果相同，只是这样更自由，所以选择了它们
		setMouseCallback("红外1图像", on_mouse2, &dMat_left);
		blur(dMat_left, result, Size(3, 3));// 先用使用 3x3内核来降噪
		threshold(result, result, 180, 255, CV_THRESH_BINARY);
		//Canny(result, result, 0, 30, 3);//边缘化  */
		dilate(result, result, ElementStructopen);//联合小球上的散斑
		erode(result, result, ElementStructclose);//整体变细
												  //morphologyEx(result, result, MORPH_OPEN, ElementStructopen);
												  // morphologyEx(result, result, MORPH_CLOSE, ElementStructopen);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		double area;

		findContours(result, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);////CV_CHAIN_APPROX_NONE
		Mat contoursImage(result.rows, result.cols, CV_8U, Scalar(0));
		//vector<Rect> boundRect(contours.size());

		float outpoint[10][3];
		for (int i = 0; i<10; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				outpoint[i][j] = 0.0;
			}
		}
		for (int i = 0; i<contours.size(); i++)//!!!此处如果contours.size()大于上面的10，会导致outpoint[10][3]溢出，注意！
		{
			//if (hierarchy[i][3] != -1)
			area = fabs(contourArea(contours[i], true)); //获取当前轮廓面积 
			if (flag)
			{
				//fprintf(fp, "%d\n", contours.size());		
				//				     fprintf(fp, " 第%d个面积为%f ", i, area);
			}
			if (area > 4 && area < 400)
			{
				drawContours(contoursImage, contours, i, Scalar(255), 1);
				//boundRect[i] = boundingRect(Mat(contours[i]));
				//rectangle(contoursImage, boundRect[i].tl(), boundRect[i].br(),Scalar(255), 1, 1, 0);
				Rect R = boundingRect(Mat(contours[i]));
				rectangle(dMat_left, R, Scalar(255), 1, 1, 0);
				//				rectangle(color_image, R, Scalar(255), 1, 1, 0);
				Point pcenter, pcolor;
				int upixel[2]; // From pixel
				float upoint[3]; // From point (in 3D)
				pcenter.x = (R.tl().x + R.br().x) / 2;
				pcenter.y = (R.tl().y + R.br().y) / 2;
				//轮廓画实心点
				circle(contoursImage, pcenter, 3, Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
				upixel[0] = pcenter.x;
				upixel[1] = pcenter.y;
				float udist = depth.get_distance(upixel[0], upixel[1]); //获得的距离单位是米
				if (udist > 0.2&&udist < 2.0)
				{
					float pd_uv[2], pc_uv[2], depth_m;
					//空间点定义
					float Pdc3[3], Pcc3[3];
					pd_uv[0] = upixel[0]; pd_uv[1] = upixel[1]; depth_m = udist;
					//将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
					rs2_deproject_pixel_to_point(Pdc3, &intrinDepth, pd_uv, depth_m);
					//将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
					rs2_transform_point_to_point(Pcc3, &extrinDepth2Color, Pdc3);//要的是Pcc3即反光小球的空间坐标。猜测Pdc3和Pcc3是相等的，因深度和彩色图在这里是对齐的
					outpoint[i][0] = Pcc3[0]*1000; outpoint[i][1] = Pcc3[1] * 1000; outpoint[i][2] = Pcc3[2] * 1000;//*1000使单位转化为mm

					//将彩色摄像头坐标系下的深度三维点映射到二维平面上
					rs2_project_point_to_pixel(pc_uv, &intrinColor, Pcc3);
					pcolor.x = pc_uv[0]; pcolor.y = pc_uv[1];
					//彩图画实心点
					circle(color_image, pcolor, 3, Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
				}
					if (infraredflag)
					{
						float udist2 = depth.get_distance(U, V);
						//			printf("z=%f\n", udist);
						Point  pcolor2;
						float pd_uv2[2], pc_uv2[2], depth_m2;
						//空间点定义
						float Pdc32[3], Pcc32[3];


						pd_uv2[0] = U; pd_uv2[1] = V; depth_m2 = udist2;
						//将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
						rs2_deproject_pixel_to_point(Pdc32, &intrinDepth, pd_uv2, depth_m2);
						//将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
						rs2_transform_point_to_point(Pcc32, &extrinDepth2Color, Pdc32);
						//将彩色摄像头坐标系下的深度三维点映射到二维平面上
						rs2_project_point_to_pixel(pc_uv2, &intrinColor, Pcc32);
						pcolor2.x = pc_uv2[0]; pcolor2.y = pc_uv2[1];
						circle(color_image, pcolor2, 3, Scalar(255, 255, 0), -1); //第五个参数我设为-1，表明这是个实点。
					}
			}
				

			 /*for (int j = 0; j<contours[i].size(); j++)//此办法也可用的
			 {
			 int x = contours[i][j].x;
			 int y = contours[i][j].y;
			 }*/

		}//对应for

		float ratio;
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		ratio = width / (float)height;
		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//glEnable(GL_DEPTH_TEST);和 GL_DEPTH_BUFFER_BIT要连用才行，没有 GL_DEPTH_BUFFER_BIT就不要GL_DEPTH_BUFFER_BIT，会出问题
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		//		gluPerspective(40.0, ratio, 0.1, 20.0);
		//		glOrtho(-ratio, ratio, -1.f, 1.f, 3.f, -3.f);
//		glOrtho(0, 1.0f, 0, 1.0f, 3.f, -3.f);
		glOrtho(-200.0f*ratio, 200.0f*ratio, -200.0f, 200.0f, 3000.f, -3000.f);//以高度为核心，宽度乘以比例。
		//opengl的左下角为最小，右上角为最大;而realsense的三维坐标中，左上角为最小，右下角为最大。所以realsense得到的y坐标要关于x轴对称一下即取相反
//		glOrtho(0, 200.0f*ratio, -200.0f, 0.0f, 3000.f, -3000.f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix();
//		glTranslated(0.4, 0.45, 0.5);//根据数据的实际坐标调整要把原点移到哪里，原来的数据中心差不多在x=0.4单位,y=0.45单位,深度一般在0.6m作用
		glTranslatef(150.0f, -35.0f, 300.0f);
//		glScalef(2.0f,2.0f,2.0f);//放大比调整视角的效果还好，必须紧接glTranslatef指明放大的中心
		glRotated((GLdouble)yangle, 0.0, 1.0, 0.0);//下面所有的图会以上面的点的Y轴为中心旋转
		glRotated((GLdouble)xangle, 1.0, 0.0, 0.0);//下面所有的图会以上面的点的x轴为中心旋转
//		glTranslated(-0.4, -0.45, -0.5);//原点坐标再移回来，下面的绘图会以（0，0）为原点的，确保数据绘出位置跟产生处相同
										//听说glLoadIdentity();是恢复到原点，我试了，确实是，但所有的图不再能旋转，所以跟上句撤回意思还是不同，不能随便用。
		glTranslatef(-150.0f, 35.0f, -300.0f);
		glPushMatrix();//隔离
		glBegin(GL_POINTS);//GL_POINTS,GL_LINE_STRIP,GL_LINE_LOOP
						   //		glColor3f(0.75, 0.75, 0.75);
						   //		glColor3f(1.0, 0.0, 0.0);
		fseek(fp, 0L, SEEK_SET);//将指针设置至文件开头,没有这句图像会闪（原因解释：指针处于文件结尾，不再读数据了，导致幕后缓冲区始终是黑的）
		fseek(fp2, 0L, SEEK_SET);
		while ((!feof(fp)) && (!feof(fp2)))
		{
			
			fscanf(fp, "%c,%f, %f, %f", &pstr,&x, &y, &z);
			fscanf(fp2, "%c,%f, %f, %f", &cstr,&Rcolor, &Gcolor, &Bcolor);
			if (pstr=='M'&&cstr == 'M')
			{   
				counter = counter + 1;
				Mtranslatex = 1000*x;
				Mtranslatey = 1000*y;
				Mtranslatez = 1000*z;
				if (counter == 1)
				{
					x1 = Mtranslatex; y1 = Mtranslatey; z1 = Mtranslatez;
				}
				else
				{
					xo1 = Mtranslatex; yo1 = Mtranslatey; zo1 = Mtranslatez;
				}
//				printf("Mtranslatex=%f,Mtranslatey=%f, Mtranslatez=%f", Mtranslatex, Mtranslatey, Mtranslatez);
//				printf("%c",pstr);//这里是覆盖，只在左上角有字符输出，如果加了回车符\n，则会一行一行，占满全屏一直输出
			}
			if (pstr == 'N'&&cstr == 'N')
			{
				counterN = counterN + 1;
				Mtranslatex = 1000 * x;
				Mtranslatey = 1000 * y;
				Mtranslatez = 1000 * z;
				if (counterN == 1)
				{
					x2 = Mtranslatex; y2 = Mtranslatey; z2 = Mtranslatez;
				}
				else
				{
					xo2 = Mtranslatex; yo2 = Mtranslatey; zo2 = Mtranslatez;
				}
			}
			if (pstr == 'Q'&&cstr == 'Q')
			{
				counterQ = counterQ + 1;
				Mtranslatex = 1000 * x;
				Mtranslatey = 1000 * y;
				Mtranslatez = 1000 * z;
				if (counterQ == 1)
				{
					x3 = Mtranslatex; y3 = Mtranslatey; z3 = Mtranslatez;
				}
				else
				{
					xo3 = Mtranslatex; yo3 = Mtranslatey; zo3 = Mtranslatez;
				}
			}
			if (pstr == 'A'&&cstr == 'A')//角度不用转化为毫米
			{
//				printf("x=%f,y=%f,z=%f", x, y, z);
				Arotationx = 3.14*x/180.0; 
				Arotationy = -3.14*z/180.0;
				Arotationz = 3.14*y/180.0;
				if (counter == 1)
				{
					A0 = Arotationx;
					A1 = Arotationy;
					A2 = Arotationz;
				}
				//JY901旋转顺序Z-Y-X
			    /*gamma = Arotationx- A0;
				beta =  Arotationy- A1;
				alpha = Arotationz- A2; */
				
  			    g = Arotationx - A0;
				b = Arotationy - A1+7/180.0;//离90差32度
				a = Arotationz - A2;
//				printf("Arotationx=%f, Arotationy=%f,Arotationz=%f", Arotationx, Arotationy, Arotationz);
//				printf("a=%f,b=%f,g=%f", a, b, g);
//				printf("%c", pstr);//这里是覆盖，只在左上角有字符输出，如果加了回车符\n，则会一行一行，占满全屏一直输出
			}
			
			if (counter >1 && counterN>1 && counterQ>1)//当有第二帧时再执行
			{
				if (posecomputer_enable_flag)//main函数初始化中为true
				{
					pts1.push_back(Point3f(x1, y1, z1));
					pts2.push_back(Point3f(xo1, yo1, zo1));
					pts1.push_back(Point3f(x2, y2, z2));
					pts2.push_back(Point3f(xo2, yo2, zo2));
					pts1.push_back(Point3f(x3, y3, z3));
					pts2.push_back(Point3f(xo3, yo3, zo3));
					pose_estimation_3d3d(pts1, pts2, R, t);//这句的计算量很大，只用一次就够所以引入了posecomputer_enable_flag
					posecomputer_enable_flag = false;//往后不再计算,否则很慢
				}
			}
			if (pstr == 'D'&&cstr == 'D')
			{
				xout = 1000*x;
				yout = 1000*y;
				zout = 1000*z;
				if (counter > 1&& counterN>1&& counterQ>1)
				{
/*
					R[0][0] = cos(alpha)*cos(beta); R[0][1] = cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma); R[0][2] = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
					R[1][0] = sin(alpha)*cos(beta); R[1][1] = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma); R[1][2] = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
					R[2][0] = -sin(beta);          R[2][1] = cos(beta)*sin(gamma);                                   R[2][2] = cos(beta)*cos(gamma);
*//*
					R[0][0] = cos(b)*cos(g) + sin(a)*sin(b)*sin(g); R[0][1] = cos(g)*sin(a)*sin(b) - cos(b)*sin(g); R[0][2] = cos(a)*sin(b);
					R[1][0] = cos(a)*sin(g); R[1][1] = cos(a)*cos(g); R[1][2] = -sin(a);
					R[2][0] = cos(b)*sin(a)*sin(g) - cos(g)*sin(b); R[2][1] = sin(b)*sin(g) + cos(b)*cos(g)*sin(a); R[2][2] = cos(a)*cos(b);
*/
/*//平面绕y轴旋转					
					R[0][0] = cos(b); R[0][1] = sin(b);
					R[1][0] = -sin(b); R[1][1] = cos(b);					
					xout = x*R[0][0] + z*R[0][1];
					zout = x*R[1][0]+ z*R[1][1];
*/
/*//空间绕y轴旋转：旋转矩阵的逆矩阵
					R[0][0] = cos(b); R[0][1] =0; R[0][2] = sin(b);
					R[1][0] = 0; R[1][1] =1; R[1][2] = 0;
					R[2][0] = -sin(b); R[2][1] =0; R[2][2] = cos(b);
*/
/*//空间绕y轴旋转：只旋转不考虑平移
					xout = x*R[0][0] + y*R[0][1] + z*R[0][2];
					yout = x*R[1][0] + y*R[1][1] + z*R[1][2];
					zout = x*R[2][0] + y*R[2][1] + z*R[2][2];
*/

/*//空间绕y轴旋转：考虑旋转和平移的
					xout = (x- Mtranslatex)*R[0][0] + (y - Mtranslatey)*R[0][1] + (z - Mtranslatez)*R[0][2]+ M0;
					yout = (x - Mtranslatex)*R[1][0] + (y - Mtranslatey)*R[1][1] + (z - Mtranslatez)*R[1][2]+M1;
					zout = (x - Mtranslatex)*R[2][0] + (y - Mtranslatey)*R[2][1] + (z- Mtranslatez)*R[2][2]+M2;
					if(monitorflag)
					fprintf(fp3, "D,%f,%f,%f\n", xout, yout, zout);
*/
					/*
//Y-Z-X顺序旋转矩阵的逆矩阵：因JY901九轴模块是按Z-Y-X的顺序定义的，而这里是以摄像头坐标的定义为基准的，
					//九轴竖着贴在摄像头后面，x,y,z值不再跟着板子上标的走，而是以水平放置时的坐标系为固定坐标系
					//然后此时获得的九轴的Z值是摄像头的-Y,九轴的Y值是摄像头的Z，X相同
					R[0][0] = cos(b)*cos(g); R[0][1] = sin(g); R[0][2] = cos(g)*sin(b);
					R[1][0] = -sin(a)*sin(b) - cos(a)*cos(b)*sin(g); R[1][1] = cos(a)*cos(g); R[1][2] = cos(b)*sin(a) - cos(a)*sin(b)*sin(g);
					R[2][0] = cos(b)*sin(a)*sin(g) - cos(a)*sin(b); R[2][1] = -cos(g)*sin(a); R[2][2] = cos(a)*cos(b) + sin(a)*sin(b)*sin(g);

					//不放心，按Z-Y-X再试一下,效果竟然相同
//					R[0][0] = cos(b)*cos(g); R[0][1] = cos(b)*sin(g); R[0][2] = sin(b);
//					R[1][0] = -cos(a)*sin(g) - cos(g)*sin(a)*sin(b); R[1][1] = cos(a)*cos(g) - sin(a)*sin(b)*sin(g); R[1][2] = cos(b)*sin(a);
//					R[2][0] = sin(a)*sin(g) - cos(a)*cos(g)*sin(b); R[2][1] = -cos(g)*sin(a) - cos(a)*sin(b)*sin(g); R[2][2] = cos(a)*cos(b);
					
					xout = (1000*x - Mtranslatex)*R[0][0] + (1000*y - Mtranslatey)*R[0][1] + (1000*z - Mtranslatez)*R[0][2] + x1;
					yout = (1000*x - Mtranslatex)*R[1][0] + (1000*y - Mtranslatey)*R[1][1] + (1000*z - Mtranslatez)*R[1][2] + y1;
					zout = (1000*x - Mtranslatex)*R[2][0] + (1000*y - Mtranslatey)*R[2][1] + (1000*z - Mtranslatez)*R[2][2] + z1;
*/
/*
					R[0][0] = (x1*zo2 - x2*zo1 - x1*zo3 + x3*zo1 + x2*zo3 - x3*zo2) / (xo1*zo2 - xo2*zo1 - xo1*zo3 + xo3*zo1 + xo2*zo3 - xo3*zo2);
					R[0][1] = 0; 
					R[0][2] = -(x1*xo2 - x2*xo1 - x1*xo3 + x3*xo1 + x2*xo3 - x3*xo2) / (xo1*zo2 - xo2*zo1 - xo1*zo3 + xo3*zo1 + xo2*zo3 - xo3*zo2);
					
					R[1][0] = (y1*zo2*zo2 + y1*zo3*zo3 - y2*zo3*zo3 - y3*zo2*zo2 - y1*yo2*zo2 + y2*yo1*zo2 + y1*yo2*zo3 + y1*yo3*zo2 - y2*yo1*zo3 - y3*yo1*zo2 - y1*yo3*zo3 - y2*yo3*zo2 + y3*yo1*zo3 + y3*yo2*zo2 + y2*yo3*zo3 - y3*yo2*zo3 - y2*zo1*zo2 - yo1*z2*zo2 + yo2*z2*zo1 - 2 * y1*zo2*zo3 + y2*zo1*zo3 + y3*zo1*zo2 + yo1*z2*zo3 + yo1*z3*zo2 - yo2*z3*zo1 - yo3*z2*zo1 + y2*zo2*zo3 - y3*zo1*zo3 - yo1*z3*zo3 - yo2*z2*zo3 + yo3*z2*zo2 + yo3*z3*zo1 + y3*zo2*zo3 + yo2*z3*zo3 - yo3*z3*zo2) / (xo1*zo2*zo2 + xo1*zo3*zo3 - xo2*zo3*zo3 - xo3*zo2*zo2 - xo1*yo2*zo2 + xo2*yo2*zo1 + xo1*yo2*zo3 + xo1*yo3*zo2 - xo2*yo3*zo1 - xo3*yo2*zo1 - xo1*yo3*zo3 - xo2*yo2*zo3 + xo3*yo2*zo2 + xo3*yo3*zo1 + xo2*yo3*zo3 - xo3*yo3*zo2 - xo2*zo1*zo2 - 2 * xo1*zo2*zo3 + xo2*zo1*zo3 + xo3*zo1*zo2 + xo2*zo2*zo3 - xo3*zo1*zo3 + xo3*zo2*zo3);
					R[1][1] = (y2 - y3 - z2 + z3) / (yo2 - yo3 - zo2 + zo3);
					R[1][2] = (xo2*y1*yo2 - xo2*y2*yo1 - xo2*y1*yo3 + xo2*y3*yo1 - xo3*y1*yo2 + xo3*y2*yo1 + xo2*y2*yo3 - xo2*y3*yo2 + xo3*y1*yo3 - xo3*y3*yo1 - xo3*y2*yo3 + xo3*y3*yo2 + xo1*y2*zo2 - xo1*yo2*z2 - xo2*y1*zo2 + xo2*yo1*z2 - xo1*y2*zo3 - xo1*y3*zo2 + xo1*yo2*z3 + xo1*yo3*z2 + xo2*y1*zo3 - xo2*yo1*z3 + xo3*y1*zo2 - xo3*yo1*z2 + xo1*y3*zo3 - xo1*yo3*z3 + xo2*y3*zo2 - xo2*yo3*z2 - xo3*y1*zo3 - xo3*y2*zo2 + xo3*yo1*z3 + xo3*yo2*z2 - xo2*y3*zo3 + xo2*yo3*z3 + xo3*y2*zo3 - xo3*yo2*z3) / (xo1*zo2*zo2 + xo1*zo3*zo3 - xo2*zo3*zo3 - xo3*zo2*zo2 - xo1*yo2*zo2 + xo2*yo2*zo1 + xo1*yo2*zo3 + xo1*yo3*zo2 - xo2*yo3*zo1 - xo3*yo2*zo1 - xo1*yo3*zo3 - xo2*yo2*zo3 + xo3*yo2*zo2 + xo3*yo3*zo1 + xo2*yo3*zo3 - xo3*yo3*zo2 - xo2*zo1*zo2 - 2 * xo1*zo2*zo3 + xo2*zo1*zo3 + xo3*zo1*zo2 + xo2*zo2*zo3 - xo3*zo1*zo3 + xo3*zo2*zo3);
					
					R[2][0] = (z1*zo2 - z2*zo1 - z1*zo3 + z3*zo1 + z2*zo3 - z3*zo2) / (xo1*zo2 - xo2*zo1 - xo1*zo3 + xo3*zo1 + xo2*zo3 - xo3*zo2);
					R[2][1] = 0; 
					R[2][2] = (xo1*z2 - xo2*z1 - xo1*z3 + xo3*z1 + xo2*z3 - xo3*z2) / (xo1*zo2 - xo2*zo1 - xo1*zo3 + xo3*zo1 + xo2*zo3 - xo3*zo2);
*///这里的旋转矩阵是2帧到1帧的，即1帧到2帧（2帧为输出，1帧为输入，前面的代码都是以后来者为输入的）的逆矩阵，但是直接求是不行的，因它肯定是奇异矩阵，无法求逆矩阵，只能通过SVD分解才行
/*					float R[3][3] = { 0.0 };//旋转矩阵
					R[0][0] = 0.9535138403822582;
					R[0][1] = -0.03725076845750107;
					R[0][2] = 0.2990380184003431;

					R[1][0] = 0.06320891770961796;
					R[1][1] = 0.9949784347989045;
					R[1][2] = -0.07760507075636959;

					R[2][0] = -0.2946455309714655;
					R[2][1] = 0.09289937854715749;
					R[2][2] = 0.9510802892206818;
*/					
					xout = (1000 * x - xo1)*R.at<double>(0, 0) + (1000 * y - yo1)*R.at<double>(0, 1) + (1000 * z - zo1)*R.at<double>(0, 2) + x1;
					yout = (1000 * x - xo1)*R.at<double>(1, 0) + (1000 * y - yo1)*R.at<double>(1, 1) + (1000 * z - zo1)*R.at<double>(1, 2) + y1;
					zout = (1000 * x - xo1)*R.at<double>(2, 0) + (1000 * y - yo1)*R.at<double>(2, 1) + (1000 * z - zo1)*R.at<double>(2, 2) + z1;
/*
					xout = (1000 * x - (xo1+ xo2+ xo3)/3.0)*R[0][0] + (1000 * y - (yo1+yo2+ yo3)/3.0)*R[0][1] + (1000 * z - (zo1+zo2+ zo3)/3.0)*R[0][2] + (x1+x2+x3)/3.0;
					yout = (1000 * x - (xo1 + xo2 + xo3) / 3.0)*R[1][0] + (1000 * y - (yo1 + yo2 + yo3) / 3.0)*R[1][1] + (1000 * z - (zo1 + zo2 + zo3) / 3.0)*R[1][2] + (y1+y2+y3)/3.0;
					zout = (1000 * x - (xo1 + xo2 + xo3) / 3.0)*R[2][0] + (1000 * y - (yo1 + yo2 + yo3) / 3.0)*R[2][1] + (1000 * z - (zo1 + zo2 + zo3) / 3.0)*R[2][2] + (z1+z2+z3)/3.0;
*/
				}
				glColor3f(Rcolor, Gcolor, Bcolor);
//				glVertex3f(x, 1 - y, z);
//				glVertex3f(xout, 1-yout, zout);
				glVertex3f(xout, -yout, zout);//opengl的左下角为最小，右上角为最大;而realsense的三维坐标中，左上角为最小，右下角为最大。所以realsense得到的y坐标要关于x轴对称一下即取相反
			}
		}
		counter = 0; counterN = 0; counterQ = 0;
//		monitorflag = false;
		glEnd();
		glPopMatrix();
		/*		//平移到指定坐标点画圆，如果连续在不同点画圆，一定记得坐标每次平移后重新移回到原点，然后再移到新的指定坐标点画圆
		glColor3f(0, 1, 0);//绿
		glBindTexture(GL_TEXTURE_2D, texture[1]);//绑定纹理
		glPushMatrix();
		glTranslatef(point[3][0], point[3][1], point[3][2]);
		glTranslatef(0.0f, 0.0f, 1500.0f);
		glRotatef(-90, 1.0f, 0.0f, 0.0f);//为了贴图能正对我
		glRotatef(-90, 0.0f, 1.0f, 0.0f);//为了贴图能正对我
		//glutSolidSphere (5.5, 40, 50); //半径5.5mm
		GLUquadricObj *quadObj4 = gluNewQuadric();//创建一个二次曲面物体
		glBegin(GL_QUADS);
		gluQuadricTexture(quadObj4, GL_TRUE);     //启用该二次曲面的纹理
		gluSphere(quadObj4, 5.5, 20, 20); //画圆
		gluDeleteQuadric(quadObj4);
		glPopMatrix();*/
		glPushMatrix();//隔离
		float a[3] = { 0.0 }, b[3] = { 0.0 }, x1[3] = { 0.0 }, x2[3] = { 0.0 };
		float l = 1.0, m = 1.0, n = 1.0;
		float t = 0.05*1000;
		glPointSize(2.0);//通过改此值，可以改点的大小
		glBegin(GL_POINTS);
		for (int i = 0; i<10; i++)
		{

			if (outpoint[i][2] > 0)
			{

				glColor3f(1.0, 0.0, 0.0);    // Red
				glVertex3f(outpoint[i][0], -outpoint[i][1], outpoint[i][2]);
				//				glVertex3f(i*0.2, 0.0, 0.0);

			}
		}
		glEnd();
		glPointSize(1.0);//还原为默认大小的点，以免影像点云出图

		glBegin(GL_LINES);//线从小球中心穿过
		for (int i = 0; i<10; i++)
		{

			if (outpoint[i][2] > 0)
			{
				a[0] = outpoint[i][0];
				a[1] = -outpoint[i][1];
				a[2] = outpoint[i][2]+5.8;//5.8mm为反光小球半径，z方向加深5.8mm使得线从小球中心穿过
				for (int j = i + 1; j < 10; j++)
				{
					if (outpoint[j][2] > 0)
					{
						b[0] = outpoint[j][0];
						b[1] = -outpoint[j][1];
						b[2] = outpoint[j][2]+5.8;//5.8mm为反光小球半径，z方向加深5.8mm使得线从小球中心穿过
						float distance = sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
							std::cout << "小球间距离" << distance <<" \r";
						if (distance > 0.05*1000 && distance < 0.5*1000)
						{
							l = b[0] - a[0];
							m = b[1] - a[1];
							n = b[2] - a[2];
							//上端点
							x1[0] = b[0] + t*l / sqrt(l*l + m*m + n*n);
							x1[1] = b[1] + t*m / sqrt(l*l + m*m + n*n);
							x1[2] = b[2] + t*n / sqrt(l*l + m*m + n*n);
							//下端点
							x2[0] = a[0] - t*l / sqrt(l*l + m*m + n*n);
							x2[1] = a[1] - t*m / sqrt(l*l + m*m + n*n);
							x2[2] = a[2] - t*n / sqrt(l*l + m*m + n*n);
							//两球之间线段
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(a[0], a[1], a[2]);
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(b[0], b[1], b[2]);
							//下球与下端间线段
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(a[0], a[1], a[2]);
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(x2[0], x2[1], x2[2]);

							/*//上下端线段
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(x1[0], x1[1], x1[2]);
							glColor3f(1.0, 0.0, 0.0);    // Red
							glVertex3f(x2[0], x2[1], x2[2]);*/

						}
					}
				}
			}
		}
		glEnd();
		glPopMatrix();
		glPopMatrix();
		glfwSwapBuffers(window);
		//		glfwPollEvents();
		flag = false;//每按一次p按键小球的坐标点只在开始输出一次
					 //		imshow(left_win, result);
		imshow("contours", contoursImage);
		imshow("红外1图像", dMat_left);
		imshow("彩色图像", color_image);
		//		setMouseCallback("彩色图像", on_mouse, &color_image);
		char c = waitKey(1);
		int si = 0;
		if (c == 'p')
		{
			sprintf_s(LName, "Linfra%d.png", si);
			imwrite(LName, result);
			flag = true;
			si++;
		}
		else if (c == 'q' || c == 27)
		{
			flag = false;
			glfwTerminate();
			break;
		}
		std::cout << "\r";
	}//对应while
	monitorflag = false;
	fclose(fp3);
	fclose(fp);
	fclose(fp2);
	glfwTerminate();
	destroyAllWindows();
	return 0;
}
void pose_estimation_3d3d(
	const vector<Point3f>& pts1,
	const vector<Point3f>& pts2,
	Mat& R, Mat& t
)
{
	Point3f p1, p2;     // center of mass
	int N = pts1.size();
	for (int i = 0; i<N; i++)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 = Point3f(Vec3f(p1) / N);
	p2 = Point3f(Vec3f(p2) / N);
	vector<Point3f>     q1(N), q2(N); // remove the center
	for (int i = 0; i<N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i<N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
//	cout << "W=" << W << endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

	if (U.determinant() * V.determinant() < 0)
	{
		for (int x = 0; x < 3; ++x)
		{
			U(x, 2) *= -1;
		}
	}

//	cout << "U=" << U << endl;
//	cout << "V=" << V << endl;

	Eigen::Matrix3d R_ = U* (V.transpose());
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	// convert to cv::Mat
	R = (Mat_<double>(3, 3) <<
		R_(0, 0), R_(0, 1), R_(0, 2),
		R_(1, 0), R_(1, 1), R_(1, 2),
		R_(2, 0), R_(2, 1), R_(2, 2)
		);
	t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}