// 2DGCP.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include <time.h>
#include "gridpip.h"

int main()
{
	GridPIP2D gcp;
	SmallTimer timer;

	gcp.readData("pol10.obj", 0);//读取数据，并将边存入相关数组，并记录各个边的起点和终点
	gcp.usedMethod = USED_METHOD_RGP;
	gcp.initData();//计算包围盒
	gcp.configData();//配置相关参数
	//preprocess
	timer.start();
	switch (gcp.usedMethod)
	{
	case USED_METHOD_GCP:
		gcp.createGrid();//GCP method
		break;
	case USED_METHOD_RGP:
		gcp.RgpcreateGrid();//RGP method
		break;
	case USED_METHOD_OGP:
		gcp.OgpcreateGrid();  //OGP method
		break;
	case USED_METHOD_G_RGP:
		gcp.G_RgpcreateGrid();//RGP method
		break;
	case USED_METHOD_G_OGP:
		gcp.OgpcreateGrid();  //OGP method
		break;
	default:
		cout << "Input Error, please set  correct parameters!" << endl;
		break;
	}
	timer.end();
	printf("Preprocess time %f\n", timer.time);

	int xcount = 1000;
	int ycount = 1000;
	gcp.testedPointCount = xcount * ycount;
	gcp.generateTestedPoint(0, xcount, ycount); //固定间隔测试点
	//gcp.generateTestedPoint(1, xcount, ycount); //随机测试点

	//point-in-polygon test
	timer.start();
	switch (gcp.usedMethod)
	{
	case USED_METHOD_GCP:
		gcp.PIP_robust();//GCP
		break;
	case USED_METHOD_RGP:
		gcp.RgpCheckPoint_Segment();  //RGP  网格线片段查找
		//gcp.RgpChekPoint_Segment_noStat();
		break;
	case USED_METHOD_OGP:
		gcp.OgpCheckPoint_Segment();
		//gcp.OgpCheckPoint_Segment_noStat();
		break;
	case USED_METHOD_G_RGP:
		gcp.G_RgpChekPoint_Segment_noStat();
		break;
	case USED_METHOD_G_OGP:
		gcp.G_OgpCheckPoint_Segment_noStat();
		break;
	default:
		cout << "Input Error, please set  correct parameters!" << endl;
		break;
	}
	timer.end();
	printf("PIP time %f\n", timer.time);

	gcp.Rgpstat.outputStatistcs();
	cout << "网格中心点奇异的情况次数:" << gcp.stat.suspectTPCount << endl;
	cout << gcp.stat.realInterCount << endl;
	//export result to OBJ file
	gcp.export2OBJ("gcp2dresult.obj", CELL_IN);

	return 0;
}
