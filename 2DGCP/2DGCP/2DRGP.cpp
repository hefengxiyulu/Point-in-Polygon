#include"pch.h"
#include "2DRGP.h"
#include <iostream>
using namespace std;
//构造函数
Line2D::Line2D(Point2D* x1, Point2D* x2)
{
	p1 =x1;
	p2 = x2;
}
//析构函数
Line2D::~Line2D()
{
	if (p1 != NULL)
	{
		free(p1);
		p1 = NULL;
	}
	if (p2 != NULL)
	{
		free(p2);
		p2 = NULL;
	}
}

//计算斜率
FLTYPE Line2D::CalculateSlope()
{
	FLTYPE slope;
	Point2D  p21;
	p21.x = p2->x - p1->x;
	p21.y = p2->y - p1->y;
	if (Equal(p21.x, 0) == 1)
	{
		slope = PI_HALF;
	}
	else
	{
		slope = p21.y / p21.x;
	}
	return slope;
}

//To determine whether two numbers are equal    EPSILON is Minimum positive number
int Line2D::Equal(FLTYPE  a, FLTYPE  b)
{
	FLTYPE  temp = abs(a - b);
	if ( temp <= EPSILON) 
		return 1; 
	else 
		return 0;
}
void RGP_Statistics::outputStatistcs()
{
	cout << "点落在平行X轴的网格线上的个数：" << onthe_x_GridEdge << endl;
	cout << "点落在平行Y轴的网格线上的个数：" << onthe_y_GridEdge << endl;
	cout << "通过坐标对比剔除的测试点个数：" << exclude_by_CoorContast << endl;
	cout << "叉积计算次数：" << cross_product_Count << endl;
	cout << "测试点在多边形外且无边的Cell中的个数：" << outsideCellPointCount << endl;
	cout << "测试点在多边形内且无边的Cell中的个数：" << insideCellPointCount << endl;
	cout << "测试点在含边的Cell中的个数:" << inEdgeCellPointCount << endl;
	cout << "测试点落在多边形边上的个数:" << ontheEdgeCount << endl;
	cout << "测试线与多边形边重合的次数:" << superpositionCount << endl;
	cout << "点在多边形边框外部的个数:" << outoftheBoundingbox << endl;
	cout << "点落在无边的网格中的个数:" << noEdgeCellPointCount << endl;
	cout << "测试阶段坐标对比次数:" << compareCount << endl;
	cout << "平行X轴查询次数:" << searchXCount << endl;
	cout << "平行Y轴查询次数:" << searchYCount << endl;
	cout << "平行Y轴查询次数:" << searchYCount << endl;
	cout << "加法次数:" << addCount << endl;
	cout << "乘法次数:" << multiplicationCount << endl;
}