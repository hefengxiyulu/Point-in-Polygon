#pragma once
#include "geometry.h"
#include <stdlib.h>
#include  <cstdlib>
#include <string>
#include "EdgeList.h"

#define FLTYPE double
#define EPSILON 0.0000000001
#define PI_HALF 1.570796327
#define max(a,b)           (((a) > (b)) ? (a) : (b))

struct RGPCell2D
{
	int flag;//网格点的属性RG
	struct  Point2D gridPoint; //记录网格单元的交点坐标

	RGPCell2D() { memset(this, 0, sizeof(*this)); }
	~RGPCell2D() {}
};

struct RGPIntersectPointX   //存储多边形边交于  平行X轴的的网格线  的交点坐标   （x坐标值，平行X轴，则Y值固定）
{
	FLTYPE x_coord;          //记录x坐标值
	int Cell_coord[3];             //为一维索引 即从左至右，从上至下，的第多少个    [ 0]：横坐标，[1]：纵坐标，[2]：点的个数
	RGPIntersectPointX() { memset(this, 0, sizeof(*this)); }
	~RGPIntersectPointX() {}
};

struct RGPIntersectPointY
{
	FLTYPE y_coord;          //记录y坐标值
	int Cell_coord[3];             //为一维索引 即从左至右，从上至下，的第多少个

	RGPIntersectPointY() { memset(this, 0, sizeof(*this)); }
	~RGPIntersectPointY() {}
};
//记录多边形边与网格线的交点，存储在每个网格线片段中
struct RgpPointInSegment
{
	FLTYPE xory_coord[10];
	int pointCount;

	//FLTYPE xory_coord;          //记录x或y坐标值
	//int Cell_coord[3];             //为一维索引 即从左至右，从上至下，的第多少个
	RgpPointInSegment(){ memset(this, 0, sizeof(*this)); }
	~RgpPointInSegment() {}
};
//Rgp 网格中边片段，记录网格线片段中存储的点
struct RgpGridEdgeSegment
{
	RgpPointInSegment *point_in_segment; //交点的相关信息
	int pointCount; //计数网格线片段上有多少点
};
struct Line2D
{
	//线段的两个坐标
	Point2D *p1;
	Point2D *p2;

	Line2D(Point2D* x1, Point2D* x2);
	~Line2D();

	FLTYPE CalculateSlope();   //计算斜率
	int Equal(FLTYPE a, FLTYPE b);       //
};

//存储边的片段，图学学报，RGP，将多边形边分割成小片段，进行存储，
//每个网格单元只包含边处于该网格单元中的片段
//struct EdgeSegment2D
//{
//	union {
//		struct { FLTYPE startPoint, endPoint; };
//		FLTYPE vertexPoint[2];
//	};
//};

//移到geometry.h
//struct RGPEdgeRef2D {
//	Point2D *start_e;
//	Point2D *end_e;
//	struct RGPEdgeRef2D *next;
//};
//////////////////////////////////////////////////////////////////
//vc6.0迁移
struct MySortOfPoint
{
	double distance1;
	Point2D a;//点坐标
	Point2D b;//小方格坐标
	int singularEdge;//是否是奇异边
};
//MySortOfPoint *mySP;

struct MSPCmp
{
	bool operator()(const MySortOfPoint& a, const MySortOfPoint& b)
	{
		return a.distance1 < b.distance1;
	}
};
struct IntersectionPoint
{
	double x;
	int mark;
	int singularpoint;
};
static bool cmp2(const double &a, const double &b)
{
	return a < b;
}
//统计相关数据
struct RGP_Statistics
{
	//点落在平行X轴的网格线上
	int onthe_x_GridEdge;
	//点落在平行Y轴的网格线上
	int onthe_y_GridEdge;
	//通过坐标对比剔除的测试点个数
	int exclude_by_CoorContast;
	//叉积计算次数
	int cross_product_Count;
	//测试点在 多边形外且无边的Cell中
	int outsideCellPointCount;
	////测试点在 多边形内且无边的Cell中
	int insideCellPointCount;
	//测试点在含边的cell 中的个数
	int inEdgeCellPointCount;
	//测试点落在多边形边上
	int ontheEdgeCount;
	//测试线与多边形边重合的次数
	int superpositionCount;
	//点在多边形边框外部的个数
	int outoftheBoundingbox;
	// 点落在无边的网格中
	int noEdgeCellPointCount;
	//测试时，对比次数
	int compareCount;
	//平行X轴查询次数
	int searchXCount;
	//平行Y轴查询次数
	int searchYCount;
	//加法次数
	int addCount;
	//乘法次数
	int multiplicationCount;

	RGP_Statistics() { clear(); }
	~RGP_Statistics() {}
	void clear() { memset(this, 0, sizeof(*this)); };
	void outputStatistcs();
};