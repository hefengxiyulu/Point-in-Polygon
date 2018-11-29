#pragma once

#ifndef GEOMETRY_H
#define GEOMETRY_H 1

#include <cstring>
#include <stdlib.h>


#define FLTYPE double
//#define pi  3.1415926

struct RecursiveGrid2D;

struct Point2D {
	union {
		struct { FLTYPE x, y; };
		FLTYPE coord[2];
	};
	Point2D(FLTYPE xx = 0, FLTYPE yy = 0)
	{
		x = xx; y = yy;
	}
};

struct Edge2D {
	//FLTYPE a,b,c;   //边所在直线方程系数
	union {
		struct { int startIndex, endIndex; };     //边的顶点索引
		int vertexIndex[2];
	};
};


struct GCPPolygon2D {

	struct Point2D * vertexTable;		//顶点列表
	int vertexCount;					//顶点数
	struct Edge2D * edgeTable;			//边列表
	int edgeCount;						//边数
	int * ringTable;					//ring list
	int ringCount;						//ring number + 1

	struct Point2D boundingBox[2];       //包围盒
	FLTYPE dx, dy;  //平均每条边在X,Y方向上的平均长度（用于计算最佳分辨率）

	GCPPolygon2D() { memset(this, 0, sizeof(*this)); }
	~GCPPolygon2D() {
		if (vertexTable != NULL)
		{
			delete[] vertexTable;
			vertexTable = NULL;
		}
		if (edgeTable != NULL)
		{
			delete[] edgeTable;
			edgeTable = NULL;
		}
		if (ringTable != NULL)
		{
			delete[] ringTable;
			ringTable = NULL;
		}
		vertexCount = 0;
		edgeCount = 0;
		ringCount = 0;
	}
};

struct EdgeRef2D {
	Edge2D * e;
	struct EdgeRef2D * next;
};

//图学学报 start
//存储多边形边片段，记录片段两端的坐标
struct RGPEdgeRef2D {
	Point2D *start_e;
	Point2D *end_e;
	FLTYPE k;//斜率
	FLTYPE B;//截距
	struct RGPEdgeRef2D *next;
};
//end
//used in group method, record the information of the point which locate in the contain edge cell
struct testPointInfo {
	int testPiontIndex;
	float x;
	float y;
};

struct GridCell2D {
	EdgeRef2D *edgeRef;    //包含的边指针列表
	int flag;       //单元的属性

	int edgeCount;       //包含的边数
	struct Point2D middle; //记录单元中心点坐标

	//start 图学学报RGP
	struct RGPEdgeRef2D *RgpEdgeRef;  //存储网格单元中包含的边片段
	int cell_InofOut;//记录网格单元是否处于多变形内、外，或含边
	FLTYPE cell_middle_y;//记录网格单元中的中点Y轴坐标，以减少测试阶段求中点坐标的计算量，测试时直接调用即可。
	//struct Point2D cell_middle; //记录单元中心点坐标

	//group method
	struct testPointInfo testPointIndex[100000];//预先申请10000个空间，用于存储测试点的索引，以及测试点在子网格中的分布情况
	int TestPointIndex[100000]; //用于存储测试点的索引，仅使用在group方法中
	int testPointCount=0;  //用于记录当前单元网格中的测试点的个数，仅在group方法中使用

//end

	GridCell2D() { memset(this, 0, sizeof(*this)); }
	~GridCell2D() {}
};


struct Grid2D {
	int resolution[2];   //分辨率
	int cellCount;       //单元总数

	Point2D boundingBox[2];  //网格包围盒
	FLTYPE gridSize[2];    //网格大小
	FLTYPE cellSize[2];     //网格单元大小
	struct GridCell2D * cell;  //网格单元列表

	int expectRefCount;     //估算的指针总数
	//int realRefCount;           //实际指针总数
	struct EdgeRef2D * edgeRef;   //作为整体分配的边指针链表
	//start 图学学报RGP
	struct RGPEdgeRef2D *RgpEdgeRef;  //存储网格单元中包含的边片段
	//struct RgpPointInSegment *RgpPoint;  //存储各个网格线片段上的点
	//end

	//struct RGPCell2D *RgpPoint;  //图学学报 存储网格交点坐标
	//struct RGPIntersectPointX *RgpIntersectPointX;   // 图学学报，储存多边形边与网格线交点 平行X轴
	//struct RGPIntersectPointY *RgpIntersectPointY;   //图学学报，储存多边形边与网格线交点  平行Y轴

	Grid2D(int x = 1, int y = 1)
	{
		resolution[0] = x;
		resolution[1] = y;
		cell = NULL;
	}
	~Grid2D()
	{
		//if (edgeRef != NULL)   // RGP方法需要注释
		//{
		//	free(edgeRef);
		//	edgeRef = NULL;
		//}
		if (RgpEdgeRef != NULL)
		{
			free(RgpEdgeRef);
			RgpEdgeRef = NULL;
		}
		if (cell != NULL)
		{
			delete[] cell;
			cell = NULL;
		}
	}

	//compute the index of the cell that index a point fell in
	int locatePoint(FLTYPE x, FLTYPE y, int & x_index, int & y_index)
	{
		x_index = (int)((x - boundingBox[0].x) / cellSize[0]+0.000001);     //+0.000001用以处理点与网格线重合
		y_index = (int)((y - boundingBox[0].y) / cellSize[1]+ 0.000001);    //这样是网格从下往上存储
		return (y_index * resolution[0] + x_index);
	}
	float locatePoint(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0] + 0.000001);     //+0.000001用以处理点与网格线重合
		y_index = ((y - boundingBox[0].y) / cellSize[1] + 0.000001);    //这样是网格从下往上存储
		return ((int)y_index * resolution[0] + (int)x_index);
	}

	//compute the float index of a cell that a point fell in
	void locatePoint_f(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0]);
		y_index = ((y - boundingBox[0].y) / cellSize[1]);    //这样是网格从下往上存储
	}
};



//层次网格结构单元
struct RecursiveGridCell2D {
	union {
		EdgeRef2D * edgeRef;				//边指针
		RecursiveGrid2D * subgrid;		//子网格
	};
	int edgeCount;	//边指针数
	int flag;	//单元属性：内/外，奇异/非奇异，中间单元/叶子单元

	RecursiveGridCell2D() { memset(this, 0, sizeof(*this)); }
	~RecursiveGridCell2D() {}
};



//层次网格结构
struct RecursiveGrid2D {

	int resolution[2];   //分辨率
	Point2D boundingBox[2];  //网格包围盒
	FLTYPE gridSize[2];    //网格大小
	FLTYPE cellSize[2];     //网格单元大小
	int expectRefCount;     //估算的指针总数
	struct EdgeRef2D * edgeRef;   //作为整体分配的边指针链表
	struct RecursiveGridCell2D * cell;  //网格单元列表

	RecursiveGrid2D() { memset(this, 0, sizeof(*this)); }
	~RecursiveGrid2D()
	{
		clear(0);
	}

	void clear(int depth);

	void subdivision(GCPPolygon2D * ply, int depth, int & total_cell, int & total_pt);
	int addEdgeRef(GCPPolygon2D * ply, EdgeRef2D * ef);
	void setCellProp_out(GCPPolygon2D * ply);
	void setCellProp_center(GCPPolygon2D * ply, int center_prop);
	bool isIntersect(Point2D * p0, Point2D * p1, Point2D * p2, Point2D *p3);
	void clipSegment(Point2D * box, Point2D & start, Point2D & end);

	//compute the index of the cell that index a point fell in
	int locatePoint(FLTYPE x, FLTYPE y, int & x_index, int & y_index)
	{
		x_index = (int)((x - boundingBox[0].x) / cellSize[0]);
		y_index = (int)((y - boundingBox[0].y) / cellSize[1]);    //这样是网格从下往上存储
		return (y_index * resolution[0] + x_index);
	}

	//compute the float index of a cell that a point fell in
	void locatePoint_f(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0]);
		y_index = ((y - boundingBox[0].y) / cellSize[1]);    //这样是网格从下往上存储
	}
};


//quad-tree

//#define QUADTREE_MAX_DEPTH 8
//#define QUADTREE_MIN_EDGECOUNT 10
#define QUADTREE_INTERNAL_NODE 4
#define QUADTREE_LEAF_NODE 8

struct QuadTreeNode {
	struct QuadTreeNode * subnode[4];
	EdgeRef2D * edgeRef;	//leaf node
	int flag;
	Point2D boundingbox[2];

	QuadTreeNode() { memset(this, 0, sizeof(*this)); }
	~QuadTreeNode()
	{
		for (int i = 0; i < 4; i++)
		{
			if (subnode[i] != NULL)
				delete subnode[i];
		}
	}

	//append new edge reference to leaf node
	int append(Edge2D * edge);
	//eliminate a certain edge reference from leaf node
	int eliminate(Edge2D * edge);
};

struct QuadTree {
	QuadTreeNode * root;

	QuadTree() { memset(this, 0, sizeof(*this)); }
	~QuadTree() { clear(root); }

	void clear(QuadTreeNode * r);
};



//3-Dimension data
struct GCPPoint3D {
	union {
		struct { FLTYPE x, y, z; };
		FLTYPE coord[3];
	};
	GCPPoint3D(FLTYPE xx = 0, FLTYPE yy = 0, FLTYPE zz = 0)
	{
		x = xx; y = yy; z = zz;
	}
};

struct Edge3D {
	//FLTYPE a,b,c;   //边所在直线方程系数
	union {
		struct { int startIndex, endIndex; };     //边的顶点索引
		int vertexIndex[2];
	};
};

struct Segment2D {
	Point2D v0, v1;
};

#define MAX_DA_SIZE 1024

//动态数组
struct DynamicPointArray {
	Segment2D * segments;
	int maxSize;

	DynamicPointArray()
	{
		maxSize = MAX_DA_SIZE;
		segments = new Segment2D[maxSize];
	}

	~DynamicPointArray()
	{
		delete[] segments;
	}

	Segment2D & operator [](int i);
	/*
	{
		if(i>=maxSize)
		{
			int old_maxSize = maxSize;
			while(i >= maxSize)
				maxSize <<= 1;
			Segment2D * new_segments = new Segment2D[maxSize];
			memcpy(new_segments, segments, sizeof(Segment2D) * old_maxSize);
			delete [] segments;
			segments = new_segments;
		}
		return segments[i];
	}*/
};


#endif
