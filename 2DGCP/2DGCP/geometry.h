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
	//FLTYPE a,b,c;   //������ֱ�߷���ϵ��
	union {
		struct { int startIndex, endIndex; };     //�ߵĶ�������
		int vertexIndex[2];
	};
};


struct GCPPolygon2D {

	struct Point2D * vertexTable;		//�����б�
	int vertexCount;					//������
	struct Edge2D * edgeTable;			//���б�
	int edgeCount;						//����
	int * ringTable;					//ring list
	int ringCount;						//ring number + 1

	struct Point2D boundingBox[2];       //��Χ��
	FLTYPE dx, dy;  //ƽ��ÿ������X,Y�����ϵ�ƽ�����ȣ����ڼ�����ѷֱ��ʣ�

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

//ͼѧѧ�� start
//�洢����α�Ƭ�Σ���¼Ƭ�����˵�����
struct RGPEdgeRef2D {
	Point2D *start_e;
	Point2D *end_e;
	FLTYPE k;//б��
	FLTYPE B;//�ؾ�
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
	EdgeRef2D *edgeRef;    //�����ı�ָ���б�
	int flag;       //��Ԫ������

	int edgeCount;       //�����ı���
	struct Point2D middle; //��¼��Ԫ���ĵ�����

	//start ͼѧѧ��RGP
	struct RGPEdgeRef2D *RgpEdgeRef;  //�洢����Ԫ�а����ı�Ƭ��
	int cell_InofOut;//��¼����Ԫ�Ƿ��ڶ�����ڡ��⣬�򺬱�
	FLTYPE cell_middle_y;//��¼����Ԫ�е��е�Y�����꣬�Լ��ٲ��Խ׶����е�����ļ�����������ʱֱ�ӵ��ü��ɡ�
	//struct Point2D cell_middle; //��¼��Ԫ���ĵ�����

	//group method
	struct testPointInfo testPointIndex[100000];//Ԥ������10000���ռ䣬���ڴ洢���Ե���������Լ����Ե����������еķֲ����
	int TestPointIndex[100000]; //���ڴ洢���Ե����������ʹ����group������
	int testPointCount=0;  //���ڼ�¼��ǰ��Ԫ�����еĲ��Ե�ĸ���������group������ʹ��

//end

	GridCell2D() { memset(this, 0, sizeof(*this)); }
	~GridCell2D() {}
};


struct Grid2D {
	int resolution[2];   //�ֱ���
	int cellCount;       //��Ԫ����

	Point2D boundingBox[2];  //�����Χ��
	FLTYPE gridSize[2];    //�����С
	FLTYPE cellSize[2];     //����Ԫ��С
	struct GridCell2D * cell;  //����Ԫ�б�

	int expectRefCount;     //�����ָ������
	//int realRefCount;           //ʵ��ָ������
	struct EdgeRef2D * edgeRef;   //��Ϊ�������ı�ָ������
	//start ͼѧѧ��RGP
	struct RGPEdgeRef2D *RgpEdgeRef;  //�洢����Ԫ�а����ı�Ƭ��
	//struct RgpPointInSegment *RgpPoint;  //�洢����������Ƭ���ϵĵ�
	//end

	//struct RGPCell2D *RgpPoint;  //ͼѧѧ�� �洢���񽻵�����
	//struct RGPIntersectPointX *RgpIntersectPointX;   // ͼѧѧ�����������α��������߽��� ƽ��X��
	//struct RGPIntersectPointY *RgpIntersectPointY;   //ͼѧѧ�����������α��������߽���  ƽ��Y��

	Grid2D(int x = 1, int y = 1)
	{
		resolution[0] = x;
		resolution[1] = y;
		cell = NULL;
	}
	~Grid2D()
	{
		//if (edgeRef != NULL)   // RGP������Ҫע��
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
		x_index = (int)((x - boundingBox[0].x) / cellSize[0]+0.000001);     //+0.000001���Դ�������������غ�
		y_index = (int)((y - boundingBox[0].y) / cellSize[1]+ 0.000001);    //����������������ϴ洢
		return (y_index * resolution[0] + x_index);
	}
	float locatePoint(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0] + 0.000001);     //+0.000001���Դ�������������غ�
		y_index = ((y - boundingBox[0].y) / cellSize[1] + 0.000001);    //����������������ϴ洢
		return ((int)y_index * resolution[0] + (int)x_index);
	}

	//compute the float index of a cell that a point fell in
	void locatePoint_f(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0]);
		y_index = ((y - boundingBox[0].y) / cellSize[1]);    //����������������ϴ洢
	}
};



//�������ṹ��Ԫ
struct RecursiveGridCell2D {
	union {
		EdgeRef2D * edgeRef;				//��ָ��
		RecursiveGrid2D * subgrid;		//������
	};
	int edgeCount;	//��ָ����
	int flag;	//��Ԫ���ԣ���/�⣬����/�����죬�м䵥Ԫ/Ҷ�ӵ�Ԫ

	RecursiveGridCell2D() { memset(this, 0, sizeof(*this)); }
	~RecursiveGridCell2D() {}
};



//�������ṹ
struct RecursiveGrid2D {

	int resolution[2];   //�ֱ���
	Point2D boundingBox[2];  //�����Χ��
	FLTYPE gridSize[2];    //�����С
	FLTYPE cellSize[2];     //����Ԫ��С
	int expectRefCount;     //�����ָ������
	struct EdgeRef2D * edgeRef;   //��Ϊ�������ı�ָ������
	struct RecursiveGridCell2D * cell;  //����Ԫ�б�

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
		y_index = (int)((y - boundingBox[0].y) / cellSize[1]);    //����������������ϴ洢
		return (y_index * resolution[0] + x_index);
	}

	//compute the float index of a cell that a point fell in
	void locatePoint_f(FLTYPE x, FLTYPE y, float & x_index, float & y_index)
	{
		x_index = ((x - boundingBox[0].x) / cellSize[0]);
		y_index = ((y - boundingBox[0].y) / cellSize[1]);    //����������������ϴ洢
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
	//FLTYPE a,b,c;   //������ֱ�߷���ϵ��
	union {
		struct { int startIndex, endIndex; };     //�ߵĶ�������
		int vertexIndex[2];
	};
};

struct Segment2D {
	Point2D v0, v1;
};

#define MAX_DA_SIZE 1024

//��̬����
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
