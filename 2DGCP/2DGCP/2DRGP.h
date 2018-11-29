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
	int flag;//����������RG
	struct  Point2D gridPoint; //��¼����Ԫ�Ľ�������

	RGPCell2D() { memset(this, 0, sizeof(*this)); }
	~RGPCell2D() {}
};

struct RGPIntersectPointX   //�洢����α߽���  ƽ��X��ĵ�������  �Ľ�������   ��x����ֵ��ƽ��X�ᣬ��Yֵ�̶���
{
	FLTYPE x_coord;          //��¼x����ֵ
	int Cell_coord[3];             //Ϊһά���� ���������ң��������£��ĵڶ��ٸ�    [ 0]�������꣬[1]�������꣬[2]����ĸ���
	RGPIntersectPointX() { memset(this, 0, sizeof(*this)); }
	~RGPIntersectPointX() {}
};

struct RGPIntersectPointY
{
	FLTYPE y_coord;          //��¼y����ֵ
	int Cell_coord[3];             //Ϊһά���� ���������ң��������£��ĵڶ��ٸ�

	RGPIntersectPointY() { memset(this, 0, sizeof(*this)); }
	~RGPIntersectPointY() {}
};
//��¼����α��������ߵĽ��㣬�洢��ÿ��������Ƭ����
struct RgpPointInSegment
{
	FLTYPE xory_coord[10];
	int pointCount;

	//FLTYPE xory_coord;          //��¼x��y����ֵ
	//int Cell_coord[3];             //Ϊһά���� ���������ң��������£��ĵڶ��ٸ�
	RgpPointInSegment(){ memset(this, 0, sizeof(*this)); }
	~RgpPointInSegment() {}
};
//Rgp �����б�Ƭ�Σ���¼������Ƭ���д洢�ĵ�
struct RgpGridEdgeSegment
{
	RgpPointInSegment *point_in_segment; //����������Ϣ
	int pointCount; //����������Ƭ�����ж��ٵ�
};
struct Line2D
{
	//�߶ε���������
	Point2D *p1;
	Point2D *p2;

	Line2D(Point2D* x1, Point2D* x2);
	~Line2D();

	FLTYPE CalculateSlope();   //����б��
	int Equal(FLTYPE a, FLTYPE b);       //
};

//�洢�ߵ�Ƭ�Σ�ͼѧѧ����RGP��������α߷ָ��СƬ�Σ����д洢��
//ÿ������Ԫֻ�����ߴ��ڸ�����Ԫ�е�Ƭ��
//struct EdgeSegment2D
//{
//	union {
//		struct { FLTYPE startPoint, endPoint; };
//		FLTYPE vertexPoint[2];
//	};
//};

//�Ƶ�geometry.h
//struct RGPEdgeRef2D {
//	Point2D *start_e;
//	Point2D *end_e;
//	struct RGPEdgeRef2D *next;
//};
//////////////////////////////////////////////////////////////////
//vc6.0Ǩ��
struct MySortOfPoint
{
	double distance1;
	Point2D a;//������
	Point2D b;//С��������
	int singularEdge;//�Ƿ��������
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
//ͳ���������
struct RGP_Statistics
{
	//������ƽ��X�����������
	int onthe_x_GridEdge;
	//������ƽ��Y�����������
	int onthe_y_GridEdge;
	//ͨ������Ա��޳��Ĳ��Ե����
	int exclude_by_CoorContast;
	//����������
	int cross_product_Count;
	//���Ե��� ����������ޱߵ�Cell��
	int outsideCellPointCount;
	////���Ե��� ����������ޱߵ�Cell��
	int insideCellPointCount;
	//���Ե��ں��ߵ�cell �еĸ���
	int inEdgeCellPointCount;
	//���Ե����ڶ���α���
	int ontheEdgeCount;
	//�����������α��غϵĴ���
	int superpositionCount;
	//���ڶ���α߿��ⲿ�ĸ���
	int outoftheBoundingbox;
	// �������ޱߵ�������
	int noEdgeCellPointCount;
	//����ʱ���Աȴ���
	int compareCount;
	//ƽ��X���ѯ����
	int searchXCount;
	//ƽ��Y���ѯ����
	int searchYCount;
	//�ӷ�����
	int addCount;
	//�˷�����
	int multiplicationCount;

	RGP_Statistics() { clear(); }
	~RGP_Statistics() {}
	void clear() { memset(this, 0, sizeof(*this)); };
	void outputStatistcs();
};