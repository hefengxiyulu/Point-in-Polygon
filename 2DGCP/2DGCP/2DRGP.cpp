#include"pch.h"
#include "2DRGP.h"
#include <iostream>
using namespace std;
//���캯��
Line2D::Line2D(Point2D* x1, Point2D* x2)
{
	p1 =x1;
	p2 = x2;
}
//��������
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

//����б��
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
	cout << "������ƽ��X����������ϵĸ�����" << onthe_x_GridEdge << endl;
	cout << "������ƽ��Y����������ϵĸ�����" << onthe_y_GridEdge << endl;
	cout << "ͨ������Ա��޳��Ĳ��Ե������" << exclude_by_CoorContast << endl;
	cout << "������������" << cross_product_Count << endl;
	cout << "���Ե��ڶ���������ޱߵ�Cell�еĸ�����" << outsideCellPointCount << endl;
	cout << "���Ե��ڶ���������ޱߵ�Cell�еĸ�����" << insideCellPointCount << endl;
	cout << "���Ե��ں��ߵ�Cell�еĸ���:" << inEdgeCellPointCount << endl;
	cout << "���Ե����ڶ���α��ϵĸ���:" << ontheEdgeCount << endl;
	cout << "�����������α��غϵĴ���:" << superpositionCount << endl;
	cout << "���ڶ���α߿��ⲿ�ĸ���:" << outoftheBoundingbox << endl;
	cout << "�������ޱߵ������еĸ���:" << noEdgeCellPointCount << endl;
	cout << "���Խ׶�����Աȴ���:" << compareCount << endl;
	cout << "ƽ��X���ѯ����:" << searchXCount << endl;
	cout << "ƽ��Y���ѯ����:" << searchYCount << endl;
	cout << "ƽ��Y���ѯ����:" << searchYCount << endl;
	cout << "�ӷ�����:" << addCount << endl;
	cout << "�˷�����:" << multiplicationCount << endl;
}