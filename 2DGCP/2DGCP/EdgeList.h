#pragma once
#include "geometry.h"
class EdgeList
{
private:
	Point2D p1, p2;
	int EdgeType;  // Either LOOP or RING
	int UsedFlag;
	Point2D CellPoint1;//wang add 用来存放该条边所在的方格坐标
	int EdgeFlag;
public:
	EdgeList* Next;
	EdgeList() {}
	EdgeList(int et, const Point2D& a, const Point2D& b)
	{
		EdgeType = et; p1 = a; p2 = b; Next = NULL;
	}

	EdgeList(int et, const Point2D& a, const Point2D& b, const Point2D& c)
	{
		EdgeType = et; p1 = a; p2 = b; CellPoint1 = c; Next = NULL;
	}

	EdgeList(const Point2D& a, const Point2D& b)
	{
		p1 = a; p2 = b; Next = NULL;
	}
	void GetEdgeList(int& et, int& uf, Point2D& a, Point2D& b, Point2D& c)
	{
		et = EdgeType; uf = UsedFlag; a = p1; b = p2; c = CellPoint1;
	}
	void GetPoint(Point2D& a, Point2D& b) { a = p1; b = p2; }

	~EdgeList() { ; }

	void ReturnEdge(Point2D& a, Point2D& b) { a = p1; b = p2; }
	int ReturnEdgeType() { return EdgeType; }
	void ResetUsedFlag() { UsedFlag = 0; }
	int ReturnUsedFlag() { return UsedFlag; }
	void SetUsedFlag() { UsedFlag = 1; }
	//////////////////
	void SetCellPoint(Point2D &c) { CellPoint1 = c; }//wang add
	void SetPoint(Point2D& a, Point2D& b) { p1 = a; p2 = b; }
	void ReturnCellPoint(Point2D &a) { a = CellPoint1; }
	void SetEdgeFlag(double slope, double x, double y, double xsize, double ysize);//设置边的的类型，垂直，平行、其他
	int GetEdgeFlag() { return EdgeFlag; }//获取边的属性
	void CopyList(EdgeList* e);

	void ReturnMaxMinCoordinate(double &min_a, double& max_a, double& min_b, double& max_b, Point2D p1, Point2D p2)
	{
		min_a = p1.x < p2.x ? p1.x : p2.x;
		max_a = p1.x > p2.x ? p1.x : p2.x;
		min_b = p1.y < p2.y ? p1.y : p2.y;
		max_b = p1.y > p2.y ? p1.y : p2.y;
	}
	//	Point CalculateCellCoordinate (Point& a);
};