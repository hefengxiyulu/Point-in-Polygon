#pragma once

#ifndef GRIDPIP_H
#define GRIDPIP_H 1

#include "geometry.h"
#include <tchar.h>
#include <windows.h>
#include  "2DRGP.h"
#include <vector>
#include <string>
#include <algorithm>
#include<math.h>
#include<iostream>

using namespace std;

#define ZERO_OFFSET 1.0E-20		//used by adding edge reference function

#define CELL_UNKNOWN -1
#define CELL_OUT	0
#define CELL_IN		1
#define CELL_SUSPECT 2

#define GRID_RES_TYPE_GIVEN 0            //指定分辨率
#define GRID_RES_TYPE_CLASSIC 1       //经典方法计算分辨率
#define GRID_RES_TYPE_EDGELENGTH 2 //经典方法+按照边长设定分辨率
#define GRID_RES_TYPE_OPTIMAL 3     //近似最优分辨率

//图学学报start
#define GRID_RES_TYPE_GCP  4          //图学学报方法
#define LEFT 0
#define RIGHT 1
#define MAX_DOUBLE 1.7E+308

#define INSIDE 1
#define OUTSIDE 0
#define BORDER -1

#define eps 1e-6
//图学学报 end

#define STATISTICS_STYLE_DETAIL 0
#define STATISTICS_STYLE_SIMPLE 1

#define ESTIMATE_REF_STYLE_ACCURATE 0	
#define ESTIMATE_REF_STYLE_REDUNDANT 1

#define TP_MODE_GEN_RANDOM 0
#define TP_MODE_IMPORT 1

#define PIP_MODE_NORMAL 0		//无健壮性调整
#define PIP_MODE_ROBUST 1		//有健壮性调整

#define SAMPLE_COUNT 50
#define SAMPLE_START 0.5 
#define SAMPLE_END 1.9
#define STORAGE_THRESHOLD 128000000
#define POINT_SAMPLE_COUNT 15

#define RES_SAMPLE_STYLE_WHOLE 0 
#define RES_SAMPLE_STYLE_SEGMENT 1

//#define PIP_ROBUST_EPSILON 1.0  //健壮性调整使用的微小距离

#define DYNAMIC_ARRAY_DEFAULT_SIZE 16		//动态数组缺省数组大小
#define DEFAULT_TP_COUNT 100		//自动生成被测点的缺省数量

#define AS_TYPE_GRID 0		//use grid
#define AS_TYPE_QUADTREE 1	//use quadtree
#define AS_TYPE_RAY_CROSSING 2 //use ray crossing
#define AS_TYPE_RECURSIVEGRID 3 //use recursive grid

#define ADD_EDGE_DDA_ONCE	0	//use 2DDDA and allocate reference memory once for all
#define ADD_EDGE_DDA_DEMAND	1	//use 2DDDA and allocate reference memory on demand
#define ADD_EDGE_BRESENHAM_ONCE	2	//use Bresenham and allocate reference memory once for all
#define ADD_EDGE_BRESENHAM_DEMAND	3	//use Bresenham and allocate reference memory once for all
#define ADD_EDGE_DDA_RGP   4    //use 2DDDA and allocate reference memory once for RGP method
#define ADD_EDGE_DDA_OGP  5   //use 2DDDA and allocate reference memory once for OGP method


#define CP_TRANS_MODE_LEFT2RIGHT 0	    //transmit the cp property from left to right, bottom to top
#define CP_TRANS_MODE_SPIRAL 1				//transmit the cp property in spiral order
#define CP_TRANS_MODE_ZIGZAG 2			    //transmit the cp property in zigzag order
#define CP_TRANS_MODE_CIRCUITOUS 3	    //transmit the cp property in circuitous order
#define CP_TRANS_MODE_RGP 4                    //transmit the cp property from left to right, bottom to top   set cell property  Set Grid point property
#define CP_TRANS_MODE_OGP 5                   // based on edge orientation


#define USED_METHOD_GCP 0           //GCP method
#define USED_METHOD_RGP 1           //RGP method
#define USED_METHOD_OGP 2          //OGP method
#define USED_METHOD_G_RGP 3     //RGP group method
#define USED_METHOD_G_OGP 4     //OGP group method

#define MAX_DEPTH 2
#define MAX_EDGE 1
#define TOTAL_M 1.0

//动态数组，用于存储每个单元中心点与边的位置关系（左、右）
struct DynamicArray {
	int * position;
	int max_size;

	DynamicArray() {
		position = new int[DYNAMIC_ARRAY_DEFAULT_SIZE];
		max_size = DYNAMIC_ARRAY_DEFAULT_SIZE;
	}

	~DynamicArray() {
		delete position;
	}

	void enlarge() {
		int new_size = max_size * 2;
		int * new_position = new int[new_size];
		memcpy(new_position, position, sizeof(int)*max_size);//把资源内存（src所指向的内存区域） 拷贝到目标内存
		max_size = new_size;
		delete position;
		position = new_position;
	}

	int &operator[](int index)
	{
		if (index >= max_size)
			enlarge();
		return position[index];
	}
};

//Statistics for GCP algorithm
struct Statistics {
	//Inside point number
	int insidePointCount;
	//Outside point number
	int outsidePointCount;
	//Measured time for initialization
	FLTYPE realInitTime;
	//Measured time for  PIP queries (all points)
	FLTYPE realTestTime;
	//Measured time for the whole procedure
	FLTYPE realTotalTime;
	//Estimated average intersection times per tested point
	float estimateInterCount;
	//Measured average intersection times per tested point
	float realInterCount;
	//Estimated number of edge references
	int estimateRefCount;
	//Measured number of edge references
	int realRefCount;
	//Storage cost for polygon data
	int storageNormal;
	//Storage cost for accelerting structure
	int storageAS;
	//Total number of intersection testes in the preprocessing
	int prepInterCount;
	//Total number of extra intersection testes in robust mode (in PIP)
	int extraInterCount;
	//Total number of extra checked cells in robust mode (in PIP)
	int extraCheckedCellCount;
	//Total number of tested points fell in non-emty cells
	int nonemptyTPCount;
	//Total number of tested points fell in suspect cells
	int suspectTPCount;

	FLTYPE setCellPropTime;

	//items for quadtree
	int internalNodeCount;
	int leafNodeCount;		//NOTE: totalNodeCount = internalNodeCount + leafNodeCount;
	int emptyLeafNodeCount;

	//items for grid / recusive grid
	int insideCellCount;
	int outsideCellCount;
	int suspectCellCount;	//NOTE: totalCellCount = insideCellCount + outsideCellCount + suspectCellCount
	int emptyCellCount;
	int nonEmptyCellCount;

	//估算初始化时间、检测时间和总时间
	FLTYPE estimateInitTime;
	FLTYPE estimateTestTime;
	FLTYPE estimateTotalTime;

	Statistics() { clear(); }
	~Statistics() {}

	void clear() { memset(this, 0, sizeof(*this)); };
};

struct GridPIP2D {

	//Tested 2D polygon (support multi-polygons, used by dynamic applications)
	GCPPolygon2D * testedPolygon;
	int testedPolygonCount;
	int curPolygonIndex;
	//Acceleration structure -- grid
	Grid2D *grid;
	RecursiveGrid2D * rgrid;
	//Acceleration structure -- quadtree
	QuadTree * quadtree;
	//Tested points list
	Point2D * testedPoint;	//裁剪时作为线段的端点（两个一组）
	int testedPointCount;
	//Tested result
	int *testedResult;
	//Performance statistics
	Statistics stat;

	/////////////////////////////////////////////////////////////////////////////////////////  
	//RGP   start   图学学报
	FLTYPE xmin,SizeX , ymin, SizeY ;
	//record the number of cell which contain edge 
	int SuspectCellCount;
	RGPCell2D *GcptestedPolygon;
	Line2D *line;

	//start 图学学报RGP
	//struct EdgeSegment2D *RgpEdgeSegmentRef;  //存储网格单元中包含的边片段
	//struct RGPEdgeRef2D *RgpEdgeRef;
	struct RGPCell2D *RgpPoint;  //图学学报 存储网格交点坐标
	//struct RgpGridEdgeSegment *RgpIntersectPointX;   // 图学学报，储存多边形边与网格线交点 平行X轴
	//struct RgpGridEdgeSegment *RgpIntersectPointY;   //图学学报，储存多边形边与网格线交点  平行Y轴
	
	struct RgpPointInSegment *RgpIntersectPointX;   // 图学学报，储存多边形边与网格线交点 平行X轴
	struct RgpPointInSegment *RgpIntersectPointY;   //图学学报，储存多边形边与网格线交点  平行Y轴
	
	//struct RgpPointInSegment *Rgppoint;//作为整体分配的点指针连表

	int count_BORDER;

	void RgpGetIntersectionPoint(Point2D* a, Point2D* b, MySortOfPoint *mySP0);
	void RgpGetIntersectionBetweenCellAndEdge(Point2D* p1, Point2D* p2, Point2D* totalPXY, int& count);
	void GetMaxMinCoordinate(double &min_a, double& max_a, double& min_b, double& max_b, Point2D* p1, Point2D* p2);
	//获取两点围成的方框所占的网格坐标
	void GetCellCoordinateRange(double minPCX, double maxPCX, double minPCY, double maxPCY,
		int& minCellX, int& maxCellX, int& minCellY, int& maxCellY);
	int JudgeInOutProperty(Point2D* p1, Point2D* p2);
	void SortTheIntersectionPoint(int count, Point2D* a, Point2D* temp0, MySortOfPoint *mySP0);

	void SetIntersectPointSite(FLTYPE coord_value,int index,int flag); //将多边形边放置到相应的网格线片段

	vector <vector <double> > vecPointX2;//列 
	vector <vector <double> > vecPointY2;//行
	vector <vector <double> >FinalVecPointX;   //排序后的各个横轴网格线上的交点
	vector <vector <double> >FinalVecPointY;   // 排序后的各个纵轴网格线上的交点

	//Set the position property of cell intersect point
	void RgpsetCellIntersectProp();//设置网格交点的位置属性 RGP method
	void OgpsetCellIntersectProp();//设置网格交点的位置属性 OGP method
	void getIntersectPointCoord();   //获取多边形边与网格线的交点坐标
	int GetNumBetweentwoPoint(vector<double >& nums, double x1, double x2);//获取两个网格交点之间的  多边形边与网格线的交点个数
	int InvertFlag(int flag);
	int Equal(double a, double b, double delta);
	int Equal_stat(double a, double b, double delta);
	int RgpcreateGrid();
	int G_RgpcreateGrid();//used for group method
	void RgpMarkCells();//标记网格单元的内外属性
	void G_RgpMarkCells();//used for group method
	void SetCellMiddlePointCoord();//计算网格单元中心点的y坐标值
	//计算测试点位置属性
	void RgpCheckPoint();
	void RgpCheckPoint_Segment();
	void RgpChekPoint_Segment_noStat();
	void G_RgpChekPoint_Segment_noStat();
	int ReturnPointCInGrid(FLTYPE x, FLTYPE y, int & x_index, int & y_index);//计算临近网格交点的坐标
	int TestEdgeSingularCase(Point2D* p1, int x, int y);
	int ReturnIntersectionPointsNumX(FLTYPE y1, FLTYPE y2, int x);////返回同一纵轴上两点中的交点个数
	int ReturnIntersectionPointsNumY(FLTYPE x1, FLTYPE x2, int y);//返回同一横轴两个点之间的交点个数
	int GetCountNumInSegment(FLTYPE a,FLTYPE b,int segment_index,int flag);// 获取给定区间中交点的数目，flag 表示查找X轴片段或Y周片段  0表示横轴，1表示纵轴
	int GetCountNumInSegment_noStat(FLTYPE a, FLTYPE b, int segment_index, int flag);//无统计数据
	bool Intersect1(Point2D* aa, Point2D* bb, Point2D* cc, Point2D* dd);
	double Mult(Point2D* a, Point2D* b, Point2D* c)
	{
		return (a->x - c->x)*(b->y - c->y) - (b->x - c->x)*(a->y - c->y);
	}

	//used in group method
	void quicksort(FLTYPE a[], int left, int right); //快排
	RGP_Statistics Rgpstat;
    //RGP   end

	//GCP  strat
	bool JudgeCollineation(Point2D* p1, Point2D* p2, Point2D* q);
	bool JudgeLineSuperposition(Point2D* A1, Point2D* A2, Point2D* B1, Point2D* B2);
	double cross(Point2D* A, Point2D* B, Point2D* C);
	bool rectsIntersect(Point2D* S1, Point2D* E1, Point2D* S2, Point2D* E2);
	//GCP end
	
	//OGP start
	int OgpcreateGrid();
	void OgpCheckPoint_Segment();              //代码中添加统计数据
	void OgpCheckPoint_Segment_noStat();  //代码中无添加统计数据
	void G_OgpCheckPoint_Segment_noStat();
	//int GetRelativeLocation(Point2D* p1,Point2D* p2,Point2D*q,FLTYPE y0);    //获取测试点在最近边的左侧还是右侧，边的右侧为多边形内,y0为测试边与多边形边的交点坐标
	int GetRelativeLocation(RGPEdgeRef2D* cur_Ogp_edge_ref, Point2D*q);    //获取测试点在最近边的左侧还是右侧，边的右侧为多边形内,y0为测试边与多边形边的交点坐标
	Point2D *minP1 = new Point2D();
	Point2D *minP2 = new Point2D();
	//OGP end
	///////////////////////////////////////////////////////////////////////////////////////////  

	//The region occupied by query points (used by dynamic test)
	Point2D testedPointRegion[2];
	//Grid's bounding box
	Point2D grid_boundingbox[2];
	//the ratio of length to width  for a grid cell
	float cell_size_ratio;
	//grid size
	FLTYPE grid_size[2];
	//grid resolution
	int grid_res[2];
	//Mode of grid resolution
	int gridResMode;
	//Mode of robust adjustment
	int robustMode;
	//Mode of displaying statistics
	int statMode;
	//Mode of  estimating the number of edge references
	int estimatedRefMode;
	//Mode of acquiring the tested point
	int tpMode;
	//Precision used by robust test
	FLTYPE robustPrecision;
	//Type of the accelerating atructure
	int asType;
	//Quadtree's terminating criterion - max depth
	int maxDepth;
	//Quadtree's terminating criterion - max edge count
	int maxEdgeCount;
	//Method on adding edge to grid cells
	int addEdgeMethod;
	//Transmission mode of cp includion properties
	int cpTransMode;
	// select  method
	int usedMethod;

	DynamicPointArray * segmentClipResult;
	int segmentCount;

	GridPIP2D();
	~GridPIP2D() { clearData(); }

	//set member variables with default value
	void configData();
	//Clear the whole structure, release the memory
	void clearData();
	//Clear all the data except the polygon itself
	void clearPartData();
	//Read data from file
	int readData(const char * fn, int type);
	int readMultiPolygons(TCHAR * fn);		//from XYZ file
	//Initialize some data
	void initData();
	//Create grid
	int createGrid();
	//Insert edges into the grid, 2DDDA, allocate the edge reference mamory in one time
	void Rgp_addEdgeRef_2DDDA();
	void Ogp_addEdgeRef_2DDDA();
	void addEdgeRef_2DDDA();
	//2DDDA, allocate the edge reference mamory one by one
	void addEdgeRef_2DDDA_2();
	//Insert edges into the grid (redlark)
	void addEdgeRef_redlark();
	//Bresenham, allocate the edge reference mamory in one time
	void addEdgeRef_Bresenham();
	//Bresenham, allocate the edge reference mamory on by one
	void addEdgeRef_Bresenham_2();
	//Set the position property of center point
	void setCellProp();
	//Accelerate the setting by record center point's relative position to edges  
	void setCellProp_Fast();
	//Other tranlating mode for comparison (non-robust)
	void setCellProp_spiral();
	void setCellProp_zigzag();
	void setCellProp_circuitous();
	//Determine the intersection between two segments
	bool isIntersect(Point2D * p0, Point2D * p1, Point2D * p2, Point2D *p3);
	//Accelerate the intersection   
	bool isIntersect(Point2D *p0, Point2D * p1, Point2D * p2, Point2D * p3, DynamicArray & da, int edge_index, bool flag);
	//Estimate the total number of edge references
	int estimateRef(int U, int V, int style);
	//Point-in-Polygon determination algorithm
	void PIP();
	//Robust enhanced Point-in-Polygon determination algorithm
	void PIP_robust();
	//Generate tested points with uniform distribution
	void generateTestedPoint(int type, int xcount, int ycount);

	//Generate random points
	FLTYPE get_random(FLTYPE start, FLTYPE end);
	//Count the number of inside points and outside points
	void countPoint(int output);
	//Get the storage cost
	void calcStorageCost(int U, int V, int * s_basic, int * s_grid);
	//Output statistics
	void printStatistics(char * log_filename, int U, int V, int style);
	//Output statistics in detail mode
	void printDetailedStatistics(FILE *fp, TCHAR * plyname);
	//Generate a series of  changing polygons, used by dynamic PIP test demo
	void generateDynamicPolygons();
	//Generate convex polygon
	void generateConvexPolygon();
	//Generate star polygon
	void generateStarPolygon();
	//创建双层递归网格
	void createRGrid();
	void RGridPIP();
	int getRGStorage(RecursiveGrid2D * rg, int & refcount, int & emptycellcount, int & nonemptycellcount);

	//compariing experiment methods
	//quadtree
	int createQuadTree();
	QuadTreeNode * createQuadTreeRecursive(EdgeRef2D * source, FLTYPE l, FLTYPE r, FLTYPE b, FLTYPE u, int level, Point2D reference, int ref_point_prop);
	QuadTreeNode * determineNodeRecursive(QuadTreeNode * node, Point2D * point);
	void quadtreePIP();
	//ray crossing
	void raycrossingPIP();

	//get the storage costs
	void getQuadtreeInfoRecursive(QuadTreeNode * node);
	void getGridInfo();
	void getStorage();
	void getResult();

	void optimalResolutionExperiment(char *polygon_file_name); //计算不同分辨率的估算值、实测值，用于最佳分辨率的试验
	void normalExperiment(int res_mode, int pip_mode);  //一般测试
	void generateResolution(float start, float end, int sample_count, int * sample, int style);   //生成所需的不同分辨率
	void estimateTime(int U, int V, FLTYPE * time1, FLTYPE * time2, FLTYPE * time3);
	void estimateTime(int cell_number, FLTYPE * time1, FLTYPE * time2, FLTYPE * time3);
	void collectEstimateStatistics(int U, int V);

	//多边形线裁剪，网格加速求交点
	void segmentClip(Point2D & v1, Point2D & v2);
	void getIntersectionInCell(Point2D * v0, Point2D * v1, int cell_index, FLTYPE * rlist, int & rcount, bool * mailbox);
	int getIntersection(Point2D *A, Point2D *B, Point2D *C, Point2D *D, FLTYPE & r, FLTYPE & s);
	int isInside(Point2D * p);
	void  insertSort(FLTYPE  * rlist, int  rcount);

	//输出结果至OBJ文件（type: CELL_IN-内部点，CELL_OUT-外部点）
	void export2OBJ(const char * filename, int type);
};

struct SmallTimer {
	LARGE_INTEGER ticksPerSecond;
	LARGE_INTEGER tick0, tick1;
	double time;

	SmallTimer()
	{
		memset(this, 0, sizeof(*this));
		QueryPerformanceFrequency(&ticksPerSecond);
	}
	~SmallTimer() {};

	void start() { QueryPerformanceCounter(&tick0); }
	void end()
	{
		QueryPerformanceCounter(&tick1);
		time = (tick1.QuadPart - tick0.QuadPart) / (double)ticksPerSecond.QuadPart;
	}
};

#endif