#include"pch.h"
#include "stdafx.h"
#include <fstream>
#include <strstream>
#include <time.h>
#include "gridpip.h"
#include <algorithm>
//#include "2DRGP.h"

using namespace std;

GridPIP2D::GridPIP2D()
{
	memset(this, 0, sizeof(*this));
	configData();
}

void GridPIP2D::configData()
{
	this->gridResMode = GRID_RES_TYPE_CLASSIC;       //1 经典方法计算分辨率
	//this->robustMode = PIP_MODE_NORMAL;                   //无健壮性调整
	this->robustMode = PIP_MODE_ROBUST;
	this->robustPrecision = 0.00001;
	this->statMode = STATISTICS_STYLE_DETAIL;
	this->estimatedRefMode = ESTIMATE_REF_STYLE_REDUNDANT;
	this->grid_res[0] = 100;
	this->grid_res[1] = 100;
	this->testedPointCount = 10000;
	this->tpMode = TP_MODE_GEN_RANDOM;
	this->testedPolygonCount = 0;
	this->asType = AS_TYPE_GRID;   //use grid
	this->asType = AS_TYPE_GRID;   //use grid
	this->maxDepth = 8;
	this->maxEdgeCount = 5;
	////////
	switch (this->usedMethod)
	{
	case USED_METHOD_GCP:
		this->addEdgeMethod = ADD_EDGE_DDA_ONCE;
		this->cpTransMode = CP_TRANS_MODE_LEFT2RIGHT;
		break;
	case USED_METHOD_RGP:
		this->addEdgeMethod = ADD_EDGE_DDA_RGP;
		this->cpTransMode = CP_TRANS_MODE_RGP;
		break;
	case USED_METHOD_OGP:
		this->addEdgeMethod = ADD_EDGE_DDA_OGP;
		this->cpTransMode = CP_TRANS_MODE_OGP;
		break;
	case USED_METHOD_G_RGP:
		this->addEdgeMethod = ADD_EDGE_DDA_RGP;
		this->cpTransMode = CP_TRANS_MODE_RGP;
		break;
	case USED_METHOD_G_OGP:
		this->addEdgeMethod = ADD_EDGE_DDA_OGP;
		this->cpTransMode = CP_TRANS_MODE_OGP;
		break;
	default:
		cout << "Input correct configure parameters!" << endl;
		break;
	}
	//this->addEdgeMethod = ADD_EDGE_DDA_ONCE;                    //GCP
	//this->usedMethod = USED_METHOD_GCP;                                //GCP
	//this->cpTransMode = CP_TRANS_MODE_LEFT2RIGHT;         //GCP
	//////////////////////////
	//this->addEdgeMethod = ADD_EDGE_DDA_RGP;                      //RGP
	//this->cpTransMode = CP_TRANS_MODE_RGP;                        //RGP
	//this->usedMethod = USED_METHOD_RGP;                              //RGP
	///////////////////////
	//this->addEdgeMethod= ADD_EDGE_DDA_OGP;                       //OGP
	//this->cpTransMode= CP_TRANS_MODE_OGP;                         //OGP
	//this->usedMethod= USED_METHOD_OGP;                                //OGP
}

void GridPIP2D::clearData()
{
	if (testedPolygon != NULL)
	{
		if (testedPolygonCount == 1)
		{
			delete testedPolygon;
			testedPolygon = NULL;
		}
		else if (testedPolygonCount > 1)
		{
			delete[] testedPolygon;
			testedPolygon = NULL;
		}
	}
	if (grid != NULL)
	{
		delete grid;
		grid = NULL;
	}
	if (rgrid != NULL)
	{
		delete rgrid;
		rgrid = NULL;
	}
	if (quadtree != NULL)
	{
		delete quadtree;
		quadtree = NULL;
	}
	if (testedPoint != NULL)
	{
		delete[] testedPoint;
		testedPoint = NULL;
	}
	if (testedResult != NULL)
	{
		delete[] testedResult;
		testedResult = NULL;
	}
	if (segmentClipResult != NULL)
		delete segmentClipResult;

	memset(this, 0, sizeof(*this));
}

//Clear all the data except the polygon itself
void GridPIP2D::clearPartData()
{
	if (grid != NULL)
	{
		delete grid;
		grid = NULL;
	}
	if (quadtree != NULL)
	{
		delete quadtree;
		quadtree = NULL;
	}
	if (testedPoint != NULL)
	{
		delete[] testedPoint;
		testedPoint = NULL;
	}
	if (testedResult != NULL)
	{
		delete[] testedResult;
		testedResult = NULL;
	}
}

//Read multi-polygon data from XYX format file
//"DESCRIPTION=Contour Line, Minor"
//"NAME=10 m"
//"[x],[y],10"
//NOTE: assume all the polygon are closed
int GridPIP2D::readMultiPolygons(TCHAR * fn)
{
	if (fn[0] == 0)
		return -1;

	ifstream ifs;
	ifs.open(fn, ios::binary);
	if (ifs.bad())
	{
		printf("Can not open polygon file.\n");
		return -1;
	}
	ifs.seekg(0, ios_base::end);
	streampos size = ifs.tellg();
	ifs.seekg(0, ios_base::beg);

	int tempsize;
	tempsize = size;
	char * buf = new char[tempsize];
	char type;
	int vertexCount = 0;
	int edgeCount = 0;
	int ringCount = 0;
	int polygonCount = 0;
	double x, y, z, old_z = -1;

	//first pass, count the polygons
	while (ifs.getline(buf, size))
	{
		if (strncmp(buf, "DESCRIPTION=", 12) == 0)
		{
			ifs.getline(buf, size);
			ifs.getline(buf, size);
			istrstream s(buf);
			s >> x >> type >> y >> type >> z;
			if (z != old_z)
			{
				polygonCount++;
				old_z = z;
			}
		}
	}
	this->testedPolygon = new GCPPolygon2D[polygonCount];
	this->testedPolygonCount = polygonCount;

	//second pass, count every polygon's vertexes, edges and rings
	ifs.clear();
	ifs.seekg(0, ios::beg);
	polygonCount = 0;
	vertexCount = 0;
	edgeCount = 0;
	ringCount = 0;
	old_z = -1;

	while (ifs.getline(buf, size))
	{
		if (strncmp(buf, "DESCRIPTION=", 12) == 0)
		{
			//a new ring, complete last ring's data
			if (polygonCount != 0 || (polygonCount == 0 && vertexCount != 0))
			{
				edgeCount--;
			}

			ifs.getline(buf, size);	//"NAME=..."
			ifs.getline(buf, size);
			istrstream s(buf);
			s >> x >> type >> y >> type >> z;

			if (z != old_z)	//a new polygon's first ring
			{
				if (polygonCount != 0 || (polygonCount == 0 && vertexCount != 0))
				{
					this->testedPolygon[polygonCount].vertexCount = vertexCount;
					this->testedPolygon[polygonCount].edgeCount = edgeCount;
					this->testedPolygon[polygonCount].ringCount = ringCount;
					polygonCount++;
				}
				vertexCount = 1;
				edgeCount = 1;
				ringCount = 1;
				old_z = z;
			}
			else //a new ring
			{
				vertexCount++;
				edgeCount++;
				ringCount++;
			}
		}
		else
		{
			if (buf[0] != 13)	//skip empty line
			{
				vertexCount++;
				edgeCount++;
			}
		}
	}
	//the last polygon's data
	this->testedPolygon[polygonCount].vertexCount = vertexCount;
	edgeCount--;
	this->testedPolygon[polygonCount].edgeCount = edgeCount;
	this->testedPolygon[polygonCount].ringCount = ringCount;
	polygonCount++;

	//allocate memory for each polygon
	for (int i = 0; i < polygonCount; i++)
	{
		this->testedPolygon[i].vertexTable = new Point2D[this->testedPolygon[i].vertexCount];
		this->testedPolygon[i].edgeTable = new Edge2D[this->testedPolygon[i].edgeCount + 1];
		this->testedPolygon[i].ringTable = new int[this->testedPolygon[i].ringCount];
	}

	//third pass, fill in data
	ifs.clear();
	ifs.seekg(0, ios::beg);
	polygonCount = 0;
	vertexCount = 0;
	edgeCount = 0;
	ringCount = 0;
	old_z = -1;
	int ringStartVertex = 0;
	while (ifs.getline(buf, size))
	{
		//new ring
		if (strncmp(buf, "DESCRIPTION=", 12) == 0)
		{
			//complete last ring's data
			if (polygonCount != 0 || (polygonCount == 0 && vertexCount != 0))
			{
				this->testedPolygon[polygonCount].edgeTable[edgeCount - 2].endIndex = ringStartVertex;
				edgeCount--;
			}

			//skip NAME=... line
			ifs.getline(buf, size);
			//read current height
			ifs.getline(buf, size);
			istrstream s(buf);
			s >> x >> type >> y >> type >> z;
			//if this ring is the start of a new polygon
			if (z != old_z)
			{
				if (polygonCount != 0 || (polygonCount == 0 && vertexCount != 0))
					polygonCount++;

				this->testedPolygon[polygonCount].vertexTable[0].x = x;
				this->testedPolygon[polygonCount].vertexTable[0].y = y;
				this->testedPolygon[polygonCount].edgeTable[0].startIndex = 0;
				this->testedPolygon[polygonCount].edgeTable[0].endIndex = 1;
				this->testedPolygon[polygonCount].ringTable[0] = 0;

				ringStartVertex = 0;
				vertexCount = 1;
				edgeCount = 1;
				ringCount = 1;
				old_z = z;

			}
			else //not a new polygon, but a new ring
			{
				ringStartVertex = vertexCount;
				this->testedPolygon[polygonCount].vertexTable[vertexCount].x = x;
				this->testedPolygon[polygonCount].vertexTable[vertexCount].y = y;
				this->testedPolygon[polygonCount].edgeTable[edgeCount].startIndex = vertexCount;
				this->testedPolygon[polygonCount].edgeTable[edgeCount].endIndex = vertexCount + 1;
				this->testedPolygon[polygonCount].ringTable[ringCount] = edgeCount;
				vertexCount++;
				edgeCount++;
				ringCount++;
			}
		}
		else
		{
			if (buf[0] != 13)	//skip empty line, normal vertex data
			{
				istrstream s(buf);
				s >> x >> type >> y;
				this->testedPolygon[polygonCount].vertexTable[vertexCount].x = x;
				this->testedPolygon[polygonCount].vertexTable[vertexCount].y = y;
				this->testedPolygon[polygonCount].edgeTable[edgeCount].startIndex = vertexCount;
				this->testedPolygon[polygonCount].edgeTable[edgeCount].endIndex = vertexCount + 1;
				vertexCount++;
				edgeCount++;
			}
		}
	}//while

	//complete the last ring's edge data
	this->testedPolygon[polygonCount].edgeTable[edgeCount - 2].endIndex = ringStartVertex;

	return 1;
}

//Read data into tested polyon or tested point structure
//type - 0: tested polygon; 1: tested points
int GridPIP2D::readData(const char * fn, int ftype)
{
	if (fn[0] == 0)
		return -1;

	//读入多边形数据
	if (ftype == 0)
	{
		//读第一遍，计算顶点个数，分配内存
		ifstream ifs;
		//ifs.open(fn, ios::binary);
		ifs.open(fn, ios::binary);
		if (ifs.bad())//如果在读写过程中出错，返回 true
		{
			printf("Can not open polygon file.\n");
			return -1;
		}
		ifs.seekg(0, ios_base::end);//将指针移到文件尾
		streampos size = ifs.tellg();//代表当前get 流指针的位置 (用tellg) 
		ifs.seekg(0, ios_base::beg);

		int tempsize;
		tempsize = size;
		char * buf = new char[tempsize];
		char type;
		int vertexCount = 0;
		int edgeCount = 0;
		int ringCount = 0;
		int temp;
		bool isFrom0 = false;

		while (ifs.getline(buf, size))
		{
			istrstream s(buf);
			s >> type;
			switch (type)
			{
			case 'v':
				vertexCount++;          // 记数点的个数
				break;
			case 'f':
			{
				while (s >> temp)
				{
					if (temp == 0)
						isFrom0 = true;
					edgeCount++;        //统计边的个数
				}
				ringCount++;
				break;
			}
			};
			type = ' ';
		}

		if (this->testedPolygon != NULL)
		{
			delete this->testedPolygon;
			this->testedPolygon = NULL;
		}
		this->testedPolygon = new struct GCPPolygon2D;
		this->testedPolygonCount = 1;
		this->testedPolygon->vertexCount = vertexCount;
		this->testedPolygon->edgeCount = edgeCount;
		this->testedPolygon->ringCount = ringCount;
		this->testedPolygon->vertexTable = new Point2D[vertexCount];
		this->testedPolygon->edgeTable = new Edge2D[edgeCount];
		this->testedPolygon->ringTable = new int[ringCount];

		//读第二遍，写入内存
		ifs.clear();// 去除 ifs 中的错误标记(如文件末尾标记或读取失败标记等)
		ifs.seekg(0, ios::beg);
		vertexCount = 0;
		edgeCount = 0;
		ringCount = 0;

		while (ifs.getline(buf, size))
		{
			istrstream s(buf);
			s >> type;
			switch (type)
			{
			case 'v':
				s >> this->testedPolygon->vertexTable[vertexCount].x;
				s >> this->testedPolygon->vertexTable[vertexCount].y;
				vertexCount++;
				break;
			case 'f':
			{
				//第一条边
				int temp, vertexIndex;
				s >> temp;
				if (isFrom0 == false)  temp = temp - 1;
				this->testedPolygon->edgeTable[edgeCount].startIndex = temp;
				this->testedPolygon->ringTable[ringCount] = edgeCount;
				ringCount++;
				edgeCount++;

				//后续边
				while (s >> vertexIndex)
				{
					if (isFrom0 == false)  vertexIndex = vertexIndex - 1;
					this->testedPolygon->edgeTable[edgeCount - 1].endIndex = vertexIndex;
					this->testedPolygon->edgeTable[edgeCount].startIndex = vertexIndex;

					edgeCount++;
				}

				//最后一条边
				this->testedPolygon->edgeTable[edgeCount - 1].endIndex = temp;
			}
			}//switch 
			type = ' ';
		}//while	
	} //if
	else if (ftype == 1)
	{
		ifstream ifs2;
		ifs2.open(fn, ios::binary);
		if (ifs2.bad())
		{
			printf("Can not open point file.\n");
			delete this->testedPolygon;
			this->testedPolygon = NULL;
			return -1;
		}
		//读第一遍，记录顶点个数
		ifs2.seekg(0, ios_base::end);
		streampos size = ifs2.tellg();
		ifs2.seekg(0, ios_base::beg);
		int tempsize;
		tempsize = size;
		char * buf = new char[tempsize];
		char type;
		int vertexCount = 0;
		int edgeCount = 0;
		int temp;

		while (ifs2.getline(buf, size))
		{
			istrstream s(buf);
			s >> type;
			switch (type)
			{
			case 'v':
				vertexCount++;
				break;
			};
			type = ' ';
		}
		//分配内存
		if (this->testedPoint != NULL)
		{
			delete[] this->testedPoint;
			this->testedPoint = NULL;
		}
		this->testedPoint = new Point2D[vertexCount];
		//读第二遍，写入顶点
		ifs2.clear();
		ifs2.seekg(0, ios::beg);
		vertexCount = 0;
		while (ifs2.getline(buf, size))
		{
			istrstream s(buf);
			s >> type;
			switch (type)
			{
			case 'v':
				s >> this->testedPoint[vertexCount].x >> this->testedPoint[vertexCount].y;
				vertexCount++;
				break;
			};
			type = ' ';
		}
		this->testedPointCount = vertexCount;
	}

	return 1;
}

//读入数据后，计算一些基本的量
void GridPIP2D::initData()
{
	if (this->testedPolygon == NULL)
		return;

	//计算多边形包围盒
	for (int i = 0; i < this->testedPolygonCount; i++)
	{
		this->testedPolygon[i].boundingBox[0].x = this->testedPolygon[i].vertexTable[0].x;
		this->testedPolygon[i].boundingBox[0].y = this->testedPolygon[i].vertexTable[0].y;
		this->testedPolygon[i].boundingBox[1].x = this->testedPolygon[i].vertexTable[0].x;
		this->testedPolygon[i].boundingBox[1].y = this->testedPolygon[i].vertexTable[0].y;
		for (int j = 0; j < this->testedPolygon[i].vertexCount; j++)//获取最大、最小坐标值
		{
			if (this->testedPolygon[i].vertexTable[j].x < this->testedPolygon[i].boundingBox[0].x)
				this->testedPolygon[i].boundingBox[0].x = this->testedPolygon[i].vertexTable[j].x;
			if (this->testedPolygon[i].vertexTable[j].y < this->testedPolygon[i].boundingBox[0].y)
				this->testedPolygon[i].boundingBox[0].y = this->testedPolygon[i].vertexTable[j].y;
			if (this->testedPolygon[i].vertexTable[j].x > this->testedPolygon[i].boundingBox[1].x)
				this->testedPolygon[i].boundingBox[1].x = this->testedPolygon[i].vertexTable[j].x;
			if (this->testedPolygon[i].vertexTable[j].y > this->testedPolygon[i].boundingBox[1].y)
				this->testedPolygon[i].boundingBox[1].y = this->testedPolygon[i].vertexTable[j].y;
		}

		//计算X,Y方向上的平均边长
		this->testedPolygon[i].dx = this->testedPolygon[i].dy = 0;
		for (int j = 0; j < this->testedPolygon[i].edgeCount; j++)
		{
			int start = this->testedPolygon[i].edgeTable[j].startIndex;
			int end = this->testedPolygon[i].edgeTable[j].endIndex;
			FLTYPE x0 = this->testedPolygon[i].vertexTable[start].x;
			FLTYPE x1 = this->testedPolygon[i].vertexTable[end].x;
			FLTYPE y0 = this->testedPolygon[i].vertexTable[start].y;
			FLTYPE y1 = this->testedPolygon[i].vertexTable[end].y;
			this->testedPolygon[i].dx += abs(x1 - x0);
			this->testedPolygon[i].dy += abs(y1 - y0);
		}
		this->testedPolygon[i].dx /= this->testedPolygon[i].edgeCount;//计算所有边x轴差值的和，并求平均值
		this->testedPolygon[i].dy /= this->testedPolygon[i].edgeCount;//计算所有边y轴差值的和，并求平均值

		this->testedPointRegion[0].x = this->testedPolygon[0].boundingBox[0].x;
		this->testedPointRegion[0].y = this->testedPolygon[0].boundingBox[0].y;
		this->testedPointRegion[1].x = this->testedPolygon[0].boundingBox[1].x;
		this->testedPointRegion[1].y = this->testedPolygon[0].boundingBox[1].y;;
	}


	/*
	//NOTE: move this part to createGrid function

	//计算网格大小。为计算方便比多边形包围盒稍大(0.1%）
	FLTYPE enlarge[2];
	enlarge[0] = (this->testedPolygon->boundingBox[1].x - this->testedPolygon->boundingBox[0].x) * 0.001;
	enlarge[1] = (this->testedPolygon->boundingBox[1].y - this->testedPolygon->boundingBox[0].y) * 0.001;
	this->grid_boundingbox[0].x = this->testedPolygon->boundingBox[0].x - enlarge[0];
	this->grid_boundingbox[0].y = this->testedPolygon->boundingBox[0].y - enlarge[1];
	this->grid_boundingbox[1].x = this->testedPolygon->boundingBox[1].x + enlarge[0];
	this->grid_boundingbox[1].y = this->testedPolygon->boundingBox[1].y + enlarge[1];
	this->grid_size[0] = this->grid_boundingbox[1].x - this->grid_boundingbox[0].x;
	this->grid_size[1] = this->grid_boundingbox[1].y - this->grid_boundingbox[0].y;

	//计算网格单元的长宽分配比例
	//this->cell_size_ratio = this->grid_size[0]*this->testedPolygon->dy / (this->grid_size[1]*this->testedPolygon->dx);
	*/

} //initData

//Create grid
int GridPIP2D::RgpcreateGrid()
{
	if (this->grid != NULL)
	{
		delete this->grid;
		this->grid = NULL;
	}
	this->asType = AS_TYPE_GRID;     //  0  use grid

	//计算网格大小。为计算方便比多边形包围盒稍大(0.1%）
	FLTYPE enlarge[2];
	enlarge[0] = (this->testedPolygon[curPolygonIndex].boundingBox[1].x - this->testedPolygon[curPolygonIndex].boundingBox[0].x) * 0.01;//源程序该处为0.001
	enlarge[1] = (this->testedPolygon[curPolygonIndex].boundingBox[1].y - this->testedPolygon[curPolygonIndex].boundingBox[0].y) * 0.01;//同上

	this->grid_boundingbox[0].x = this->testedPolygon[curPolygonIndex].boundingBox[0].x - enlarge[0];
	this->grid_boundingbox[0].y = this->testedPolygon[curPolygonIndex].boundingBox[0].y - enlarge[1];
	this->grid_boundingbox[1].x = this->testedPolygon[curPolygonIndex].boundingBox[1].x + enlarge[0];
	this->grid_boundingbox[1].y = this->testedPolygon[curPolygonIndex].boundingBox[1].y + enlarge[1];

	this->grid_size[0] = this->grid_boundingbox[1].x - this->grid_boundingbox[0].x;            //边界框的尺寸大小
	this->grid_size[1] = this->grid_boundingbox[1].y - this->grid_boundingbox[0].y;

	//计算网格单元的长宽分配比例
	//this->cell_size_ratio = this->grid_size[0]*this->testedPolygon->dy / (this->grid_size[1]*this->testedPolygon->dx);

	//计算网格分辨率
	if (this->gridResMode == GRID_RES_TYPE_CLASSIC)  //1  经典方法计算分辨率
	{
		/*
		this->grid_res[0] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[0] / this->grid_size[1] ;
		this->grid_res[0] = this->grid_res[0] << 1 ;
		this->grid_res[1] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[1] / this->grid_size[0] ;
		this->grid_res[1] = this->grid_res[1] << 1 ;
		*/
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		//原方法的计算方式
		/*this->grid_res[0] = sqrt((float)K*W / H);
		this->grid_res[1] = sqrt((float)K*H / W);*/


		//修改后代码(图学学报)
		this->grid_res[0] = 2 * (1 + (int)(sqrt((float)K)*W / H));
		this->grid_res[1] = 2 * (1 + (int)(sqrt((float)K)*H / W));
		//图学学报，新添加代码
		if (this->grid_res[0] > 100)     this->grid_res[0] = 100;
		if (this->grid_res[1] > 100)		 this->grid_res[1] = 100;
		//添加完毕
		printf("Grid resolution X=%d\n", this->grid_res[0]);
		printf("Grid resolution Y=%d\n", this->grid_res[1]);
		//当前修改完毕
	}
	else if (this->gridResMode == GRID_RES_TYPE_EDGELENGTH)
	{
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H =  this->grid_size[1];
		this->grid_res[0] = sqrt((float)K*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)K*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	else if (this->gridResMode == GRID_RES_TYPE_OPTIMAL)    //最佳分辨率Ma,Mb,Ms
	{
		//计算Ma, Mb, Ms和Mo
		int Ma, Mb, Ms, Mo;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		int N = this->testedPolygon[curPolygonIndex].vertexCount;
		float c1 = 4.0;
		float c2 = 4.0;
		float c3 = 1.0;

		float temp = 0.25 * (W * H) / (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy);
		if (N < temp)
			Ma = N;
		else
			Ma = temp;
		Ma = c1 * Ma;

		temp = 4.0 * N * N * (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy) / (W*H);
		if (N > temp)
			Mb = N;
		else
			Mb = temp;
		Mb = c2 * Mb;

		Ms = c3 * this->testedPointCount;
		if (Ms < Ma)
			Ms = Ma;
		if (Ms > Mb)
			Ms = Mb;

		//按边长分配Mo
		this->grid_res[0] = sqrt((float)Ms*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)Ms*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	//clamp the range  
	if (this->grid_res[0] == 0)		this->grid_res[0] = 1;
	if (this->grid_res[1] == 0)		this->grid_res[1] = 1;

	//创建网格
	this->grid = new Grid2D();

	this->grid->boundingBox[0].x = this->grid_boundingbox[0].x;//边界框左上角和右下角的坐标值
	this->grid->boundingBox[0].y = this->grid_boundingbox[0].y;
	this->grid->boundingBox[1].x = this->grid_boundingbox[1].x;
	this->grid->boundingBox[1].y = this->grid_boundingbox[1].y;

	this->grid->gridSize[0] = this->grid_size[0];//边界框的长与高
	this->grid->gridSize[1] = this->grid_size[1];

	this->grid->resolution[0] = this->grid_res[0];//网格分辨率
	this->grid->resolution[1] = this->grid_res[1];

	this->grid->cellCount = this->grid_res[0] * this->grid_res[1];//网格的数量，即网格分别率相乘 

	this->grid->cellSize[0] = (FLTYPE)this->grid->gridSize[0] / this->grid->resolution[0];//小网格的长与宽
	this->grid->cellSize[1] = (FLTYPE)this->grid->gridSize[1] / this->grid->resolution[1];

	this->grid->cell = new GridCell2D[this->grid->cellCount];     //创建网格单元

	//估算指针总数，预分配指针空间
	//this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], ESTIMATE_REF_STYLE_REDUNDANT);
	this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], this->estimatedRefMode);
	//size_t size = sizeof(struct EdgeRef2D) * this->grid->expectRefCount;
	//this->grid->edgeRef = (struct EdgeRef2D *)malloc(size);   //动态内存分配，用于申请一块连续的指定大小的内存块区域以void*类型返回分配的内存区域地址
	//memset(this->grid->edgeRef, 0, size);   //memset:作用是在一段内存块中填充某个给定的值

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//图学学报 start
	xmin = grid_boundingbox[0].x;
	SizeX = this->grid->cellSize[0];
	ymin = grid_boundingbox[0].y;
	SizeY = this->grid->cellSize[1];
	
	size_t gcpsize = (this->grid_res[0] + 1) * (this->grid_res[1] + 1);//网格交点的个数
	this->RgpPoint = new RGPCell2D[gcpsize];

	//计算存储X轴上交点的片段数，横轴数量*（纵轴数量-1）最上边和最底边不可能和多边形边有交点
	size_t gcpXSegmentsize = this->grid_res[0] * (this->grid_res[1] - 1);
	//计算存储Y轴上交点的片段数，纵轴数量*（横轴数量-1）最左边和最右边不可能和多边形边有交点
	size_t gcpYSegmentsize = (this->grid_res[0] - 1) * this->grid_res[1];
	//创建存储多边形边与网格线交点的  片段单元

	this->RgpIntersectPointX = new RgpPointInSegment[gcpXSegmentsize];
	this->RgpIntersectPointY = new RgpPointInSegment[gcpYSegmentsize];

	//估算指针总数，预分配存储多边形边片段的指针空间
	//size的大小需要重新估计，目前利用上述估计的结果
	size_t size_Rgp = sizeof(struct RGPEdgeRef2D) * this->grid->expectRefCount * 2;
	this->grid->RgpEdgeRef = (struct RGPEdgeRef2D *)malloc(size_Rgp);
	memset(this->grid->RgpEdgeRef, 0, size_Rgp);

	//估算指针总数，预分配存储多边形边与网格线交点的指针空间
	/*this->grid->RgpPoint = (struct RgpPointInSegment*)malloc(size);
	memset(this->grid->RgpPoint,0,size);*/
	//图学学报 end
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//创建单元指针列表
	//遍历每条边，计算各个边占据的网格单元，将边指针加入相关单元
	if (this->addEdgeMethod == ADD_EDGE_DDA_ONCE)
		this->addEdgeRef_2DDDA();  //GCP 2012 软件学报
	if(this->addEdgeMethod == ADD_EDGE_DDA_RGP)
		this->Rgp_addEdgeRef_2DDDA();//图学学报
	else if (this->addEdgeMethod == ADD_EDGE_DDA_DEMAND)
		this->addEdgeRef_2DDDA_2();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_ONCE)
		this->addEdgeRef_Bresenham();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_DEMAND)
		this->addEdgeRef_Bresenham_2();

	//计算网格单元中点处的点包信息，设置单元属性（纯内、纯外、混合）
	SmallTimer tm;
	tm.start();
	if (this->cpTransMode == CP_TRANS_MODE_LEFT2RIGHT)
	{
		setCellProp();
	}
	else if (this->cpTransMode == CP_TRANS_MODE_RGP)
	{
		this->RgpsetCellIntersectProp();//设置网格交点位置属性
		this->SetCellMiddlePointCoord();
		this->RgpMarkCells();
	}
	else if (this->cpTransMode == CP_TRANS_MODE_SPIRAL)
		setCellProp_spiral();
	else if (this->cpTransMode == CP_TRANS_MODE_ZIGZAG)
		setCellProp_zigzag();
	else if (this->cpTransMode == CP_TRANS_MODE_CIRCUITOUS)
		setCellProp_circuitous();
	tm.end();
	this->stat.setCellPropTime = tm.time;

	return 1;
} //createGrid

//Group method
int GridPIP2D::G_RgpcreateGrid()
{
	if (this->grid != NULL)
	{
		delete this->grid;
		this->grid = NULL;
	}
	this->asType = AS_TYPE_GRID;     //  0  use grid

	//计算网格大小。为计算方便比多边形包围盒稍大(0.1%）
	FLTYPE enlarge[2];
	enlarge[0] = (this->testedPolygon[curPolygonIndex].boundingBox[1].x - this->testedPolygon[curPolygonIndex].boundingBox[0].x) * 0.01;//源程序该处为0.001
	enlarge[1] = (this->testedPolygon[curPolygonIndex].boundingBox[1].y - this->testedPolygon[curPolygonIndex].boundingBox[0].y) * 0.01;//同上

	this->grid_boundingbox[0].x = this->testedPolygon[curPolygonIndex].boundingBox[0].x - enlarge[0];
	this->grid_boundingbox[0].y = this->testedPolygon[curPolygonIndex].boundingBox[0].y - enlarge[1];
	this->grid_boundingbox[1].x = this->testedPolygon[curPolygonIndex].boundingBox[1].x + enlarge[0];
	this->grid_boundingbox[1].y = this->testedPolygon[curPolygonIndex].boundingBox[1].y + enlarge[1];

	this->grid_size[0] = this->grid_boundingbox[1].x - this->grid_boundingbox[0].x;            //边界框的尺寸大小
	this->grid_size[1] = this->grid_boundingbox[1].y - this->grid_boundingbox[0].y;

	//计算网格单元的长宽分配比例
	//this->cell_size_ratio = this->grid_size[0]*this->testedPolygon->dy / (this->grid_size[1]*this->testedPolygon->dx);

	//计算网格分辨率
	if (this->gridResMode == GRID_RES_TYPE_CLASSIC)  //1  经典方法计算分辨率
	{
		/*
		this->grid_res[0] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[0] / this->grid_size[1] ;
		this->grid_res[0] = this->grid_res[0] << 1 ;
		this->grid_res[1] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[1] / this->grid_size[0] ;
		this->grid_res[1] = this->grid_res[1] << 1 ;
		*/
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		//原方法的计算方式
		/*this->grid_res[0] = sqrt((float)K*W / H);
		this->grid_res[1] = sqrt((float)K*H / W);*/


		//修改后代码(图学学报)
		this->grid_res[0] = 2 * (1 + (int)(sqrt((float)K)*W / H));
		this->grid_res[1] = 2 * (1 + (int)(sqrt((float)K)*H / W));
		//图学学报，新添加代码
		if (this->grid_res[0] > 100)     this->grid_res[0] = 100;
		if (this->grid_res[1] > 100)		 this->grid_res[1] = 100;
		//添加完毕
		printf("Grid resolution X=%d\n", this->grid_res[0]);
		printf("Grid resolution Y=%d\n", this->grid_res[1]);
		//当前修改完毕
	}
	else if (this->gridResMode == GRID_RES_TYPE_EDGELENGTH)
	{
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		this->grid_res[0] = sqrt((float)K*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)K*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	else if (this->gridResMode == GRID_RES_TYPE_OPTIMAL)    //最佳分辨率Ma,Mb,Ms
	{
		//计算Ma, Mb, Ms和Mo
		int Ma, Mb, Ms, Mo;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		int N = this->testedPolygon[curPolygonIndex].vertexCount;
		float c1 = 4.0;
		float c2 = 4.0;
		float c3 = 1.0;

		float temp = 0.25 * (W * H) / (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy);
		if (N < temp)
			Ma = N;
		else
			Ma = temp;
		Ma = c1 * Ma;

		temp = 4.0 * N * N * (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy) / (W*H);
		if (N > temp)
			Mb = N;
		else
			Mb = temp;
		Mb = c2 * Mb;

		Ms = c3 * this->testedPointCount;
		if (Ms < Ma)
			Ms = Ma;
		if (Ms > Mb)
			Ms = Mb;

		//按边长分配Mo
		this->grid_res[0] = sqrt((float)Ms*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)Ms*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	//clamp the range  
	if (this->grid_res[0] == 0)		this->grid_res[0] = 1;
	if (this->grid_res[1] == 0)		this->grid_res[1] = 1;

	//创建网格
	this->grid = new Grid2D();

	this->grid->boundingBox[0].x = this->grid_boundingbox[0].x;//边界框左上角和右下角的坐标值
	this->grid->boundingBox[0].y = this->grid_boundingbox[0].y;
	this->grid->boundingBox[1].x = this->grid_boundingbox[1].x;
	this->grid->boundingBox[1].y = this->grid_boundingbox[1].y;

	this->grid->gridSize[0] = this->grid_size[0];//边界框的长与高
	this->grid->gridSize[1] = this->grid_size[1];

	this->grid->resolution[0] = this->grid_res[0];//网格分辨率
	this->grid->resolution[1] = this->grid_res[1];

	this->grid->cellCount = this->grid_res[0] * this->grid_res[1];//网格的数量，即网格分别率相乘 

	this->grid->cellSize[0] = (FLTYPE)this->grid->gridSize[0] / this->grid->resolution[0];//小网格的长与宽
	this->grid->cellSize[1] = (FLTYPE)this->grid->gridSize[1] / this->grid->resolution[1];

	this->grid->cell = new GridCell2D[this->grid->cellCount];     //创建网格单元

	//估算指针总数，预分配指针空间
	//this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], ESTIMATE_REF_STYLE_REDUNDANT);
	this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], this->estimatedRefMode);
	//size_t size = sizeof(struct EdgeRef2D) * this->grid->expectRefCount;
	//this->grid->edgeRef = (struct EdgeRef2D *)malloc(size);   //动态内存分配，用于申请一块连续的指定大小的内存块区域以void*类型返回分配的内存区域地址
	//memset(this->grid->edgeRef, 0, size);   //memset:作用是在一段内存块中填充某个给定的值

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//图学学报 start
	xmin = grid_boundingbox[0].x;
	SizeX = this->grid->cellSize[0];
	ymin = grid_boundingbox[0].y;
	SizeY = this->grid->cellSize[1];

	size_t gcpsize = (this->grid_res[0] + 1) * (this->grid_res[1] + 1);//网格交点的个数
	this->RgpPoint = new RGPCell2D[gcpsize];

	//计算存储X轴上交点的片段数，横轴数量*（纵轴数量-1）最上边和最底边不可能和多边形边有交点
	size_t gcpXSegmentsize = this->grid_res[0] * (this->grid_res[1] - 1);
	//计算存储Y轴上交点的片段数，纵轴数量*（横轴数量-1）最左边和最右边不可能和多边形边有交点
	size_t gcpYSegmentsize = (this->grid_res[0] - 1) * this->grid_res[1];
	//创建存储多边形边与网格线交点的  片段单元

	this->RgpIntersectPointX = new RgpPointInSegment[gcpXSegmentsize];
	this->RgpIntersectPointY = new RgpPointInSegment[gcpYSegmentsize];

	//估算指针总数，预分配存储多边形边片段的指针空间
	//size的大小需要重新估计，目前利用上述估计的结果
	size_t size_Rgp = sizeof(struct RGPEdgeRef2D) * this->grid->expectRefCount * 2;
	this->grid->RgpEdgeRef = (struct RGPEdgeRef2D *)malloc(size_Rgp);
	memset(this->grid->RgpEdgeRef, 0, size_Rgp);

	//估算指针总数，预分配存储多边形边与网格线交点的指针空间
	/*this->grid->RgpPoint = (struct RgpPointInSegment*)malloc(size);
	memset(this->grid->RgpPoint,0,size);*/
	//图学学报 end
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//创建单元指针列表
	//遍历每条边，计算各个边占据的网格单元，将边指针加入相关单元
	if (this->addEdgeMethod == ADD_EDGE_DDA_ONCE)
		this->addEdgeRef_2DDDA();  //GCP 2012 软件学报
	if (this->addEdgeMethod == ADD_EDGE_DDA_RGP)
		this->Rgp_addEdgeRef_2DDDA();//图学学报
	else if (this->addEdgeMethod == ADD_EDGE_DDA_DEMAND)
		this->addEdgeRef_2DDDA_2();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_ONCE)
		this->addEdgeRef_Bresenham();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_DEMAND)
		this->addEdgeRef_Bresenham_2();

	//计算网格单元中点处的点包信息，设置单元属性（纯内、纯外、混合）
	SmallTimer tm;
	tm.start();
	if (this->cpTransMode == CP_TRANS_MODE_LEFT2RIGHT)
	{
		setCellProp();
	}
	else if (this->cpTransMode == CP_TRANS_MODE_RGP)
	{
		this->RgpsetCellIntersectProp();//设置网格交点位置属性
		this->SetCellMiddlePointCoord();
		this->G_RgpMarkCells();
	}
	else if (this->cpTransMode == CP_TRANS_MODE_SPIRAL)
		setCellProp_spiral();
	else if (this->cpTransMode == CP_TRANS_MODE_ZIGZAG)
		setCellProp_zigzag();
	else if (this->cpTransMode == CP_TRANS_MODE_CIRCUITOUS)
		setCellProp_circuitous();
	tm.end();
	this->stat.setCellPropTime = tm.time;

	return 1;
}

//O-GP method  based  on edge orientation
int GridPIP2D::OgpcreateGrid()
{
	if (this->grid != NULL)
	{
		delete this->grid;
		this->grid = NULL;
	}
	this->asType = AS_TYPE_GRID;     //  0  use grid

	//计算网格大小。为计算方便比多边形包围盒稍大(0.01%）
	FLTYPE enlarge[2];
	enlarge[0] = (this->testedPolygon[curPolygonIndex].boundingBox[1].x - this->testedPolygon[curPolygonIndex].boundingBox[0].x) * 0.01;//源程序该处为0.001
	enlarge[1] = (this->testedPolygon[curPolygonIndex].boundingBox[1].y - this->testedPolygon[curPolygonIndex].boundingBox[0].y) * 0.01;//同上

	this->grid_boundingbox[0].x = this->testedPolygon[curPolygonIndex].boundingBox[0].x - enlarge[0];
	this->grid_boundingbox[0].y = this->testedPolygon[curPolygonIndex].boundingBox[0].y - enlarge[1];
	this->grid_boundingbox[1].x = this->testedPolygon[curPolygonIndex].boundingBox[1].x + enlarge[0];
	this->grid_boundingbox[1].y = this->testedPolygon[curPolygonIndex].boundingBox[1].y + enlarge[1];

	this->grid_size[0] = this->grid_boundingbox[1].x - this->grid_boundingbox[0].x;            //边界框的尺寸大小
	this->grid_size[1] = this->grid_boundingbox[1].y - this->grid_boundingbox[0].y;

	//计算网格单元的长宽分配比例
	//this->cell_size_ratio = this->grid_size[0]*this->testedPolygon->dy / (this->grid_size[1]*this->testedPolygon->dx);

	//计算网格分辨率
	if (this->gridResMode == GRID_RES_TYPE_CLASSIC)  //1  经典方法计算分辨率
	{
		/*
		this->grid_res[0] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[0] / this->grid_size[1] ;
		this->grid_res[0] = this->grid_res[0] << 1 ;
		this->grid_res[1] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[1] / this->grid_size[0] ;
		this->grid_res[1] = this->grid_res[1] << 1 ;
		*/
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		//原方法的计算方式
		/*this->grid_res[0] = sqrt((float)K*W / H);
		this->grid_res[1] = sqrt((float)K*H / W);*/

		this->grid_res[0] = 2 * (1 + (int)(sqrt((float)K)*W / H));
		this->grid_res[1] = 2 * (1 + (int)(sqrt((float)K)*H / W));

		if (this->grid_res[0] > 100)     this->grid_res[0] = 100;
		if (this->grid_res[1] > 100)		 this->grid_res[1] = 100;

		printf("Grid resolution X=%d\n", this->grid_res[0]);
		printf("Grid resolution Y=%d\n", this->grid_res[1]);
	}
	else if (this->gridResMode == GRID_RES_TYPE_EDGELENGTH)
	{
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		this->grid_res[0] = sqrt((float)K*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)K*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	else if (this->gridResMode == GRID_RES_TYPE_OPTIMAL)    //最佳分辨率Ma,Mb,Ms
	{
		//计算Ma, Mb, Ms和Mo
		int Ma, Mb, Ms, Mo;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		int N = this->testedPolygon[curPolygonIndex].vertexCount;
		float c1 = 4.0;
		float c2 = 4.0;
		float c3 = 1.0;

		float temp = 0.25 * (W * H) / (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy);
		if (N < temp)
			Ma = N;
		else
			Ma = temp;
		Ma = c1 * Ma;

		temp = 4.0 * N * N * (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy) / (W*H);
		if (N > temp)
			Mb = N;
		else
			Mb = temp;
		Mb = c2 * Mb;

		Ms = c3 * this->testedPointCount;
		if (Ms < Ma)
			Ms = Ma;
		if (Ms > Mb)
			Ms = Mb;

		//按边长分配Mo
		this->grid_res[0] = sqrt((float)Ms*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)Ms*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	//clamp the range  
	if (this->grid_res[0] == 0)		this->grid_res[0] = 1;
	if (this->grid_res[1] == 0)		this->grid_res[1] = 1;

	//创建网格
	this->grid = new Grid2D();

	this->grid->boundingBox[0].x = this->grid_boundingbox[0].x;//边界框左上角和右下角的坐标值
	this->grid->boundingBox[0].y = this->grid_boundingbox[0].y;
	this->grid->boundingBox[1].x = this->grid_boundingbox[1].x;
	this->grid->boundingBox[1].y = this->grid_boundingbox[1].y;

	this->grid->gridSize[0] = this->grid_size[0];//边界框的长与高
	this->grid->gridSize[1] = this->grid_size[1];

	this->grid->resolution[0] = this->grid_res[0];//网格分辨率
	this->grid->resolution[1] = this->grid_res[1];

	this->grid->cellCount = this->grid_res[0] * this->grid_res[1];//网格的数量，即网格分别率相乘 

	this->grid->cellSize[0] = (FLTYPE)this->grid->gridSize[0] / this->grid->resolution[0];//小网格的长与宽
	this->grid->cellSize[1] = (FLTYPE)this->grid->gridSize[1] / this->grid->resolution[1];

	this->grid->cell = new GridCell2D[this->grid->cellCount];     //创建网格单元

	//估算指针总数，预分配指针空间
	this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], this->estimatedRefMode);
	//size_t size = sizeof(struct EdgeRef2D) * this->grid->expectRefCount;
	//this->grid->edgeRef = (struct EdgeRef2D *)malloc(size);   //动态内存分配，用于申请一块连续的指定大小的内存块区域以void*类型返回分配的内存区域地址
	//memset(this->grid->edgeRef, 0, size);   //memset:作用是在一段内存块中填充某个给定的值


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//图学学报 start
	xmin = grid_boundingbox[0].x;
	SizeX = this->grid->cellSize[0];
	ymin = grid_boundingbox[0].y;
	SizeY = this->grid->cellSize[1];

	size_t gcpsize = (this->grid_res[0] + 1) * (this->grid_res[1] + 1);//网格交点的个数
	this->RgpPoint = new RGPCell2D[gcpsize];

	//计算存储X轴上交点的片段数，横轴数量*（纵轴数量-1）最上边和最底边不可能和多边形边有交点
	size_t gcpXSegmentsize = this->grid_res[0] * (this->grid_res[1] - 1);

	//计算存储Y轴上交点的片段数，纵轴数量*（横轴数量-1）最左边和最右边不可能和多边形边有交点
	size_t gcpYSegmentsize = (this->grid_res[0] - 1) * this->grid_res[1];
	
	//创建存储多边形边与网格线交点的片段单元
	this->RgpIntersectPointX = new RgpPointInSegment[gcpXSegmentsize];
	this->RgpIntersectPointY = new RgpPointInSegment[gcpYSegmentsize];

	//估算指针总数，预分配存储多边形边片段的指针空间
	//size的大小需要重新估计，目前利用上述估计的结果
	size_t size_Rgp = sizeof(struct RGPEdgeRef2D) * this->grid->expectRefCount * 2;
	this->grid->RgpEdgeRef = (struct RGPEdgeRef2D *)malloc(size_Rgp);
	memset(this->grid->RgpEdgeRef, 0, size_Rgp);

	//估算指针总数，预分配存储多边形边与网格线交点的指针空间
	/*this->grid->RgpPoint = (struct RgpPointInSegment*)malloc(size);
	memset(this->grid->RgpPoint,0,size);*/
	//图学学报 end
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//创建单元指针列表
	//遍历每条边，计算各个边占据的网格单元，将边指针加入相关单元
	switch (this->addEdgeMethod)
	{
	case ADD_EDGE_DDA_ONCE:
		this->addEdgeRef_2DDDA();  //GCP 2012 软件学报
		break;
	case ADD_EDGE_DDA_RGP:
		this->Rgp_addEdgeRef_2DDDA();//图学学报
		break;
	case ADD_EDGE_DDA_OGP:
		this->Ogp_addEdgeRef_2DDDA();//TVCG
		break;
	case ADD_EDGE_DDA_DEMAND:
		this->addEdgeRef_2DDDA_2();
		break;
	case ADD_EDGE_BRESENHAM_ONCE:
		this->addEdgeRef_Bresenham();
		break;
	case ADD_EDGE_BRESENHAM_DEMAND:
		this->addEdgeRef_Bresenham_2();
		break;
	default:
		cout << "addEdgeMethod paramater error!" << endl;
		break;
	}
	//下边注释部分转换为上述switch 语句
	//if (this->addEdgeMethod == ADD_EDGE_DDA_ONCE)
	//	this->addEdgeRef_2DDDA();  //GCP 2012 软件学报
	//if (this->addEdgeMethod == ADD_EDGE_DDA_RGP)
	//	this->Rgp_addEdgeRef_2DDDA();//图学学报
	//else if (this->addEdgeMethod == ADD_EDGE_DDA_DEMAND)
	//	this->addEdgeRef_2DDDA_2();
	//else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_ONCE)
	//	this->addEdgeRef_Bresenham();
	//else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_DEMAND)
	//	this->addEdgeRef_Bresenham_2();

	//计算网格单元中点处的点包信息，设置单元属性（纯内、纯外、混合）
	SmallTimer tm;
	tm.start();
	switch (this->cpTransMode)
	{
	case CP_TRANS_MODE_LEFT2RIGHT:
		setCellProp();
		break;
	case CP_TRANS_MODE_RGP:
		this->RgpsetCellIntersectProp();//设置网格交点位置属性
		this->SetCellMiddlePointCoord();
		this->RgpMarkCells();
		break;
	case CP_TRANS_MODE_OGP:
		this->OgpsetCellIntersectProp();//设置网格交点位置属性
		this->SetCellMiddlePointCoord();
		this->RgpMarkCells();
		break;
	case CP_TRANS_MODE_SPIRAL:
		setCellProp_spiral();
		break;
	case CP_TRANS_MODE_ZIGZAG:
		setCellProp_zigzag();
		break;
	case CP_TRANS_MODE_CIRCUITOUS:
		setCellProp_circuitous();
		break;
	default:
		break;
	}
	//下述注释的代码转换为switch 语句
	//if (this->cpTransMode == CP_TRANS_MODE_LEFT2RIGHT)
	//{
	//	setCellProp();
	//}
	//else if (this->cpTransMode == CP_TRANS_MODE_RGP)
	//{
	//	this->setCellIntersectProp();//设置网格交点位置属性
	//	this->SetCellMiddlePointCoord();
	//	this->RgpMarkCells();
	//}
	//else if (this->cpTransMode == CP_TRANS_MODE_SPIRAL)
	//	setCellProp_spiral();
	//else if (this->cpTransMode == CP_TRANS_MODE_ZIGZAG)
	//	setCellProp_zigzag();
	//else if (this->cpTransMode == CP_TRANS_MODE_CIRCUITOUS)
	//	setCellProp_circuitous();

	tm.end();
	this->stat.setCellPropTime = tm.time;

	return 1;
}//OgpcreateGrid

//Create grid   GCP
int GridPIP2D::createGrid()
{
	if (this->grid != NULL)
	{
		delete this->grid;
		this->grid = NULL;
	}
	this->asType = AS_TYPE_GRID;     //  0  use grid

	//计算网格大小。为计算方便比多边形包围盒稍大(0.1%）
	FLTYPE enlarge[2];
  	enlarge[0] = (this->testedPolygon[curPolygonIndex].boundingBox[1].x - this->testedPolygon[curPolygonIndex].boundingBox[0].x) * 0.01;//源程序该处为0.001
	enlarge[1] = (this->testedPolygon[curPolygonIndex].boundingBox[1].y - this->testedPolygon[curPolygonIndex].boundingBox[0].y) * 0.01;//同上

	this->grid_boundingbox[0].x = this->testedPolygon[curPolygonIndex].boundingBox[0].x - enlarge[0];
	this->grid_boundingbox[0].y = this->testedPolygon[curPolygonIndex].boundingBox[0].y - enlarge[1];
	this->grid_boundingbox[1].x = this->testedPolygon[curPolygonIndex].boundingBox[1].x + enlarge[0];
	this->grid_boundingbox[1].y = this->testedPolygon[curPolygonIndex].boundingBox[1].y + enlarge[1];

	this->grid_size[0] = this->grid_boundingbox[1].x - this->grid_boundingbox[0].x;            //边界框的尺寸大小
	this->grid_size[1] = this->grid_boundingbox[1].y - this->grid_boundingbox[0].y;

	//计算网格单元的长宽分配比例
	//this->cell_size_ratio = this->grid_size[0]*this->testedPolygon->dy / (this->grid_size[1]*this->testedPolygon->dx);

	//计算网格分辨率
	if (this->gridResMode == GRID_RES_TYPE_CLASSIC)  //1  经典方法计算分辨率
	{
		/*
		this->grid_res[0] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[0] / this->grid_size[1] ;
		this->grid_res[0] = this->grid_res[0] << 1 ;
		this->grid_res[1] = sqrt((FLTYPE)this->testedPolygon[curPolygonIndex].edgeCount) * this->grid_size[1] / this->grid_size[0] ;
		this->grid_res[1] = this->grid_res[1] << 1 ;
		*/
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		//原方法的计算方式
		/*this->grid_res[0] = sqrt((float)K*W / H);
		this->grid_res[1] = sqrt((float)K*H / W);*/


		//修改后代码(图学学报)
		this->grid_res[0] = 2 * (1 + (int)(sqrt((float)K)*W / H));
		this->grid_res[1] = 2 * (1 + (int)(sqrt((float)K)*H / W));
		//图学学报，新添加代码
		if (this->grid_res[0] > 100)     this->grid_res[0] = 100;
		if (this->grid_res[1] > 100)		 this->grid_res[1] = 100;
		//添加完毕
		printf("Grid resolution X=%d\n", this->grid_res[0]);
		printf("Grid resolution Y=%d\n", this->grid_res[1]);
		//当前修改完毕
	}
	else if (this->gridResMode == GRID_RES_TYPE_EDGELENGTH)
	{
		int K = this->testedPolygon[curPolygonIndex].edgeCount;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		this->grid_res[0] = sqrt((float)K*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)K*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}
	else if (this->gridResMode == GRID_RES_TYPE_OPTIMAL)    //最佳分辨率Ma,Mb,Ms
	{
		//计算Ma, Mb, Ms和Mo
		int Ma, Mb, Ms, Mo;
		float W = this->grid_size[0];
		float H = this->grid_size[1];
		int N = this->testedPolygon[curPolygonIndex].vertexCount;
		float c1 = 4.0;
		float c2 = 4.0;
		float c3 = 1.0;

		float temp = 0.25 * (W * H) / (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy);
		if (N < temp)
			Ma = N;
		else
			Ma = temp;
		Ma = c1 * Ma;

		temp = 4.0 * N * N * (this->testedPolygon[curPolygonIndex].dx * this->testedPolygon[curPolygonIndex].dy) / (W*H);
		if (N > temp)
			Mb = N;
		else
			Mb = temp;
		Mb = c2 * Mb;

		Ms = c3 * this->testedPointCount;
		if (Ms < Ma)
			Ms = Ma;
		if (Ms > Mb)
			Ms = Mb;

		//按边长分配Mo
		this->grid_res[0] = sqrt((float)Ms*(W*this->testedPolygon[curPolygonIndex].dy) / (H*this->testedPolygon[curPolygonIndex].dx));
		this->grid_res[1] = sqrt((float)Ms*(H*this->testedPolygon[curPolygonIndex].dx) / (W*this->testedPolygon[curPolygonIndex].dy));
	}

	//clamp the range  
	if (this->grid_res[0] == 0)		this->grid_res[0] = 1;
	if (this->grid_res[1] == 0)		this->grid_res[1] = 1;

	//创建网格
  	this->grid = new Grid2D();

	this->grid->boundingBox[0].x = this->grid_boundingbox[0].x;//边界框左上角和右下角的坐标值
	this->grid->boundingBox[0].y = this->grid_boundingbox[0].y;
	this->grid->boundingBox[1].x = this->grid_boundingbox[1].x;
	this->grid->boundingBox[1].y = this->grid_boundingbox[1].y;

	this->grid->gridSize[0] = this->grid_size[0];//边界框的长与高
	this->grid->gridSize[1] = this->grid_size[1];

	this->grid->resolution[0] = this->grid_res[0];//网格分辨率
	this->grid->resolution[1] = this->grid_res[1];

	this->grid->cellCount = this->grid_res[0] * this->grid_res[1];//网格的数量，即网格分别率相乘 

	this->grid->cellSize[0] = (FLTYPE)this->grid->gridSize[0] / this->grid->resolution[0];//小网格的长与宽
	this->grid->cellSize[1] = (FLTYPE)this->grid->gridSize[1] / this->grid->resolution[1];

	this->grid->cell = new GridCell2D[this->grid->cellCount];     //创建网格单元

	//估算指针总数，预分配指针空间
	//this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], ESTIMATE_REF_STYLE_REDUNDANT);
	this->grid->expectRefCount = estimateRef(this->grid_res[0], this->grid_res[1], this->estimatedRefMode);
	size_t size = sizeof(struct EdgeRef2D) * this->grid->expectRefCount;
	this->grid->edgeRef = (struct EdgeRef2D *)malloc(size);   //动态内存分配，用于申请一块连续的指定大小的内存块区域以void*类型返回分配的内存区域地址
	memset(this->grid->edgeRef, 0, size);   //memset:作用是在一段内存块中填充某个给定的值


	//创建单元指针列表
	//遍历每条边，计算各个边占据的网格单元，将边指针加入相关单元
	if (this->addEdgeMethod == ADD_EDGE_DDA_ONCE)
		this->addEdgeRef_2DDDA();  //GCP 2012 软件学报
	else if (this->addEdgeMethod == ADD_EDGE_DDA_RGP)
		this->Rgp_addEdgeRef_2DDDA();//图学学报
	else if (this->addEdgeMethod == ADD_EDGE_DDA_DEMAND)
		this->addEdgeRef_2DDDA_2();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_ONCE)
		this->addEdgeRef_Bresenham();
	else if (this->addEdgeMethod == ADD_EDGE_BRESENHAM_DEMAND)
		this->addEdgeRef_Bresenham_2();

	//计算网格单元中点处的点包信息，设置单元属性（纯内、纯外、混合）
	SmallTimer tm;
	tm.start();
	if (this->cpTransMode == CP_TRANS_MODE_LEFT2RIGHT)   //GCP method
		setCellProp();
	else if(this->cpTransMode==CP_TRANS_MODE_RGP)  //RGP method
	{
		this->RgpsetCellIntersectProp();//设置网格交点位置属性
		this->SetCellMiddlePointCoord();
		this->RgpMarkCells();
	}
	else if (this->cpTransMode == CP_TRANS_MODE_SPIRAL)
		setCellProp_spiral();
	else if (this->cpTransMode == CP_TRANS_MODE_ZIGZAG)
		setCellProp_zigzag();
	else if (this->cpTransMode == CP_TRANS_MODE_CIRCUITOUS)
		setCellProp_circuitous();
	tm.end();
	this->stat.setCellPropTime = tm.time;

	return 1;
} //createGrid


//通过记录并使用中心点与边的位置关系来加快求交判定
bool GridPIP2D::isIntersect(Point2D*p0, Point2D * p1, Point2D * p2, Point2D * p3, DynamicArray & da, int edge_index, bool flag)
{
	//p2是否在p0p1的左侧
	int p2_side;
	FLTYPE temp;
	Point2D p2p0, p1p0;
	p1p0.x = p1->x - p0->x;
	p1p0.y = p1->y - p0->y;
	if (flag == 0)	//使用关于p2的位置记录
		p2_side = da[edge_index];
	else
	{
		p2p0.x = p2->x - p0->x;
		p2p0.y = p2->y - p0->y;
		temp = p2p0.x*p1p0.y - p2p0.y*p1p0.x;
		if (temp < 0)
			p2_side = 1;	//左侧
		else
			p2_side = 0;	//右侧
	}

	//p3是否在p0p1的左侧
	Point2D p3p0;
	int p3_side;
	p3p0.x = p3->x - p0->x;
	p3p0.y = p3->y - p0->y;
	temp = p3p0.x*p1p0.y - p3p0.y*p1p0.x;
	if (temp < 0)
		p3_side = 1;	//左侧
	else
		p3_side = 0;	//右侧
	if (flag == 1)	//记录关于p3的位置记录
		da[edge_index] = p3_side;

	//p0是否位于p2p3的两侧
	Point2D p3p2, p0p2;
	int p0_side;
	p3p2.x = p3->x - p2->x;
	p3p2.y = p3->y - p2->y;
	p0p2.x = p0->x - p2->x;
	p0p2.y = p0->y - p2->y;
	temp = p0p2.x*p3p2.y - p0p2.y*p3p2.x;
	if (temp < 0)
		p0_side = 1;	//左侧
	else
		p0_side = 0;	//右侧

	//p1是否位于p3p2的两侧
	Point2D p1p2;
	int p1_side;
	p1p2.x = p1->x - p2->x;
	p1p2.y = p1->y - p2->y;
	temp = p1p2.x*p3p2.y - p1p2.y*p3p2.x;
	if (temp < 0)
		p1_side = 1;	//左侧
	else
		p1_side = 0;	//右侧

	//p2,p3同在左侧，无交；否则有交
	if ((p2_side != p3_side) && (p0_side != p1_side))
		return true;
	else
		return false;
} //isIntersect

//判断p0p1和p2p3是否相交
bool GridPIP2D::isIntersect(Point2D * p0, Point2D * p1, Point2D * p2, Point2D *p3)
{
	//p2是否在p0p1的左侧
	Point2D p2p0, p1p0;
	FLTYPE temp;
	int p2_side;
	p2p0.x = p2->x - p0->x;
	p2p0.y = p2->y - p0->y;
	p1p0.x = p1->x - p0->x;
	p1p0.y = p1->y - p0->y;
	temp = p2p0.x*p1p0.y - p2p0.y*p1p0.x;
	if (temp < 0)
		p2_side = 1;	//左侧
	else
		p2_side = 0;	//右侧

	//p3是否在p0p1的左侧
	Point2D p3p0;
	int p3_side;
	p3p0.x = p3->x - p0->x;
	p3p0.y = p3->y - p0->y;
	temp = p3p0.x*p1p0.y - p3p0.y*p1p0.x;
	if (temp < 0)
		p3_side = 1;	//左侧
	else
		p3_side = 0;	//右侧

	//p0是否位于p2p3的两侧
	Point2D p3p2, p0p2;
	int p0_side;
	p3p2.x = p3->x - p2->x;
	p3p2.y = p3->y - p2->y;
	p0p2.x = p0->x - p2->x;
	p0p2.y = p0->y - p2->y;
	temp = p0p2.x*p3p2.y - p0p2.y*p3p2.x;
	if (temp < 0)
		p0_side = 1;	//左侧
	else
		p0_side = 0;	//右侧

	//p1是否位于p3p2的两侧
	Point2D p1p2;
	int p1_side;
	p1p2.x = p1->x - p2->x;
	p1p2.y = p1->y - p2->y;
	temp = p1p2.x*p3p2.y - p1p2.y*p3p2.x;
	if (temp < 0)
		p1_side = 1;	//左侧
	else
		p1_side = 0;	//右侧

	//p2,p3同在左侧，无交；否则有交
	if ((p2_side != p3_side) && (p0_side != p1_side))
		return true;
	else
		return false;
}



//创建单元指针列表
//遍历每条边，以2D DDA算法计算各个边占据的网格单元，将边指针加入相关单元
void GridPIP2D::Rgp_addEdgeRef_2DDDA()
{
	//struct EdgeRef2D * cur_location = this->grid->edgeRef;
	struct RGPEdgeRef2D *cur_segment = this->grid->RgpEdgeRef;//图学学报
	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];     //边的起始点
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];        //边的终点

		//图学学报 start
		//计算斜率
		Line2D* line = NULL;
		line = new Line2D(start_vertex, end_vertex);
		double k = line->CalculateSlope();
		double B;
		if (k != PI_HALF)
		{
			B = start_vertex->y - k * start_vertex->x;
		}
		else
		{
			B = MAX_DOUBLE;//垂直X轴 
		}

		int mark0 = 0;//wang add
		int segment_num = 0;
		MySortOfPoint *mySP0;
		mySP0 = new MySortOfPoint[100];
		RgpGetIntersectionPoint(start_vertex, end_vertex, mySP0);//计算多边形边与网格线的交点
		Point2D *start_segment_edge = &mySP0[segment_num++].a;
		Point2D *end_segment_edge = &mySP0[segment_num].a;

		
		//图学学报 end

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引 即从左至右，从上至下，的第多少个
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);    //获取起始点所在的网格
		//cur_location->e = cur_edge;

		//图学学报 start
		
		cur_segment->start_e = start_segment_edge;
		cur_segment->end_e = end_segment_edge;
		cur_segment->k = k;
		cur_segment->B = B;
		cur_segment->next = this->grid->cell[start_index[2]].RgpEdgeRef;//头插法
		this->grid->cell[start_index[2]].RgpEdgeRef = cur_segment;
		this->grid->cell[start_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
		this->grid->cell[start_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
		cur_segment++;
		//图学学报 end

		/*cur_location->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = cur_location;
		this->grid->cell[start_index[2]].edgeCount++;

		cur_location++;*/

		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);  //获取终点所在的网格
		if (end_index[2] == start_index[2])  //整条边在一个单元内
			continue;
		//cur_location->e = cur_edge;
		//cur_location->next = this->grid->cell[end_index[2]].edgeRef;
		//this->grid->cell[end_index[2]].edgeRef = cur_location;
		//this->grid->cell[end_index[2]].edgeCount++;
		//cur_location++;

		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		FLTYPE start_coord[2];
 		int signx = 1;
		int signy = 1;
		if (end_vertex->x > start_vertex->x) //向右
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		else if (end_vertex->x < start_vertex->x)//向左
		{
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
			signx = -1;
		}
		else //x坐标相等
		{
			start_coord[0] = start_vertex->x;
			signx = 0;
		}

		if (end_vertex->y > start_vertex->y) //向上
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		else if (end_vertex->y < start_vertex->y)//向下
		{
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
			signy = -1;
		}
		else //y坐标相等
		{
			start_coord[1] = start_vertex->y;
			signy = 0;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(end_vertex->x - start_vertex->x);
		y_length = abs(end_vertex->y - start_vertex->y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->grid->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->grid->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - start_vertex->x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - start_vertex->y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {                     //x方向移动
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}
			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
			if (cur_index[2] == end_index[2])		//截止退出
				break;
			//cur_location->e = cur_edge;
			//cur_location->next = this->grid->cell[cur_index[2]].edgeRef;
			//this->grid->cell[cur_index[2]].edgeRef = cur_location;
			//this->grid->cell[cur_index[2]].edgeCount++;
			//cur_location++;

			//图学学报 start
			start_segment_edge = &mySP0[segment_num++].a;
			end_segment_edge = &mySP0[segment_num].a;
			cur_segment->start_e = start_segment_edge;
			cur_segment->end_e = end_segment_edge;
			cur_segment->k = k;
			cur_segment->B = B;

			cur_segment->next = this->grid->cell[cur_index[2]].RgpEdgeRef;
			this->grid->cell[cur_index[2]].RgpEdgeRef = cur_segment;
			this->grid->cell[cur_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
			this->grid->cell[cur_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
			cur_segment++;
			//图学学报 end
		} //while

		//图学学报 start
		cur_segment->start_e = &mySP0[segment_num++].a;
		cur_segment->end_e = &mySP0[segment_num].a;
		cur_segment->k = k;
		cur_segment->B = B;
		cur_segment->next = this->grid->cell[end_index[2]].RgpEdgeRef;
		this->grid->cell[end_index[2]].RgpEdgeRef = cur_segment;
		this->grid->cell[end_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
		this->grid->cell[end_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
		cur_segment++;
		//图学学报 end
	} //for	

	//记录真实指针数
	//this->stat.realRefCount = cur_location - this->grid->edgeRef + 1;
}

//OGP method  based on orientation
void GridPIP2D::Ogp_addEdgeRef_2DDDA()
{
	//struct EdgeRef2D * cur_location = this->grid->edgeRef;
	struct RGPEdgeRef2D *cur_segment = this->grid->RgpEdgeRef;//图学学报
	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];     //边的起始点
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];        //边的终点

		//图学学报 start
		//计算斜率
		Line2D* line = NULL;
		line = new Line2D(start_vertex, end_vertex);
		double k = line->CalculateSlope();
		double B;
		if (k != PI_HALF)
		{
			B = start_vertex->y - k * start_vertex->x;
		}
		else
		{
			B = MAX_DOUBLE;//垂直X轴 
		}
		int mark0 = 0;//wang add
		int segment_num = 0;
		MySortOfPoint *mySP0;
		mySP0 = new MySortOfPoint[100];
		RgpGetIntersectionPoint(start_vertex, end_vertex, mySP0);//计算多边形边与网格线的交点
		Point2D *start_segment_edge = &mySP0[segment_num++].a;
		Point2D *end_segment_edge = &mySP0[segment_num].a;
		//图学学报 end

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引 即从左至右，从上至下，的第多少个
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);    //获取起始点所在的网格
		//cur_location->e = cur_edge;

		//图学学报 start
		cur_segment->start_e = start_segment_edge;
		cur_segment->end_e = end_segment_edge;
		cur_segment->k = k;
		cur_segment->B = B;
		cur_segment->next = this->grid->cell[start_index[2]].RgpEdgeRef;//头插法
		this->grid->cell[start_index[2]].RgpEdgeRef = cur_segment;
		this->grid->cell[start_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
		this->grid->cell[start_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
		cur_segment++;
		//图学学报 end

		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);  //获取终点所在的网格
		if (end_index[2] == start_index[2])  //整条边在一个单元内
			continue;

		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		FLTYPE start_coord[2];
		int signx = 1;
		int signy = 1;
		if (end_vertex->x > start_vertex->x) //向右
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		else if (end_vertex->x < start_vertex->x)//向左
		{
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
			signx = -1;
		}
		else //x坐标相等
		{
			start_coord[0] = start_vertex->x;
			signx = 0;
		}

		if (end_vertex->y > start_vertex->y) //向上
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		else if (end_vertex->y < start_vertex->y)//向下
		{
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
			signy = -1;
		}
		else //y坐标相等
		{
			start_coord[1] = start_vertex->y;
			signy = 0;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(end_vertex->x - start_vertex->x);
		y_length = abs(end_vertex->y - start_vertex->y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->grid->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->grid->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - start_vertex->x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - start_vertex->y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {                     //x方向移动
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}
			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
			if (cur_index[2] == end_index[2])		//截止退出
				break;

			//图学学报 start
			start_segment_edge = &mySP0[segment_num++].a;
			end_segment_edge = &mySP0[segment_num].a;
			cur_segment->start_e = start_segment_edge;
			cur_segment->end_e = end_segment_edge;
			cur_segment->k = k;
			cur_segment->B = B;

			cur_segment->next = this->grid->cell[cur_index[2]].RgpEdgeRef;
			this->grid->cell[cur_index[2]].RgpEdgeRef = cur_segment;
			this->grid->cell[cur_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
			this->grid->cell[cur_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
			cur_segment++;
			//图学学报 end
		} //while

		//图学学报 start
		cur_segment->start_e = &mySP0[segment_num++].a;
		cur_segment->end_e = &mySP0[segment_num].a;
		cur_segment->k = k;
		cur_segment->B = B;
		cur_segment->next = this->grid->cell[end_index[2]].RgpEdgeRef;
		this->grid->cell[end_index[2]].RgpEdgeRef = cur_segment;
		this->grid->cell[end_index[2]].cell_InofOut = CELL_SUSPECT;   //设置包含边的网格为CELL_SUSPECT
		this->grid->cell[end_index[2]].edgeCount++;//该步需要注释，和下列的重复操作
		cur_segment++;
		//图学学报 end
	} //for	
	//记录真实指针数
	//this->stat.realRefCount = cur_location - this->grid->edgeRef + 1;
}

//创建单元指针列表
//遍历每条边，以2D DDA算法计算各个边占据的网格单元，将边指针加入相关单元
void GridPIP2D::addEdgeRef_2DDDA()
{
	struct EdgeRef2D * cur_location = this->grid->edgeRef;
	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);
		cur_location->e = cur_edge;
		cur_location->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = cur_location;
		this->grid->cell[start_index[2]].edgeCount++;

		cur_location++;

		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);
		if (end_index[2] == start_index[2])  //整条边在一个单元内
			continue;
		cur_location->e = cur_edge;
		cur_location->next = this->grid->cell[end_index[2]].edgeRef;
		this->grid->cell[end_index[2]].edgeRef = cur_location;
		this->grid->cell[end_index[2]].edgeCount++;
		cur_location++;

		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		//x=x0+x1*t_x
		//y=y0+y1*t_y
		//FLTYPE start_coord[2];
		//int signx = 1;
		//int signy = 1;
		//if (end_vertex->x > start_vertex->x) //向右
		//	start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		//else //向左
		//{
		//	start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
		//	signx = -1;
		//}

		//if (end_vertex->y > start_vertex->y) //向上
		//	start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		//else //向下
		//{
		//	start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
		//	signy = -1;
		//}

		//上述部分替换为
		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		FLTYPE start_coord[2];
		int signx = 1;
		int signy = 1;
		if (end_vertex->x > start_vertex->x) //向右
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		else if (end_vertex->x < start_vertex->x)//向左
		{
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
			signx = -1;
		}
		else //x坐标相等
		{
			start_coord[0] = start_vertex->x;
			signx = 0;
		}

		if (end_vertex->y > start_vertex->y) //向上
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		else if (end_vertex->y < start_vertex->y)//向下
		{
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
			signy = -1;
		}
		else //y坐标相等
		{
			start_coord[1] = start_vertex->y;
			signy = 0;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(end_vertex->x - start_vertex->x);
		y_length = abs(end_vertex->y - start_vertex->y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->grid->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->grid->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - start_vertex->x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - start_vertex->y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}
			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
			if (cur_index[2] == end_index[2])		//截止退出
				break;
			cur_location->e = cur_edge;
			cur_location->next = this->grid->cell[cur_index[2]].edgeRef;
			this->grid->cell[cur_index[2]].edgeRef = cur_location;
			this->grid->cell[cur_index[2]].edgeCount++;
			cur_location++;
		} //while
	} //for	

	//记录真实指针数
	this->stat.realRefCount = cur_location - this->grid->edgeRef + 1;
}

//comparison version - alloc edge reference memory one by one
void GridPIP2D::addEdgeRef_2DDDA_2()
{
	struct EdgeRef2D * new_ref = NULL;
	this->stat.realRefCount = 0;

	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);
		new_ref = new EdgeRef2D;
		new_ref->e = cur_edge;
		new_ref->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = new_ref;
		this->grid->cell[start_index[2]].edgeCount++;
		this->stat.realRefCount++;

		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);
		if (end_index[2] == start_index[2])  //整条边在一个单元内
			continue;
		new_ref = new EdgeRef2D;
		new_ref->e = cur_edge;
		new_ref->next = this->grid->cell[end_index[2]].edgeRef;
		this->grid->cell[end_index[2]].edgeRef = new_ref;
		this->grid->cell[end_index[2]].edgeCount++;
		this->stat.realRefCount++;

		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		//x=x0+x1*t_x
		//y=y0+y1*t_y
		FLTYPE start_coord[2];
		int signx = 1;
		int signy = 1;
		if (end_vertex->x > start_vertex->x) //向右
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		else //向左
		{
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
			signx = -1;
		}

		if (end_vertex->y > start_vertex->y) //向上
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		else //向下
		{
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
			signy = -1;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(end_vertex->x - start_vertex->x);
		y_length = abs(end_vertex->y - start_vertex->y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->grid->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->grid->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - start_vertex->x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - start_vertex->y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}
			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
			if (cur_index[2] == end_index[2])		//截止退出
				break;

			new_ref = new EdgeRef2D;
			new_ref->e = cur_edge;
			new_ref->next = this->grid->cell[cur_index[2]].edgeRef;
			this->grid->cell[cur_index[2]].edgeRef = new_ref;
			this->grid->cell[cur_index[2]].edgeCount++;
			this->stat.realRefCount++;
		} //while
	} //for	
}

//comparison version - Bresenham algorithm and Zalik's code based algorithm
//allocate the edge reference one by one
void GridPIP2D::addEdgeRef_Bresenham_2()
{
	struct EdgeRef2D * new_ref = NULL;
	this->stat.realRefCount = 0;

	FLTYPE cx, cy;
	cx = this->grid->cellSize[0];
	cy = this->grid->cellSize[1];

	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];

		int start_index[3], end_index[3]; //[3]为一维索引
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);
		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);

		//add current edge to the start cell
		new_ref = new EdgeRef2D;
		new_ref->e = cur_edge;
		new_ref->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = new_ref;
		this->grid->cell[start_index[2]].edgeCount++;
		this->stat.realRefCount++;

		//start vertex and end vertex locate at the same cell
		if (start_index[2] == end_index[2])
			continue;

		FLTYPE dx, dy, temp, p, another_p;
		int old_index[2], cur_index[3], new_index[3], s0, s1, r0, r1;

		dx = end_vertex->x - start_vertex->x;
		dy = end_vertex->y - start_vertex->y;

		//calculate current edge's direction region
		if (end_vertex->x > start_vertex->x)
			s0 = 1;
		else //向左
			s0 = -1;
		if (end_vertex->y > start_vertex->y)
			s1 = 1;
		else //向下
			s1 = -1;

		int dir;
		if (s0 == 1 && s1 == 1)		//quadrant I
		{
			if (dy*cx < dx*cy)
				dir = 0;
			else
				dir = 1;
		}
		else if (s0 == -1 && s1 == 1)	//quadrant II
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 3;
			else
				dir = 2;
		}
		else if (s0 == -1 && s1 == -1)	//quadrant III
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 4;
			else
				dir = 5;
		}
		else if (s0 == 1 && s1 == -1)	//quadrant IV
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 7;
			else
				dir = 6;
		}

		//calculate the initial value of p
		//p = p1_coefficient[dir] * cx*dy - cy*dx;
		FLTYPE x, y, left, right, top, bottom;
		left = this->grid->boundingBox[0].x + cx * start_index[0];
		right = this->grid->boundingBox[0].x + cx * (start_index[0] + 1);
		top = this->grid->boundingBox[0].y + cy * (start_index[1] + 1);
		bottom = this->grid->boundingBox[0].y + cy * start_index[1];
		if (dir == 0 || dir == 7)
		{
			y = (end_vertex->y - start_vertex->y) * (right - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
			p = dx * ((y - bottom) - (top - y));
		}
		else if (dir == 1 || dir == 2)
		{
			x = (end_vertex->x - start_vertex->x) * (top - start_vertex->y) / (end_vertex->y - start_vertex->y) + start_vertex->x;
			p = dy * ((x - left) - (right - x));
		}
		else if (dir == 3 || dir == 4)
		{
			y = (end_vertex->y - start_vertex->y) * (left - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
			p = dx * ((y - bottom) - (top - y));
		}
		else if (dir == 5 || dir == 6)
		{
			x = (end_vertex->x - start_vertex->x) * (bottom - start_vertex->y) / (end_vertex->y - start_vertex->y) + start_vertex->x;
			p = dy * ((x - left) - (right - x));
		}

		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		new_index[0] = 0;
		new_index[1] = 0;

		while (1)
		{
			old_index[0] = cur_index[0];
			old_index[1] = cur_index[1];

			switch (dir)
			{
			case 0:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 1;
					another_p = p + 2 * dy*cx;
					p = p + 2 * dy*cx - 2 * dx*cy;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += 0;
					another_p = p + 2 * dy*cx - 2 * dx*cy;
					p = p + 2 * dy*cx;
				}
				break;
			case 1:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy;
					p = p + 2 * dx*cy - 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 0;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy - 2 * dy*cx;
					p = p + 2 * dx*cy;
				}
				break;
			case 2:
				if (p >= 0)
				{
					cur_index[0] += 0;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy + 2 * dy*cx;
					p = p + 2 * dx*cy;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy;
					p = p + 2 * dx*cy + 2 * dy*cx;
				}
				break;
			case 3:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += 0;
					another_p = p - 2 * dy*cx - 2 * dx*cy;
					p = p - 2 * dy*cx;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 1;
					another_p = p - 2 * dy*cx;
					p = p - 2 * dy*cx - 2 * dx*cy;
				}
				break;
			case 4:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += -1;
					another_p = p - 2 * dy*cx;
					p = p - 2 * dy*cx + 2 * dx*cy;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 0;
					another_p = p - 2 * dy*cx + 2 * dx*cy;
					p = p - 2 * dy*cx;
				}
				break;
			case 5:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy;
					p = p - 2 * dx*cy + 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 0;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy + 2 * dy*cx;
					p = p - 2 * dx*cy;
				}
				break;
			case 6:
				if (p >= 0)
				{
					cur_index[0] += 0;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy - 2 * dy*cx;
					p = p - 2 * dx*cy;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy;
					p = p - 2 * dx*cy - 2 * dy*cx;
				}
				break;
			case 7:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 0;
					another_p = p + 2 * dx*cy + 2 * dy*cx;
					p = p + 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += -1;
					another_p = p + 2 * dy*cx;
					p = p + 2 * dx*cy + 2 * dy*cx;
				}
				break;
			}

			r0 = cur_index[0] - old_index[0];
			r1 = cur_index[1] - old_index[1];

			//find the missing cell from Ci to Cj
			FLTYPE ylcs, x, y, next;
			bool add_flag = false, correct_flag = false, vertical_flag = false;

			if (end_vertex->x == start_vertex->x)
				vertical_flag = true;

			if (r0 == 1 && r1 == 1)	//M==>RT
			{
				ylcs = this->grid->boundingBox[0].y + cy * cur_index[1];
				x = this->grid->boundingBox[0].x + cx * cur_index[0];
				//y = y0 + (x-xa)*(y1-y0)/(x1-x0)
				y = ((end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x)) + start_vertex->y;
				if (y > ylcs || vertical_flag == true) //add T
				{
					new_index[0] = cur_index[0] - 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y > (ylcs + cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y < ylcs)	//add R
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] - 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (next < ylcs)
						correct_flag = true;
				}
			}

			if (r0 == -1 && r1 == 1)	//M==>LT
			{
				ylcs = this->grid->boundingBox[0].y + cy * cur_index[1];
				x = this->grid->boundingBox[0].x + cx * (cur_index[0] + 1);
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y > ylcs || vertical_flag == true) //add T
				{
					new_index[0] = cur_index[0] + 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y > (ylcs + cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y < ylcs) //add L
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] - 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (y < ylcs)
						correct_flag = true;
				}
			}

			if (r0 == 1 && r1 == -1)	//M==>RB
			{
				ylcs = this->grid->boundingBox[0].y + cy * (cur_index[1] + 1);
				x = this->grid->boundingBox[0].x + cx * cur_index[0];
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y <= ylcs || vertical_flag == true) //add B
				{
					new_index[0] = cur_index[0] - 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y < (ylcs - cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y > ylcs) //add R
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] + 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (next > ylcs)
						correct_flag = true;
				}
			}

			if (r0 == -1 && r1 == -1)	//M==>LB
			{
				ylcs = this->grid->boundingBox[0].y + cy * (cur_index[1] + 1);
				x = this->grid->boundingBox[0].x + cx * (cur_index[0] + 1);
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y < ylcs || vertical_flag == true) //add B
				{
					new_index[0] = cur_index[0] + 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y < (ylcs - cy))
						correct_flag = true;
				}
				else if (y > ylcs) //add L
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] + 1;
					add_flag = true;

					next = y - dy / dx * cx;
					if (next > ylcs)
						correct_flag = true;
				}
			}

			//add edge reference to the current cell (Cj)
			if (correct_flag == false &&
				cur_index[0] >= 0 && cur_index[0] < this->grid->resolution[0] &&
				cur_index[1] >= 0 && cur_index[1] < this->grid->resolution[1])
			{
				cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
				new_ref = new EdgeRef2D;
				new_ref->e = cur_edge;
				new_ref->next = this->grid->cell[cur_index[2]].edgeRef;
				this->grid->cell[cur_index[2]].edgeRef = new_ref;
				this->grid->cell[cur_index[2]].edgeCount++;
				this->stat.realRefCount++;
			}
			else //correct_flag == true
			{
				cur_index[0] = new_index[0];
				cur_index[1] = new_index[1];
				p = another_p;
			}

			//add edge reference to the missing cell
			if (add_flag == true &&
				new_index[0] >= 0 && new_index[0] <= this->grid->resolution[0] - 1 &&
				new_index[1] >= 0 && new_index[1] <= this->grid->resolution[1] - 1)
			{
				new_index[2] = new_index[1] * this->grid->resolution[0] + new_index[0];
				new_ref = new EdgeRef2D;
				new_ref->e = cur_edge;
				new_ref->next = this->grid->cell[new_index[2]].edgeRef;
				this->grid->cell[new_index[2]].edgeRef = new_ref;
				this->grid->cell[new_index[2]].edgeCount++;
				this->stat.realRefCount++;
			}

			//termination criterion
			if (cur_index[0] == end_index[0] && cur_index[1] == end_index[1] ||
				add_flag == true && new_index[0] == end_index[0] && new_index[1] == end_index[1])
				break;
		}
	} //for i
}  //addEdgeRef_Bresenham


//comparison version - Bresenham algorithm and Zalik's code based algorithm
//allocate the edge references in one time
void GridPIP2D::addEdgeRef_Bresenham()
{
	struct EdgeRef2D * cur_location = this->grid->edgeRef;
	FLTYPE cx, cy;
	cx = this->grid->cellSize[0];
	cy = this->grid->cellSize[1];

	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];

		int start_index[3], end_index[3]; //[3]为一维索引
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);
		end_index[2] = this->grid->locatePoint(end_vertex->x, end_vertex->y, end_index[0], end_index[1]);

		//add current edge to the start cell
		cur_location->e = cur_edge;
		cur_location->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = cur_location;
		this->grid->cell[start_index[2]].edgeCount++;
		cur_location++;

		//start vertex and end vertex locate at the same cell
		if (start_index[2] == end_index[2])
			continue;

		FLTYPE dx, dy, temp, p, another_p;
		int old_index[2], cur_index[3], new_index[3], s0, s1, r0, r1;

		dx = end_vertex->x - start_vertex->x;
		dy = end_vertex->y - start_vertex->y;

		//calculate current edge's direction region
		if (end_vertex->x > start_vertex->x)
			s0 = 1;
		else //向左
			s0 = -1;
		if (end_vertex->y > start_vertex->y)
			s1 = 1;
		else //向下
			s1 = -1;

		int dir;
		if (s0 == 1 && s1 == 1)		//quadrant I
		{
			if (dy*cx < dx*cy)
				dir = 0;
			else
				dir = 1;
		}
		else if (s0 == -1 && s1 == 1)	//quadrant II
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 3;
			else
				dir = 2;
		}
		else if (s0 == -1 && s1 == -1)	//quadrant III
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 4;
			else
				dir = 5;
		}
		else if (s0 == 1 && s1 == -1)	//quadrant IV
		{
			if (abs(dy*cx) < abs(dx*cy))
				dir = 7;
			else
				dir = 6;
		}

		//calculate the initial value of p
		//p = p1_coefficient[dir] * cx*dy - cy*dx;
		FLTYPE x, y, left, right, top, bottom;
		left = this->grid->boundingBox[0].x + cx * start_index[0];
		right = this->grid->boundingBox[0].x + cx * (start_index[0] + 1);
		top = this->grid->boundingBox[0].y + cy * (start_index[1] + 1);
		bottom = this->grid->boundingBox[0].y + cy * start_index[1];
		if (dir == 0 || dir == 7)
		{
			y = (end_vertex->y - start_vertex->y) * (right - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
			p = dx * ((y - bottom) - (top - y));
		}
		else if (dir == 1 || dir == 2)
		{
			x = (end_vertex->x - start_vertex->x) * (top - start_vertex->y) / (end_vertex->y - start_vertex->y) + start_vertex->x;
			p = dy * ((x - left) - (right - x));
		}
		else if (dir == 3 || dir == 4)
		{
			y = (end_vertex->y - start_vertex->y) * (left - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
			p = dx * ((y - bottom) - (top - y));
		}
		else if (dir == 5 || dir == 6)
		{
			x = (end_vertex->x - start_vertex->x) * (bottom - start_vertex->y) / (end_vertex->y - start_vertex->y) + start_vertex->x;
			p = dy * ((x - left) - (right - x));
		}

		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		new_index[0] = 0;
		new_index[1] = 0;

		while (1)
		{
			old_index[0] = cur_index[0];
			old_index[1] = cur_index[1];

			switch (dir)
			{
			case 0:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 1;
					another_p = p + 2 * dy*cx;
					p = p + 2 * dy*cx - 2 * dx*cy;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += 0;
					another_p = p + 2 * dy*cx - 2 * dx*cy;
					p = p + 2 * dy*cx;
				}
				break;
			case 1:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy;
					p = p + 2 * dx*cy - 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 0;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy - 2 * dy*cx;
					p = p + 2 * dx*cy;
				}
				break;
			case 2:
				if (p >= 0)
				{
					cur_index[0] += 0;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy + 2 * dy*cx;
					p = p + 2 * dx*cy;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 1;
					another_p = p + 2 * dx*cy;
					p = p + 2 * dx*cy + 2 * dy*cx;
				}
				break;
			case 3:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += 0;
					another_p = p - 2 * dy*cx - 2 * dx*cy;
					p = p - 2 * dy*cx;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 1;
					another_p = p - 2 * dy*cx;
					p = p - 2 * dy*cx - 2 * dx*cy;
				}
				break;
			case 4:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += -1;
					another_p = p - 2 * dy*cx;
					p = p - 2 * dy*cx + 2 * dx*cy;
				}
				else
				{
					cur_index[0] += -1;
					cur_index[1] += 0;
					another_p = p - 2 * dy*cx + 2 * dx*cy;
					p = p - 2 * dy*cx;
				}
				break;
			case 5:
				if (p >= 0)
				{
					cur_index[0] += -1;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy;
					p = p - 2 * dx*cy + 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 0;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy + 2 * dy*cx;
					p = p - 2 * dx*cy;
				}
				break;
			case 6:
				if (p >= 0)
				{
					cur_index[0] += 0;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy - 2 * dy*cx;
					p = p - 2 * dx*cy;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += -1;
					another_p = p - 2 * dx*cy;
					p = p - 2 * dx*cy - 2 * dy*cx;
				}
				break;
			case 7:
				if (p >= 0)
				{
					cur_index[0] += 1;
					cur_index[1] += 0;
					another_p = p + 2 * dx*cy + 2 * dy*cx;
					p = p + 2 * dy*cx;
				}
				else
				{
					cur_index[0] += 1;
					cur_index[1] += -1;
					another_p = p + 2 * dy*cx;
					p = p + 2 * dx*cy + 2 * dy*cx;
				}
				break;
			}

			r0 = cur_index[0] - old_index[0];
			r1 = cur_index[1] - old_index[1];

			//find the missing cell from Ci to Cj
			FLTYPE ylcs, x, y, next;
			bool add_flag = false, correct_flag = false, vertical_flag = false;

			if (end_vertex->x == start_vertex->x)
				vertical_flag = true;

			if (r0 == 1 && r1 == 1)	//M==>RT
			{
				ylcs = this->grid->boundingBox[0].y + cy * cur_index[1];
				x = this->grid->boundingBox[0].x + cx * cur_index[0];
				//y = y0 + (x-xa)*(y1-y0)/(x1-x0)
				y = ((end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x)) + start_vertex->y;
				if (y > ylcs || vertical_flag == true) //add T
				{
					new_index[0] = cur_index[0] - 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y > (ylcs + cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y < ylcs)	//add R
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] - 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (next < ylcs)
						correct_flag = true;
				}
			}

			if (r0 == -1 && r1 == 1)	//M==>LT
			{
				ylcs = this->grid->boundingBox[0].y + cy * cur_index[1];
				x = this->grid->boundingBox[0].x + cx * (cur_index[0] + 1);
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y > ylcs || vertical_flag == true) //add T
				{
					new_index[0] = cur_index[0] + 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y > (ylcs + cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y < ylcs) //add L
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] - 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (y < ylcs)
						correct_flag = true;
				}
			}

			if (r0 == 1 && r1 == -1)	//M==>RB
			{
				ylcs = this->grid->boundingBox[0].y + cy * (cur_index[1] + 1);
				x = this->grid->boundingBox[0].x + cx * cur_index[0];
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y <= ylcs || vertical_flag == true) //add B
				{
					new_index[0] = cur_index[0] - 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y < (ylcs - cy) || vertical_flag == true)
						correct_flag = true;
				}
				else if (y > ylcs) //add R
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] + 1;
					add_flag = true;

					next = y + dy / dx * cx;
					if (next > ylcs)
						correct_flag = true;
				}
			}

			if (r0 == -1 && r1 == -1)	//M==>LB
			{
				ylcs = this->grid->boundingBox[0].y + cy * (cur_index[1] + 1);
				x = this->grid->boundingBox[0].x + cx * (cur_index[0] + 1);
				y = (end_vertex->y - start_vertex->y) * (x - start_vertex->x) / (end_vertex->x - start_vertex->x) + start_vertex->y;
				if (y < ylcs || vertical_flag == true) //add B
				{
					new_index[0] = cur_index[0] + 1;
					new_index[1] = cur_index[1];
					add_flag = true;

					if (y < (ylcs - cy))
						correct_flag = true;
				}
				else if (y > ylcs) //add L
				{
					new_index[0] = cur_index[0];
					new_index[1] = cur_index[1] + 1;
					add_flag = true;

					next = y - dy / dx * cx;
					if (next > ylcs)
						correct_flag = true;
				}
			}

			//add edge reference to the current cell (Cj)
			if (correct_flag == false &&
				cur_index[0] >= 0 && cur_index[0] < this->grid->resolution[0] &&
				cur_index[1] >= 0 && cur_index[1] < this->grid->resolution[1])
			{
				cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
				cur_location->e = cur_edge;
				cur_location->next = this->grid->cell[cur_index[2]].edgeRef;
				this->grid->cell[cur_index[2]].edgeRef = cur_location;
				this->grid->cell[cur_index[2]].edgeCount++;
				cur_location++;
			}
			else //correct_flag == true
			{
				cur_index[0] = new_index[0];
				cur_index[1] = new_index[1];
				p = another_p;
			}

			//add edge reference to the missing cell
			if (add_flag == true &&
				new_index[0] >= 0 && new_index[0] <= this->grid->resolution[0] - 1 &&
				new_index[1] >= 0 && new_index[1] <= this->grid->resolution[1] - 1)
			{
				new_index[2] = new_index[1] * this->grid->resolution[0] + new_index[0];
				cur_location->e = cur_edge;
				cur_location->next = this->grid->cell[new_index[2]].edgeRef;
				this->grid->cell[new_index[2]].edgeRef = cur_location;
				this->grid->cell[new_index[2]].edgeCount++;
				cur_location++;
			}

			//termination criterion
			if (cur_index[0] == end_index[0] && cur_index[1] == end_index[1] ||
				add_flag == true && new_index[0] == end_index[0] && new_index[1] == end_index[1])
				break;
		}
	} //for i

	//记录真实指针数
	this->stat.realRefCount = cur_location - this->grid->edgeRef + 1;

}  //addEdgeRef_Bresenham


/*
//modified version of bresenham algorithm
void GridPIP2D::addEdgeRef_redlark()
{
	struct EdgeRef2D * cur_location = this->grid->edgeRef;
	for(int i=0; i<this->testedPolygon->edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon->edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon->vertexTable[cur_edge->startIndex];
		Point2D * end_vertex = &this->testedPolygon->vertexTable[cur_edge->endIndex];

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引
		float fxi, fyi;				//the float index of the cell that current point fell in
		float fxi2, fxi2;			//the float index of the cell that the end point fell in
		int xi, yi;					//the integer index of current cell
		float ystep;				//the step length in y when marching a cell in x

		ystep = (end_vertex->y - start_vertex->y) / (end_vertex->x - start_vertex->x);
		this->grid->locatePoint_f(start_vertex->x, start_vertex->y, &fxi, &fyi);
		this->grid->locatePoint_f(end_vertex->x, end_vertex->y, &fxi2, &fyi2);
		start_index[2] = this->grid->locatePoint(start_vertex->x, start_vertex->y, start_index[0], start_index[1]);
		xi = start_index[0];
		yi = start_index[1];

		//add start cell [xi][yi]
		cur_location->e = cur_edge;
		cur_location->next = this->grid->cell[start_index[2]].edgeRef;
		this->grid->cell[start_index[2]].edgeRef = cur_location;
		this->grid->cell[start_index[2]].edgeCount++;
		cur_location++;

		//compute the sign of increment
		int signx, signy;
		if(end_vertex->x > start_vertex->x) //向右
			signx = 1;
		else //向左
			signx = -1;
		if(end_vertex->y > start_vertex->y) //向上
			signy = 1;
		else //向下
			signy = -1;

		//start increment
		fyi += ystep * (xi +1 - fxi);
		fxi = xi + 1;
		do{
			if(fyi >= yi+1)
			{
				//add cell[xi][yi+1]
				...
			}
			xi++;
			yi=(int)fyi;
			//add cell[xi][yi]
			{
				...
			}
			fxi += 1;
			fyi += ystep;
		}while(fxi < fxi2)

		//current location is at the right edge of the cell the end point locate
		//rollback to the end point
		fyi -= ystep * (fxi - fxi2);
		if(fyi > (yi+1))
		{
			//add cell[xi][yi+1]
			...
		}
		else
		{
			//add cell [xi+1][(int)fyi]
			...
		}
	}
}
*/

//图学学报
//标记网格单元的位置属性，即，多边形外，多边形内，含多边形
// CELL_OUT	0        CELL_IN		1        CELL_SUSPECT 2
void GridPIP2D::RgpMarkCells()
{
	int temp = 0;
	for (int j=0;j<this->grid->resolution[1];j++)
	{
		for (int i=0;i<this->grid->resolution[0];i++)
		{
			if(this->grid->cell[temp].cell_InofOut!= CELL_SUSPECT)
			{
				int flag = this->RgpPoint[temp+j].flag;
				this->grid->cell[temp].cell_InofOut= flag;
			}//if
			temp++;
		}//secon for
	}//first for
}

void GridPIP2D::G_RgpMarkCells()
{
	int temp = 0;
	this->SuspectCellCount = 0;//used only in group method
	for (int j = 0; j < this->grid->resolution[1]; j++)
	{
		for (int i = 0; i < this->grid->resolution[0]; i++)
		{
			if (this->grid->cell[temp].cell_InofOut != CELL_SUSPECT)
			{
				int flag = this->RgpPoint[temp + j].flag;
				this->grid->cell[temp].cell_InofOut = flag;
			}//if
			else
			{
				//该部分内容用于，确定含边单元网格的个数，用于group方法，其他方法可将else部分注释
				this->SuspectCellCount++;
			}
			temp++;
		}//secon for
	}//first for
}
//图学学报
void GridPIP2D::RgpGetIntersectionPoint(Point2D* a, Point2D* b, MySortOfPoint *mySP0)
{
	Point2D* temp0;
	int count0;
	temp0 = new Point2D[1000];
	RgpGetIntersectionBetweenCellAndEdge(a, b, temp0, count0);//计算一条边与网格单元的交点
	SortTheIntersectionPoint(count0, a, temp0, mySP0);
}
//图学学报
//对获取的多边形片段进行排序
void GridPIP2D::SortTheIntersectionPoint(int count, Point2D* a, Point2D* temp0, MySortOfPoint *mySP0)
{
	for (int iii1 = 0; iii1 < count; iii1++)
	{
		mySP0[iii1].distance1 = fabs(temp0[iii1].x - a->x) + fabs(temp0[iii1].y - a->y);
		mySP0[iii1].a = temp0[iii1];
	}
	MSPCmp cmp;
	sort(mySP0, mySP0 + count, cmp);
}

//图学学报
void GridPIP2D::RgpGetIntersectionBetweenCellAndEdge(Point2D* p1, Point2D* p2, Point2D* totalPXY, int& count)
{
	int minCellX, minCellY, maxCellX, maxCellY;
	double minPCX, maxPCX, minPCY, maxPCY;

	GetMaxMinCoordinate(minPCX, maxPCX, minPCY, maxPCY, p1, p2);//计算两个点的最大最小X，Y坐标值，即两点围成方框的左上角和右下角
	GetCellCoordinateRange(minPCX, maxPCX, minPCY, maxPCY, minCellX, maxCellX, minCellY, maxCellY);//获取两点围成的方框所占的网格坐标

	Line2D* line0 = NULL;
	line0 = new Line2D(p1, p2);
	double slope0 = line0->CalculateSlope();
	double lineB = p1->y - slope0 * p1->x;

	int i = 1;
	totalPXY[0].x = p1->x;
	totalPXY[0].y = p1->y;
	for (int i0 = minCellX; i0 < maxCellX; i0++)
	{
		double tempX = xmin + (i0 + 1)*SizeX;
		//如果垂直X轴，择不会跨越多个横坐标
		totalPXY[i].x = tempX;
		double v0 = slope0 * tempX + lineB;//计算多边形边与纵轴的交点坐标（y值）
		totalPXY[i].y = v0;
		//vecPointX2[i0].push_back(v0);//i0从第二条竖直边开始，即，除去最左边和最右边
		SetIntersectPointSite(v0, i0, 1);
		i++;
	}
	for (int j0 = minCellY; j0 < maxCellY; j0++)
	{
		double tempY = ymin + (j0 + 1)*SizeY;
		//interPoint[j0].mark = InsideFlag;//LEFT 0
		if (slope0 == PI_HALF)
		{
			totalPXY[i].x = p1->x;
			totalPXY[i].y = tempY;
			//vecPointY2[j0].push_back(p1->x);
			//interPoint[j0].x = p1->x;
			SetIntersectPointSite(p1->x, j0, 0);
			i++;
		}
		//如果平行X轴，则不会出现跨越多个纵向网格
		else
		{
			double yy = (tempY - lineB) / slope0;
			totalPXY[i].x = yy;
			totalPXY[i].y = tempY;
			//vecPointY2[j0].push_back(yy);
			//interPoint[j0].x = yy;
			SetIntersectPointSite(yy, j0, 0);
			//	structPointX[j0].push_back(interPoint[j0]);
			//	FinalstructPointX[j0].push_back(interPoint);
			i++;
		}
	}
	totalPXY[i].x = p2->x;
	totalPXY[i].y = p2->y;
	count = ++i;
}
//图学学报 //将多边形边放置到相应的网格线片段
void GridPIP2D::SetIntersectPointSite(FLTYPE coord_value, int index, int flag)
{
	//flag 标记位，0表示横轴，1表示纵轴
	int a, b;
	if (flag == 0) //存储到横轴网格线中
	{
		a = (int)(coord_value - this->grid->boundingBox[0].x) / this->grid->cellSize[0];
		b = index * this->grid->resolution[0] + a;  //此处index-1是由于最上侧那条边与多边形边无交点，因此减少一条计数边
		this->RgpIntersectPointX[b].xory_coord[this->RgpIntersectPointX[b].pointCount++] = coord_value;
	}
	else
	{
		a = (int)(coord_value - this->grid->boundingBox[0].y) / this->grid->cellSize[1];
		b = index * this->grid->resolution[1] + a;//原因同上，最左侧边不需要计数
		this->RgpIntersectPointY[b].xory_coord[this->RgpIntersectPointY[b].pointCount++] = coord_value;
	}
}
//图学学报
int GridPIP2D::JudgeInOutProperty(Point2D* p1, Point2D* p2)
{
	if (p1->y < p2->y)
	{
		return LEFT;
	}
	else
	{
		return RIGHT;
	}
}

//图学学报
void GridPIP2D::GetCellCoordinateRange(double minPCX, double maxPCX, double minPCY, double maxPCY,
	int& minCellX, int& maxCellX, int& minCellY, int& maxCellY)
{
	double a = (minPCX - xmin) / SizeX;
	minCellX = (int)a;
	double b = (maxPCX - xmin) / SizeX;
	maxCellX = (int)b;
	double c = (minPCY - ymin) / SizeY;
	minCellY = (int)c;
	double d = (maxPCY - ymin) / SizeY;
	maxCellY = (int)d;
}

//图学学报
void GridPIP2D::GetMaxMinCoordinate(double &min_a, double& max_a, double& min_b, double& max_b, Point2D* p1, Point2D* p2)
{
	min_a = p1->x < p2->x ? p1->x : p2->x;
	max_a = p1->x > p2->x ? p1->x : p2->x;
	min_b = p1->y < p2->y ? p1->y : p2->y;
	max_b = p1->y > p2->y ? p1->y : p2->y;
}

//图学学报方法 GCP    计算网格交点的坐标，以及网格交点的位置属性
//记录多边形边与网格线的交点，分开存储，竖直网格线、平行网格线
void GridPIP2D::RgpsetCellIntersectProp()
{
	//排序各个网格线上的交点坐标
	//再次改进版，使用vector存储边及相关交点
	//由于前期处理申请的内存过多，处理时只添加了少数交点坐标，
	//下列主要工作是将有用的坐标信息提出，删除无用的值（即MAX_DOUBLE）
	//int m, n;
	//vector <vector <double> >vecCPY;//用来存储排序后的交点Y轴坐标
	//vector <double> vecPY;//存储每一纵轴上的交点的Y值
	//for (m = 0; m < vecPointX2.size(); m++)
	//{
	//	for (n = 0; n < vecPointX2[m].size(); n++)
	//	{
	//		double valueX = vecPointX2[m][n];
	//		if (valueX != MAX_DOUBLE)
	//		{
	//			vecPY.push_back(valueX);
	//		}
	//	}
	//	sort(vecPY.begin(), vecPY.end(), cmp2);
	//	vecCPY.push_back(vecPY);
	//	vecPY.clear();
	//}
	//FinalVecPointY = vecCPY;
	////  
	//vector <vector <double> >vecCPX;//用来存储排序后的交点X轴坐标
	//vector <double> vecPX;//存储每一纵轴上的交点的X值
	//for (m = 0; m < vecPointY2.size(); m++)
	//{
	//	for (n = 0; n < vecPointY2[m].size(); n++)
	//	{
	//		double valueY = vecPointY2[m][n];
	//		if (valueY != MAX_DOUBLE)
	//		{
	//			vecPX.push_back(valueY);
	//		}
	//	}
	//	sort(vecPX.begin(), vecPX.end(), cmp2);
	//	double v2 = vecPX[0];
	//	double v3 = vecPX[1];
	//	vecCPX.push_back(vecPX);
	//	vecPX.clear();
	//}
	//FinalVecPointX = vecCPX;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//处理各个网格交点的位置属性
	//CELL_UNKNOWN - 1  CELL_OUT	0   CELL_IN		1     CELL_SUSPECT 2
	//设置边界框四周的网格交点位置属性为多边形外
	//最左边
	//int temp=0;
	for (int i = 0; i < this->grid->resolution[1] + 1; i++)
	{
		int temp = i * (this->grid->resolution[0] + 1);
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x;
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y + i * this->grid->cellSize[1];
		//temp = temp+ (this->grid->resolution[0] + 1);
	}
	//最右边
	for (int i = 0; i < this->grid->resolution[1] + 1; i++)
	{
		int temp = i * (this->grid->resolution[0] + 1) + this->grid->resolution[0];
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x + this->grid->resolution[0] * this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y + i * this->grid->cellSize[1];
	}
	//最上边 
	for(int i=1;i< this->grid->resolution[0];i++)
	{
		int temp = i ;
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x+i* this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y ;
	}
	//最下边
	for (int i = 1; i < this->grid->resolution[0]; i++)
	{
		int temp = i+ (this->grid->resolution[0] + 1)*this->grid->resolution[1];
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x + i * this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y+ this->grid->resolution[1] *this->grid->cellSize[1];
	}
	//其余边
	//count_BORDER = 0;//用来统计网格交点处于多边形边上的次数
	FLTYPE left_point_x, cur_point_x, cur_point_y;
	int cur_line_point;   //当前网格点的一维数组排序 （每行的第一个交点的坐标）
	cur_point_y = this->grid->boundingBox[0].y;
	int x_segment_index = 0;
	for (int i = 1; i < this->grid->resolution[1]; i++)
	{
		cur_line_point = i * (this->grid->resolution[0] + 1);
		left_point_x = this->grid->boundingBox[0].x;
		cur_point_y += this->grid->cellSize[1];
		for (int j = 1; j < this->grid->resolution[0]; j++)
		{
			//参考原方法的处理步骤，减少乘法次数
			int temp = cur_line_point + j;//计算当前网格交点所处一位数组的位置 
			cur_point_x=left_point_x+ this->grid->cellSize[0];
			this->RgpPoint[temp].gridPoint.x = cur_point_x;
			this->RgpPoint[temp].gridPoint.y = cur_point_y;
	
			FLTYPE a1 = left_point_x;
			FLTYPE a2 = cur_point_x;
			left_point_x = cur_point_x;

			int flag = this->RgpPoint[temp - 1].flag;
			//int num = this->GetNumBetweentwoPoint(FinalVecPointX[i-1],a1,a2);//统计交点个数
			int num= this->RgpIntersectPointX[x_segment_index++].pointCount;
			int mark = num % 2 ;
			switch (mark)
			{
			case 1:
				flag = InvertFlag(flag);
				this->RgpPoint[temp].flag = flag;
				break;
			default:
				this->RgpPoint[temp].flag = flag;
				break;
			}
		}//inside for
		x_segment_index++;
	}//outside for
	//printf("BORDER times %f\n", count_BORDER);
}

//TVCG OGP method
void GridPIP2D::OgpsetCellIntersectProp()
{
	//处理各个网格交点的位置属性
	//CELL_UNKNOWN - 1  CELL_OUT	0   CELL_IN		1     CELL_SUSPECT 2

	//设置边界框四周的网格交点位置属性为多边形外
	//最左边
	//int temp=0;
	for (int i = 0; i < this->grid->resolution[1] + 1; i++)
	{
		int temp = i * (this->grid->resolution[0] + 1);
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x;
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y + i * this->grid->cellSize[1];
		//temp = temp+ (this->grid->resolution[0] + 1);
	}
	//最右边
	for (int i = 0; i < this->grid->resolution[1] + 1; i++)
	{
		int temp = i * (this->grid->resolution[0] + 1) + this->grid->resolution[0];
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x + this->grid->resolution[0] * this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y + i * this->grid->cellSize[1];
	}
	//最上边 
	for (int i = 1; i < this->grid->resolution[0]; i++)
	{
		int temp = i;
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x + i * this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y;
	}
	//最下边
	for (int i = 1; i < this->grid->resolution[0]; i++)
	{
		int temp = i + (this->grid->resolution[0] + 1)*this->grid->resolution[1];
		this->RgpPoint[temp].flag = CELL_OUT;
		this->RgpPoint[temp].gridPoint.x = this->grid->boundingBox[0].x + i * this->grid->cellSize[0];
		this->RgpPoint[temp].gridPoint.y = this->grid->boundingBox[0].y + this->grid->resolution[1] * this->grid->cellSize[1];
	}
	//其余边
	//count_BORDER = 0;//用来统计网格交点处于多边形边上的次数
	FLTYPE left_point_x, cur_point_x, cur_point_y;
	int cur_line_point;   //当前网格点的一维数组排序 （每行的第一个交点的坐标）
	cur_point_y = this->grid->boundingBox[0].y;
	int x_segment_index = 0;
	FLTYPE coord_value = 0;
	Point2D* p1 = new Point2D();
	Point2D* p2 = new Point2D();
	for (int i = 1; i < this->grid->resolution[1]; i++)
	{
		cur_line_point = i * (this->grid->resolution[0] + 1);
		left_point_x = this->grid->boundingBox[0].x;
		cur_point_y += this->grid->cellSize[1];
		for (int j = 1; j < this->grid->resolution[0]; j++)
		{
			//参考原方法的处理步骤，减少乘法次数
			int temp = cur_line_point + j;//计算当前网格交点所处一位数组的位置 
			cur_point_x = left_point_x + this->grid->cellSize[0];
			this->RgpPoint[temp].gridPoint.x = cur_point_x;
			this->RgpPoint[temp].gridPoint.y = cur_point_y;

			FLTYPE a1 = left_point_x;
			FLTYPE a2 = cur_point_x;
			left_point_x = cur_point_x;

			GridCell2D *cur_cell = &(this->grid->cell[x_segment_index]);
			RGPEdgeRef2D* cur_Ogp_edge_ref = cur_cell->RgpEdgeRef;

			int flag = this->RgpPoint[temp - 1].flag;
			int num = this->RgpIntersectPointX[x_segment_index++].pointCount;  //获取当前网格片段上的交点
			if (num == 0&& cur_Ogp_edge_ref==NULL)//需要改进，判断边片段是否和网格交点重合
			{
				this->RgpPoint[temp].flag = flag;
			}
			else if (num == 1&& cur_Ogp_edge_ref != NULL)
			{
				p1 = cur_Ogp_edge_ref->start_e;
				p2 = cur_Ogp_edge_ref->end_e;
				int flag0 = p1->x <= p2->x ? 0 : 1;
				int flag1 = p1->y <= p2->y ? 2 : 4;
				int flag2 = flag0 + flag1;
				switch (flag2)
				{
				case 2:
					this->RgpPoint[temp].flag = CELL_OUT; //0
					break;
				case 4:
					this->RgpPoint[temp].flag = CELL_IN;
					break;
				case 3:
					this->RgpPoint[temp].flag = CELL_OUT;
					break;
				default:
					this->RgpPoint[temp].flag = CELL_IN;
					break;
				}
			}
			else  //网格线片段中含多个点   找出最右侧的点，即最大值
			{
				FLTYPE x_value = this->RgpIntersectPointX[x_segment_index - 1].xory_coord[0];
				for (int i=1;i<num;i++)
				{
					if (x_value < this->RgpIntersectPointX[x_segment_index - 1].xory_coord[i])
					{
						x_value = this->RgpIntersectPointX[x_segment_index - 1].xory_coord[i];
					}
				}
				//寻找与上述找到x_value值重合的边
				while (cur_Ogp_edge_ref != NULL)
				{
					p1 = cur_Ogp_edge_ref->start_e;
					p2 = cur_Ogp_edge_ref->end_e;
					if (p1->x == x_value || p2->x == x_value)
					{
						cur_Ogp_edge_ref=NULL;
					}
					else
					{
						cur_Ogp_edge_ref = cur_Ogp_edge_ref->next;
					}
				}//while

				int flag0 = p1->x <= p2->x ? 0 : 1;
				int flag1 = p1->y <= p2->y ? 2 : 4;
				int flag2 = flag0 + flag1;
				switch (flag2)
				{
				case 2:
					this->RgpPoint[temp].flag = CELL_OUT; //0
					break;
				case 4:
					this->RgpPoint[temp].flag = CELL_IN;
					break;
				case 3:
					this->RgpPoint[temp].flag = CELL_OUT;
					break;
				default:
					this->RgpPoint[temp].flag = CELL_IN;
					break;
				}//switch
			}//else

			/*int mark = num % 2;
			switch (mark)
			{
			case 1:
				flag = InvertFlag(flag);
				this->RgpPoint[temp].flag = flag;
				break;
			default:
				this->RgpPoint[temp].flag = flag;
				break;
			}*/

		}//inside for
		x_segment_index++;
	}//outside for
	//printf("BORDER times %f\n", count_BORDER);
}

//图学学报
int GridPIP2D::InvertFlag(int flag)
{
	if (flag == 0)
	{
		flag = 1;
	}
	else
	{
		flag = 0;
	}
	return flag;
}
int GridPIP2D::Equal(double a, double b, double delta)
{
	if (abs(a - b) <= delta) 
		return 1; 
	else 
		return 0;
}
//包含统计数据代码
int GridPIP2D::Equal_stat(double a, double b, double delta)
{
	this->Rgpstat.addCount++;
	this->Rgpstat.compareCount++;
	if (abs(a - b) <= delta)
		return 1;
	else
		return 0;
}

// 图学学报 使用二分法查找两个网格交点之间  多边形边与网格线的交点个数
int GridPIP2D::GetNumBetweentwoPoint(vector<double >& nums, double x1, double x2)
{
	int num = 0;//用于计数
	int start = 0, end = nums.size() - 1;
	int end1 = end;
	while (start <= end1)
	{
		int mid = (start + end1) / 2;
		if (nums[mid] <= x1)
			start = mid + 1;
		else if (nums[mid] > x1)
			end1 = mid - 1;
	}
	double tempv;
	if (start <= 0)
	{
		tempv = nums[start];
		for (int i = start; i <= end && tempv < x2; i++)
		{
			tempv = nums[i];
			if ((Equal(x1, tempv, 10e-6) == 1) || (Equal(x2, tempv, 10e-6) == 1))
			{
				return -1;
			}
			if (x1<tempv&&x2>tempv)
			{
				num++;
			}
		}
	}//if
	else
	{
		tempv = nums[start - 1];
		for (int i = start - 1; i <= end &&tempv < x2; i++)
		{
			tempv = nums[i];
			if ((Equal(x1, tempv, 10e-6) == 1) || (Equal(x2, tempv, 10e-6) == 1))
			{
				return -1;
			}
			if (x1<tempv&&x2>tempv) //	if(x3<x5&&x4>x5)
			{
				num++;
			}
		}//else
	}
	return num;
}
//图学学报  计算邻近网格交点的位置
int GridPIP2D::ReturnPointCInGrid(FLTYPE x, FLTYPE y, int & x_index, int & y_index)
{
	/*int Point_index;
	FLTYPE a0 = (x - this->grid->boundingBox[0].x) / grid->cellSize[0] + 0.5;
	FLTYPE b0 = (y - this->grid->boundingBox[0].y) / grid->cellSize[1] + 0.5;
	x_index = (int)a0;
	y_index = (int)b0;
	Point_index = y_index * (this->grid->resolution[0] + 1) + x_index;
	return Point_index;*/
	x_index =(int)((x - this->grid->boundingBox[0].x) / grid->cellSize[0] + 0.5);
	y_index = (int)((y - this->grid->boundingBox[0].y) / grid->cellSize[1] + 0.5);
	return (y_index * (this->grid->resolution[0] + 1) + x_index);
}
//图学学报 返回同一纵轴上两点中的交点个数
int GridPIP2D::ReturnIntersectionPointsNumX(FLTYPE y1, FLTYPE y2, int x)
{
	int num = 0;
	if (x < 0 || x == 0 || x == this->grid->resolution[0])
	{
		return 0;
	}
	//二分法查找
	double y3 = y1 < y2 ? y1 : y2;
	double y4 = y1 > y2 ? y1 : y2;
	num = GetNumBetweentwoPoint(FinalVecPointY[x - 1], y3, y4);
	return num;
}
//图学学报
//返回同一横轴两个点之间的交点个数
int GridPIP2D::ReturnIntersectionPointsNumY(FLTYPE x1, FLTYPE x2, int y)
{
	int num = 0;
	if (y < 0 || y == (this->grid->resolution[1] - 1))
	{
		return 0;
	}
	FLTYPE x3 = x1 < x2 ? x1 : x2;
	FLTYPE x4 = x1 > x2 ? x1 : x2;
	//二分法查找相邻网格交点之间的数据
	num = GetNumBetweentwoPoint(FinalVecPointX[y], x3, x4);
	return num;
}
//当测试点的落在网格线上的情况
//p1:测试点坐标。x，y：临近网格交点坐标
int GridPIP2D::TestEdgeSingularCase(Point2D* p1, int x, int y)
{
	FLTYPE a, b;
	a = this->RgpPoint->gridPoint.x;
	b = this->RgpPoint->gridPoint.y;
	if ((Equal(p1->x, a, 10e-6) == 1) && Equal(p1->y, b, 10e-6) == 1)
	{
		return this->RgpPoint->flag;
	}
	if (Equal(p1->x, a, 10e-6) == 1)
	{
		int num0= ReturnIntersectionPointsNumX(p1->y, b, x);
		if (num0 == -1)
		{
			return num0;
		}
		else if (num0 % 2 == 0)
		{
			return this->RgpPoint->flag;
		}
		else
		{
			int num2 = InvertFlag(this->RgpPoint->flag);
			return	num2;
		}
	}
}
//叉积计算两条线段是否相交
bool GridPIP2D::Intersect1(Point2D* aa, Point2D* bb, Point2D* cc, Point2D* dd)
{
	if (max(aa->x, bb->x) < min(cc->x, dd->x))
	{
		return false;
	}
	if (max(aa->y, bb->y) < min(cc->y, dd->y))
	{
		return false;
	}
	if (max(cc->x, dd->x) < min(aa->x, bb->x))
	{
		return false;
	}
	if (max(cc->y, dd->y) < min(aa->y, bb->y))
	{
		return false;
	}
	if (Mult(cc, bb, aa)*Mult(bb, dd, aa) < 0)
	{
		return false;
	}
	if (Mult(aa, dd, cc)*Mult(dd, bb, cc) < 0)
	{
		return false;
	}
	return true;
}
//图学学报 
//计算网格单元中心点的y坐标值
void GridPIP2D::SetCellMiddlePointCoord()
{
	int  cell_index = 0;
	FLTYPE half_cell_size_y, tempy;
	half_cell_size_y = this->grid->cellSize[1] / 2;
	tempy = this->grid->boundingBox[0].y + half_cell_size_y;
	for (int i = 0; i < this->grid->resolution[1]; i++)
	{
		for (int j = 0; j < this->grid->resolution[0]; j++)
		{
			this->grid->cell[cell_index++].cell_middle_y = tempy;
		}//first for
		tempy += this->grid->cellSize[1];
	}//second for
}

//图学学报 计算测试点的位置属性 根据网格线片段标记查找
void GridPIP2D::RgpCheckPoint_Segment()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;
	//统计乘法次数
	this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		this->Rgpstat.compareCount++;     //for 循环的比较
		this->Rgpstat.addCount++;            //for循环的加法
		Point2D * p = &(this->testedPoint[i]);
		//剔除在包围盒外部的点
		if ((this->Rgpstat.compareCount++&&p->x < this->grid->boundingBox[0].x) ||
			(this->Rgpstat.compareCount++&&p->x > this->grid->boundingBox[1].x) ||
			(this->Rgpstat.compareCount++&&p->y < this->grid->boundingBox[0].y) ||
			(this->Rgpstat.compareCount++&&p->y > this->grid->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);
		//统计上述函数中加法和乘法次数
		this->Rgpstat.addCount = this->Rgpstat.addCount + 5;
		this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 3;
		//图学学报 start
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		//统计比较次数
		this->Rgpstat.compareCount++;
		if (cell_flag == CELL_OUT)   
		{
			//this->Rgpstat.outsideCellPointCount++;
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		//统计比较次数
		this->Rgpstat.compareCount++;
		if (cell_flag == CELL_IN)    
		{
			//this->Rgpstat.insideCellPointCount++;
			this->testedResult[i] = CELL_IN;
			continue;
		}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		//this->Rgpstat.inEdgeCellPointCount++;   //统计测试点落在含边网格中的个数
		int point_index[3];
		int x_segment_index, y_segment_index;
		int up_or_down;     //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		double tempy;        //考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;

		this->Rgpstat.compareCount++;
		if (p->y <= tempy1)
		{
			tempy = tempy1 - half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1];
			point_index[2]= cell_index[2] + cell_index[1];
			up_or_down = 0;
			this->Rgpstat.addCount = this->Rgpstat.addCount + 2;  //统计加法次数
		}//if
		else
		{
			tempy = tempy1 + half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1]+1;
			point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
			up_or_down = 1;
			this->Rgpstat.addCount = this->Rgpstat.addCount + 5;   			//统计加法次数
		}//else
		//
		FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;
		int  markflag = this->RgpPoint[point_index[2]].flag; //获取网格点的属性
		int countNum = 0;//记录交点数总数

		this->Rgpstat.compareCount++;
		if (Equal_stat(y1, p->y, 10e-10) == 1)    //平行X轴
		{
			//this->Rgpstat.onthe_x_GridEdge++;
			if ((this->Rgpstat.compareCount++&&Equal_stat(p->y, this->grid->boundingBox[0].y, 10e-6) == 1)
				||(this->Rgpstat.compareCount++&& Equal_stat(p->y, this->grid->boundingBox[1].y, 10e-6) == 1))
			{
				this->testedResult[i] = markflag;
				continue;
			}//if
			else
			{
				this->Rgpstat.compareCount++;
				if (up_or_down == 0)
				{
					x_segment_index = cell_index[2] - this->grid->resolution[0];
					this->Rgpstat.addCount++;
				}
				else
				{
					x_segment_index = cell_index[2];
				}
				countNum = this->GetCountNumInSegment(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
				//根据交点总数判断测试点的位置属性
				int mark = countNum % 2;
				this->Rgpstat.multiplicationCount++;  //统计乘法次数
				this->Rgpstat.compareCount++;
				switch (mark)
				{
				case 1:
					this->Rgpstat.compareCount++; //统计比较次数
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					break;
				default:
					this->testedResult[i] = markflag;
				}
				continue;
			}//else
		}

		this->Rgpstat.compareCount++;
		if (Equal_stat(x1, p->x, 10e-10) == 1)   //平行Y轴
		{
			//this->Rgpstat.onthe_y_GridEdge++;
			/*if (Equal(p->x,this->grid->boundingBox[0].x, 10e-6)==1
				||Equal(p->x,this->grid->boundingBox[1].x, 10e-6)==1)*/
			if ((this->Rgpstat.compareCount++&&Equal_stat(p->x, this->grid->boundingBox[0].x, 10e-6) == 1)
				|| (this->Rgpstat.compareCount++&&Equal_stat(p->x, this->grid->boundingBox[1].x, 10e-6) == 1))
			{
				this->testedResult[i] = markflag;
				continue;
			}
			else
			{
				y_segment_index = (cell_index[0]-1)*this->grid->resolution[1]+cell_index[1];
				countNum = this->GetCountNumInSegment(p->y, y1, y_segment_index, 1);
				int mark = countNum % 2;
				this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
				this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;
				this->Rgpstat.compareCount++;
				switch (mark)
				{
				case 1:
					this->Rgpstat.compareCount++;
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					break;
				default:
					this->testedResult[i] = markflag;
				}
				continue;
			}//else
		}//if
		//
		//代码中需要考虑垂直测试线与网格线的交点是否落在多边形边上，点（p->x,tempy）
		//考虑网格交点是否是奇异点
		//
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		FLTYPE k, B;//斜率和截距
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;

		this->Rgpstat.compareCount++;//统计while比较次数
		while (cur_Rgp_edge_ref != NULL)
		{
			this->Rgpstat.compareCount++;//统计比较次数
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			double min_a = p1->x < p2->x ? p1->x : p2->x;
			double max_a = p1->x > p2->x ? p1->x : p2->x;
			double min_b = p1->y < p2->y ? p1->y : p2->y;
			double max_b = p1->y > p2->y ? p1->y : p2->y;
			this->Rgpstat.compareCount= this->Rgpstat.compareCount+5;
			switch (up_or_down)
			{
			case 0:     //向上
				//if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b))
				if ((this->Rgpstat.compareCount++&&p->x <= min_a) ||
					(this->Rgpstat.compareCount++&&p->x >= max_a ) ||
					(this->Rgpstat.compareCount++&&p->y <= min_b&&
						this->Rgpstat.compareCount++&&tempy <= min_b))
				{
					//this->Rgpstat.exclude_by_CoorContast++;
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					continue;
				}//if
				else
				{
					//this->Rgpstat.cross_product_Count++;
					//利用斜率计算交点坐标
					k = cur_Rgp_edge_ref->k;
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					this->Rgpstat.addCount++;
					this->Rgpstat.multiplicationCount++;
					this->Rgpstat.compareCount++;
					if (temp_y_value >= p->y)
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						continue;
					}
					else
					{
						countNum++;
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						this->Rgpstat.addCount++;
					}
					break;
				}
				break;
			default:     //向下
				//if (p->x <= min_a || p->x >= max_a || (p->y >= max_b && tempy >= max_b))
				if ((this->Rgpstat.compareCount++&&p->x <= min_a) ||
					(this->Rgpstat.compareCount++&&p->x >= max_a) ||
					(this->Rgpstat.compareCount++&&p->y >= max_b &&
						this->Rgpstat.compareCount++ && tempy >= max_b ))
				{
					//this->Rgpstat.exclude_by_CoorContast++;
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					continue;
				}//if
				else
				{
					//this->Rgpstat.cross_product_Count++;
					//利用斜率计算交点坐标
					k = cur_Rgp_edge_ref->k;
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					this->Rgpstat.addCount++;
					this->Rgpstat.multiplicationCount++;
					this->Rgpstat.compareCount++;
					if (temp_y_value <= p->y)
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						continue;
					}
					else
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						countNum++;
						this->Rgpstat.addCount++;
					}
					break;
				}
				break;
			}//switch
		}//while
		int num=0;
		//根据网格线片段查找 start
		//if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
		if ((this->Rgpstat.compareCount++&&point_index[1] == 0) ||
			(this->Rgpstat.compareCount++&&point_index[1] == (this->grid->resolution[1])))
		{
			num = 0;
		}
		else
		{
			this->Rgpstat.compareCount++;
			if (up_or_down == 0)
			{
				x_segment_index = cell_index[2] - this->grid->resolution[0];
				this->Rgpstat.addCount++;
			}
			else
			{
				x_segment_index = cell_index[2];
			}
			num= this->GetCountNumInSegment(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
		}
		//根据网格线片段查找 end

		countNum = countNum + num;
		int mark = countNum % 2;
		this->Rgpstat.addCount++;
		this->Rgpstat.multiplicationCount++;
		this->Rgpstat.compareCount++;
		switch (mark)
		{
		case 1:
			this->Rgpstat.compareCount++;
			markflag = InvertFlag(markflag);
			this->testedResult[i] = markflag;
			break;
		default:
			this->testedResult[i] = markflag;
		}
		continue;
	} //for
	this->Rgpstat.compareCount++;//for 循环的比较
}

//图学学报 计算测试点的位置属性 根据网格线片段标记查找 无统计代码
void GridPIP2D::RgpChekPoint_Segment_noStat()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);
		//剔除在包围盒外部的点
		if ((p->x < this->grid->boundingBox[0].x) ||
			(p->x > this->grid->boundingBox[1].x) ||
			(p->y < this->grid->boundingBox[0].y) ||
			(p->y > this->grid->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

		//图学学报 start
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		if (cell_flag == CELL_OUT)
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		if (cell_flag == CELL_IN)
		{
			this->testedResult[i] = CELL_IN;
			continue;
		}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		int point_index[3];
		int x_segment_index, y_segment_index;
		int up_or_down; //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		double tempy;  //考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;
		if (p->y <= tempy1)
		{
			tempy = tempy1 - half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1];
			point_index[2] = cell_index[2] + cell_index[1];
			up_or_down = 0;
		}//if
		else
		{
			tempy = tempy1 + half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1] + 1;
			point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
			up_or_down = 1;
		}//else
		//
		FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;

		int  markflag = this->RgpPoint[point_index[2]].flag; //获取网格点的属性
		int countNum = 0;//记录交点数总数

		if (Equal(y1, p->y, 10e-10) == 1)    //平行X轴
		{
			if (Equal(p->y, this->grid->boundingBox[0].y, 10e-6) == 1
				|| Equal(p->y, this->grid->boundingBox[1].y, 10e-6) == 1)
			{
				this->testedResult[i] = markflag;
				continue;
			}//if
			else
			{
				if (up_or_down == 0)
				{
					x_segment_index = cell_index[2] - this->grid->resolution[0];
				}
				else
				{
					x_segment_index = cell_index[2];
				}
				countNum = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
				//根据交点总数判断测试点的位置属性
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					break;
				default:
					this->testedResult[i] = markflag;
				}
				continue;
			}//else
		}

		if (Equal(x1, p->x, 10e-10) == 1)   //平行Y轴
		{
			if (Equal(p->x,this->grid->boundingBox[0].x, 10e-6)==1
				||Equal(p->x,this->grid->boundingBox[1].x, 10e-6)==1)
			{
				this->testedResult[i] = markflag;
				continue;
			}
			else
			{
				y_segment_index = (cell_index[0] - 1)*this->grid->resolution[1] + cell_index[1];
				countNum = this->GetCountNumInSegment_noStat(p->y, y1, y_segment_index, 1);
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					break;
				default:
					this->testedResult[i] = markflag;
				}
				continue;
			}//else
		}//if

		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		FLTYPE k, B;//斜率和截距
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;

		while (cur_Rgp_edge_ref != NULL)
		{
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			double min_a = p1->x < p2->x ? p1->x : p2->x;
			double max_a = p1->x > p2->x ? p1->x : p2->x;
			double min_b = p1->y < p2->y ? p1->y : p2->y;
			double max_b = p1->y > p2->y ? p1->y : p2->y;
			switch (up_or_down)
			{
			case 0:
				if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b))
				{
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					continue;
				}//if
				else
				{
					//利用斜率计算交点坐标
					k = cur_Rgp_edge_ref->k;
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					if (temp_y_value >= p->y)
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						continue;
					}
					else
					{
						countNum++;
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					}
					break;
				}
				break;
			default:
				if (p->x <= min_a || p->x >= max_a || (p->y >= max_b && tempy >= max_b))
				{
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					continue;
				}//if
				else
				{
					//利用斜率计算交点坐标
					k = cur_Rgp_edge_ref->k;
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					if (temp_y_value <= p->y)
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						continue;
					}
					else
					{
						cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
						countNum++;
					}
					break;
				}
				break;
			}//switch
		}//while
		int num = 0;
		//根据网格线片段查找 start
		if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
		{
			num = 0;
		}
		else
		{
			if (up_or_down == 0)
			{
				x_segment_index = cell_index[2] - this->grid->resolution[0];
			}
			else
			{
				x_segment_index = cell_index[2];
			}
			num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
		}
		//根据网格线片段查找 end

		countNum = countNum + num;
		int mark = countNum % 2;
		switch (mark)
		{
		case 1:
			markflag = InvertFlag(markflag);
			this->testedResult[i] = markflag;
			break;
		default:
			this->testedResult[i] = markflag;
		}
		continue;
	} //for
}

//Group handle method
void GridPIP2D::G_RgpChekPoint_Segment_noStat()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);
		//确定被测点所在单元
		//int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		float cell_index[3];
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

		//图学学报 start
		int cell_flag = this->grid->cell[(int)cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		if (cell_flag == CELL_OUT)
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		if (cell_flag == CELL_IN)
		{
			this->testedResult[i] = CELL_IN;
			continue;
		}
		//如果网格单元中存在边，确定测试点所处网格，并将该点存储在该网格中，以便于后续分组
		if (cell_flag == CELL_SUSPECT)
		{
			//this->grid->cell[(int)cell_index[2]].TestPointIndex[this->grid->cell[(int)cell_index[2]].testPointCount++] = i;
			float px, py;
			px = cell_index[0] - (int)cell_index[0];
			py = cell_index[1] - (int)cell_index[1];
			this->grid->cell[(int)cell_index[2]].testPointIndex[this->grid->cell[(int)cell_index[2]].testPointCount].x = px;
			this->grid->cell[(int)cell_index[2]].testPointIndex[this->grid->cell[(int)cell_index[2]].testPointCount].y = py;
			this->grid->cell[(int)cell_index[2]].testPointIndex[this->grid->cell[(int)cell_index[2]].testPointCount++].testPiontIndex = i;
			int num = this->SuspectCellCount;
		}
	}//for

	//处理测试点落在含边网格中的情况
	int sub_cell_resolution;
	float cell_index_f[3]; //计算浮点数的坐标索引

	for (int cell_num=0;cell_num<this->grid->cellCount;cell_num++)
	{
		if (this->grid->cell[cell_num].cell_InofOut == CELL_SUSPECT)
		{
			switch (this->grid->cell[cell_num].testPointCount /100)
			{
			case 0:
				sub_cell_resolution = 2;
				break;
			case 1:
				sub_cell_resolution = 3;
				break;
			case 2:
				sub_cell_resolution = 4;
			default:
				sub_cell_resolution = 5;
				break;
			}
			for (int p_count = 0; p_count < this->grid->cell[cell_num].testPointCount; p_count++)
			{
				int point_index= this->grid->cell[cell_num].TestPointIndex[p_count];
				//计算测试点所在网格的
				//cell_index_f[3] = this->grid->locatePoint(this->testedPoint[point_index].x, this->testedPoint[point_index].y, cell_index_f[0], cell_index_f[2]);
				
			}
		}
	}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		//int point_index[3];
		//int x_segment_index, y_segment_index;
		//int up_or_down; //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		//double tempy;  //考虑预处理存储在网格单元中
		//FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;
		//if (p->y <= tempy1)
		//{
		//	tempy = tempy1 - half_cell_size[1];
		//	point_index[0] = cell_index[0];
		//	point_index[1] = cell_index[1];
		//	point_index[2] = cell_index[2] + cell_index[1];
		//	up_or_down = 0;
		//}//if
		//else
		//{
		//	tempy = tempy1 + half_cell_size[1];
		//	point_index[0] = cell_index[0];
		//	point_index[1] = cell_index[1] + 1;
		//	point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
		//	up_or_down = 1;
		//}//else
		////
		//FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		//FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;

		//int  markflag = this->RgpPoint[point_index[2]].flag; //获取网格点的属性
		//int countNum = 0;//记录交点数总数

		//if (Equal(y1, p->y, 10e-10) == 1)    //平行X轴
		//{
		//	if (Equal(p->y, this->grid->boundingBox[0].y, 10e-6) == 1
		//		|| Equal(p->y, this->grid->boundingBox[1].y, 10e-6) == 1)
		//	{
		//		this->testedResult[i] = markflag;
		//		continue;
		//	}//if
		//	else
		//	{
		//		if (up_or_down == 0)
		//		{
		//			x_segment_index = cell_index[2] - this->grid->resolution[0];
		//		}
		//		else
		//		{
		//			x_segment_index = cell_index[2];
		//		}
		//		countNum = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
		//		//根据交点总数判断测试点的位置属性
		//		int mark = countNum % 2;
		//		switch (mark)
		//		{
		//		case 1:
		//			markflag = InvertFlag(markflag);
		//			this->testedResult[i] = markflag;
		//			break;
		//		default:
		//			this->testedResult[i] = markflag;
		//		}
		//		continue;
		//	}//else
		//}

		//if (Equal(x1, p->x, 10e-10) == 1)   //平行Y轴
		//{
		//	if (Equal(p->x, this->grid->boundingBox[0].x, 10e-6) == 1
		//		|| Equal(p->x, this->grid->boundingBox[1].x, 10e-6) == 1)
		//	{
		//		this->testedResult[i] = markflag;
		//		continue;
		//	}
		//	else
		//	{
		//		y_segment_index = (cell_index[0] - 1)*this->grid->resolution[1] + cell_index[1];
		//		countNum = this->GetCountNumInSegment_noStat(p->y, y1, y_segment_index, 1);
		//		int mark = countNum % 2;
		//		switch (mark)
		//		{
		//		case 1:
		//			markflag = InvertFlag(markflag);
		//			this->testedResult[i] = markflag;
		//			break;
		//		default:
		//			this->testedResult[i] = markflag;
		//		}
		//		continue;
		//	}//else
		//}//if

		//GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		////此处得到网格单元中均含有边
		//Point2D* p1, *p2;
		//FLTYPE k, B;//斜率和截距
		//RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		//RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;

		//while (cur_Rgp_edge_ref != NULL)
		//{
		//	p1 = cur_Rgp_edge_ref->start_e;
		//	p2 = cur_Rgp_edge_ref->end_e;
		//	//确定最大最小值
		//	double min_a = p1->x < p2->x ? p1->x : p2->x;
		//	double max_a = p1->x > p2->x ? p1->x : p2->x;
		//	double min_b = p1->y < p2->y ? p1->y : p2->y;
		//	double max_b = p1->y > p2->y ? p1->y : p2->y;
		//	switch (up_or_down)
		//	{
		//	case 0:
		//		if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b))
		//		{
		//			cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//			continue;
		//		}//if
		//		else
		//		{
		//			//利用斜率计算交点坐标
		//			k = cur_Rgp_edge_ref->k;
		//			B = cur_Rgp_edge_ref->B;
		//			FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
		//			if (temp_y_value >= p->y)
		//			{
		//				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//				continue;
		//			}
		//			else
		//			{
		//				countNum++;
		//				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//			}
		//			break;
		//		}
		//		break;
		//	default:
		//		if (p->x <= min_a || p->x >= max_a || (p->y >= max_b && tempy >= max_b))
		//		{
		//			cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//			continue;
		//		}//if
		//		else
		//		{
		//			//利用斜率计算交点坐标
		//			k = cur_Rgp_edge_ref->k;
		//			B = cur_Rgp_edge_ref->B;
		//			FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
		//			if (temp_y_value <= p->y)
		//			{
		//				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//				continue;
		//			}
		//			else
		//			{
		//				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		//				countNum++;
		//			}
		//			break;
		//		}
		//		break;
		//	}//switch
		//}//while
		//int num = 0;
		////根据网格线片段查找 start
		//if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
		//{
		//	num = 0;
		//}
		//else
		//{
		//	if (up_or_down == 0)
		//	{
		//		x_segment_index = cell_index[2] - this->grid->resolution[0];
		//	}
		//	else
		//	{
		//		x_segment_index = cell_index[2];
		//	}
		//	num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
		//}
		////根据网格线片段查找 end

		//countNum = countNum + num;
		//int mark = countNum % 2;
		//switch (mark)
		//{
		//case 1:
		//	markflag = InvertFlag(markflag);
		//	this->testedResult[i] = markflag;
		//	break;
		//default:
		//	this->testedResult[i] = markflag;
		//}
		//continue;
	//} //for
}

//图学学报
//FLTYPE b  仅有来查询Y轴  由于可能是向上查询或者向下查询    添加了数据统计代码
int GridPIP2D::GetCountNumInSegment(FLTYPE a,FLTYPE b, int segment_index, int flag)
{
	int pointcount, num;
	num = 0;
	FLTYPE tempv = 0;
	//flag 标记位，0表示横轴，1表示纵轴
	this->Rgpstat.compareCount++;
	if (flag == 0)  //横轴
	{
		pointcount = this->RgpIntersectPointX[segment_index].pointCount;
		this->Rgpstat.compareCount++;
		switch (pointcount)
		{
		case 0:
		{
			return 0;
			break;
		}//case 0
		case 1:
		{
			tempv = this->RgpIntersectPointX[segment_index].xory_coord[0];
			if (this->Rgpstat.compareCount++&&tempv < a)
			{
				num++;
				this->Rgpstat.addCount++;
				return num;
			}
			else if (this->Rgpstat.compareCount++&&tempv > a)
			{
				return 0;
			}
			else  //tempv=a  需要判断交点两侧的边在网格线的同侧还是异侧
			{
				return 1;
			}
			break;
		}//case 1
		default:   //多于1个点
		{
			for (int i = 0; i < pointcount; i++)
			{
				this->Rgpstat.compareCount++;//for 循环的比较
				this->Rgpstat.addCount++;//统计加法次数 i++的加法
				tempv = this->RgpIntersectPointX[segment_index].xory_coord[i];
				if (this->Rgpstat.compareCount++&&a>tempv)
				{
					num++;
					this->Rgpstat.addCount++;//统计加法次数
				}
				else if(this->Rgpstat.compareCount++&&a<tempv)
				{
					continue;
				}
				else //tempv=a   需改进
				{
					num++;
					this->Rgpstat.addCount++;//统计加法次数
				}
			}//for
			this->Rgpstat.compareCount++;//for 循环的比较
			return num;
		}//default
		}//switch
	}//if
	else      //纵轴
	{
		pointcount = this->RgpIntersectPointY[segment_index].pointCount;
		this->Rgpstat.compareCount++;
		switch (pointcount)
		{
		case 0:
		{
			return 0;
			break;
		}//case 0
		case 1:
		{
			tempv = this->RgpIntersectPointY[segment_index].xory_coord[0];
			//if ((tempv<a&&tempv<b)||(tempv>a&&tempv>b) )
			if ((this->Rgpstat.compareCount++&&this->Rgpstat.compareCount++&&tempv < a&&tempv < b) ||
				(this->Rgpstat.compareCount++&&this->Rgpstat.compareCount++&&tempv > a&&tempv > b))
			{
				return 0;
			}
			//else if (tempv == a||tempv==b)
			else if ((this->Rgpstat.compareCount++&&tempv == a ) ||
				(this->Rgpstat.compareCount++&&tempv == b))
			{
				return 1;
			}
			else  //tempv=a  需要判断交点两侧的边在网格线的同侧还是异侧
			{
				return 1;
			}
			break;
		}//case 1
		default:
		{
			for (int i = 0; i < pointcount; i++)
			{
				this->Rgpstat.compareCount++;  //for循环的比较和加法
				this->Rgpstat.addCount++;//统计加法次数
				tempv = this->RgpIntersectPointY[segment_index].xory_coord[i];
				//if ((tempv < a&&tempv < b) || (tempv > a&&tempv > b))
				if ((this->Rgpstat.compareCount++&&this->Rgpstat.compareCount++&&tempv < a&&tempv < b) ||
					(this->Rgpstat.compareCount++&&this->Rgpstat.compareCount++&&tempv > a&&tempv > b))
				{
					continue;
				}
				//else if (tempv == a || tempv == b)
				else if ((this->Rgpstat.compareCount++&&tempv == a) ||
					(this->Rgpstat.compareCount++&&tempv == b))
				{
					num++;
					this->Rgpstat.addCount++;//统计加法次数
				}
				else //tempv=a   需改进
				{
					num++;
					this->Rgpstat.addCount++;//统计加法次数
				}
			}//for
			this->Rgpstat.compareCount++;//for 循环的比较
			return num;
		}//default
		}//switch
	}//else
}

//FLTYPE b  仅有来查询Y轴  由于可能是向上查询或者向下查询    无数据统计代码
int GridPIP2D::GetCountNumInSegment_noStat(FLTYPE a, FLTYPE b, int segment_index, int flag)
{
	int pointcount, num;
	num = 0;
	FLTYPE tempv = 0;
	//flag 标记位，0表示横轴，1表示纵轴
	if (flag == 0)  //横轴
	{
		pointcount = this->RgpIntersectPointX[segment_index].pointCount;
		switch (pointcount)
		{
		case 0:
		{
			return 0;
			break;
		}//case 0
		case 1:
		{
			tempv = this->RgpIntersectPointX[segment_index].xory_coord[0];
			if (tempv < a)
			{
				num++;
				return num;
			}
			else if (tempv > a)
			{
				return 0;
			}
			else  //tempv=a  需要判断交点两侧的边在网格线的同侧还是异侧
			{
				return 1;
			}
			break;
		}//case 1
		default: //多于1个点
		{
			for (int i = 0; i < pointcount; i++)
			{
				tempv = this->RgpIntersectPointX[segment_index].xory_coord[i];
				if (a > tempv)
				{
					num++;
				}
				else if (a < tempv)
				{
					continue;
				}
				else //tempv=a   需改进
				{
					num++;
				}
			}//for
			return num;
		}//default
		}//switch
	}//if
	else      //纵轴
	{
		pointcount = this->RgpIntersectPointY[segment_index].pointCount;
		switch (pointcount)
		{
		case 0:
		{
			return 0;
			break;
		}//case 0
		case 1:
		{
			tempv = this->RgpIntersectPointY[segment_index].xory_coord[0];
			if ((tempv < a&&tempv < b) || (tempv > a&&tempv > b))
			{
				return 0;
			}
			else if (tempv == a || tempv == b)
			{
				return 1;
			}
			else  //tempv=a  需要判断交点两侧的边在网格线的同侧还是异侧
			{
				return 1;
			}
			break;
		}//case 1
		default:
		{
			for (int i = 0; i < pointcount; i++)
			{
				tempv = this->RgpIntersectPointY[segment_index].xory_coord[i];
				if ((tempv < a&&tempv < b) || (tempv > a&&tempv > b))
				{
					continue;
				}
				else if (tempv == a || tempv == b)
				{
					num++;
				}
				else //tempv=a   需改进
				{
					num++;
				}
			}//for
			return num;
		}//default
		}//switch
	}//else
}

//图学学报 计算测试点的位置属性  根据二分法查找vector数组
void GridPIP2D::RgpCheckPoint()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);

		//剔除在包围盒外部的点
		if ((p->x < this->grid->boundingBox[0].x) ||
			(p->x > this->grid->boundingBox[1].x) ||
			(p->y < this->grid->boundingBox[0].y) ||
			(p->y > this->grid->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

		//图学学报 start
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		if (cell_flag == CELL_OUT)   // 如果网格单元为多边形外，直接返回测试点的位置属性为外
		{
			this->Rgpstat.outsideCellPointCount++;
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		if (cell_flag == CELL_IN)     //如果网格单元为多边形内，直接返回测试的位置属性为内
		{
			this->Rgpstat.insideCellPointCount++;
			this->testedResult[i] = CELL_IN;
			continue;
		}
		//调整垂线的趋向：向上或向下
		double tempy;//考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;
		if (p->y <= tempy1)
		{
			tempy = tempy1 - half_cell_size[1];
		}
		else
		{
			tempy = tempy1 + half_cell_size[1];
		}
		//
		Point2D *intersection_point=new Point2D();//垂直Y轴做垂线，交点
		intersection_point->x = p->x;
		intersection_point->y = tempy;
		//计算垂线交点临近点的一维索引
		int point_index[3];
		point_index[2] = this->ReturnPointCInGrid(p->x,tempy, point_index[0], point_index[1]);
		FLTYPE  x1 = this->grid->boundingBox[0].x + point_index[0] * this->grid->cellSize[0];
		int  markflag = this->RgpPoint[point_index[2]].flag;
		//处理奇异情况
		if (Equal(p->x, x1, 10e-6) == 1)
		{
			//Singular_times++;
			int num0 = TestEdgeSingularCase(p, point_index[0], point_index[1]);
			if (num0 == -1)
			{
				this->testedResult[i] = BORDER;
				continue; 
			}
			else
			{
				this->testedResult[i] = num0;
				continue;
			}
		}
		/////奇异情况完毕
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		int countNum = 0;//记录交点数总数
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;
		while (cur_Rgp_edge_ref != NULL)
		{
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			double min_a = p1->x < p2->x ? p1->x : p2->x;
			double max_a = p1->x > p2->x ? p1->x : p2->x;
			double min_b = p1->y < p2->y ? p1->y : p2->y;
			double max_b = p1->y > p2->y ? p1->y : p2->y;

			if ((Equal(p->x, min_a, 10e-6) == 1))
			{
				//测试线与边的顶点重合 判断该顶点连接的两条边是否在顶点的同侧（记为偶数）或异侧（记为奇数）未完待续
				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
				continue;

			}
			else if (p->x<min_a || p->x>max_a || (p->y < min_b&&tempy < min_b) || (p->y > max_b&&tempy > max_b))
			{
				this->Rgpstat.exclude_by_CoorContast++;
				cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
				continue;
			}
			else
			{
				//Count_Calculations_with_Edges++;
				bool mark0 = this->Intersect1(p1, p2, p, intersection_point);
				if (mark0 == true)
				{
					//计算交点坐标
					countNum++;
				}
			}
			cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
		}//while
		//查询横轴上的交点个数
		int num = ReturnIntersectionPointsNumY(p->x, x1, point_index[1] - 1);
		if (num != -1)
		{
			countNum = countNum + num;
		}
		else//点落在边与网格线的交点上
		{
			this->testedResult[i] = BORDER;
			continue;
		}
		if (countNum % 2 == 1)
		{
			markflag = InvertFlag(markflag);
			this->testedResult[i] = markflag;
			continue;
		}
		if (countNum % 2 == 0)
		{
			this->testedResult[i] = markflag;
			continue;
		}
		//图学学报 end
	} //for
}

//图学学报方法 GCP 计算多边形边与网格线的交点
//并分别存储在相应的数组中，并记录相应网格线的坐标，及该网格线片段上的点的计数
void GridPIP2D::getIntersectPointCoord()
{
	//
	this->RgpIntersectPointX = new RgpPointInSegment[this->grid->cellCount]; //创建存储横轴坐标的数组
	this->RgpIntersectPointY = new RgpPointInSegment[this->grid->cellCount];  //创建存储纵轴坐标的数组

	for (int i = 0; i < this->testedPolygon[curPolygonIndex].edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon[curPolygonIndex].edgeTable[i];
		Point2D * start_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->startIndex];     //边的起始点
		Point2D * end_vertex = &this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->endIndex];        //边的终点
		line->p1 = start_vertex;
		line->p2 = end_vertex;
		FLTYPE slope = line->CalculateSlope();
	}//for
}


//计算网格单元中点处的点包信息（纯内、纯外）
//健壮性调整时，设置为可疑
void GridPIP2D::setCellProp()
{
	GridCell2D * cur_cell = this->grid->cell;
	FLTYPE first_x = this->grid->boundingBox[0].x -( this->grid->cellSize[0]/ 2.0f); //最左侧的点的x坐标
	FLTYPE second_x = this->grid->boundingBox[0].x + (this->grid->cellSize[0] / 2.0f);  //最左侧单元的中心点x坐标
	FLTYPE first_y = this->grid->boundingBox[0].y + (this->grid->cellSize[1] / 2.0f); //最左侧单元的中心点y坐标
	FLTYPE x0, x1, x, y;
	x0 = first_x;
	x1 = second_x;
	y = first_y;
	int left_property = CELL_OUT;	//纯外
	EdgeRef2D * left_edge_ref = NULL;
	this->stat.prepInterCount = 0;      //统计预处理时求交计算的总次数

	for (int j = 0; j < this->grid->resolution[1]; j++) //纵坐标
	{
		for (int i = 0; i < this->grid->resolution[0]; i++)  //横坐标
		{
			//计算中点连线与两单元（左侧与当前）中的边的交点个数，
			//偶数则与左侧单元属性相同，奇数则与左侧单元属性相反
			int intersect_count = 0;
			EdgeRef2D * cur_edge_ref = left_edge_ref;

			//计算左侧单元内的边与中点连线段相交的数量
			while (cur_edge_ref)
			{
				Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
				Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
				Point2D p2, p3;
				p2.x = x0;
				p2.y = y;
				p3.x = x1;
				p3.y = y;

				intersect_count += isIntersect(p0, p1, &p2, &p3);	//有交为1，无交为0
				this->stat.prepInterCount++;
				cur_edge_ref = cur_edge_ref->next;
			}
			//加上当前单元内的边与中点连线段相交的数量，注意剔除重复的边
			cur_edge_ref = cur_cell->edgeRef;
			while (cur_edge_ref)
			{
				//判定是否重复
				EdgeRef2D * temp = left_edge_ref;
				bool duplicate = false;
				while (temp)
				{
					if (temp->e == cur_edge_ref->e)
					{
						duplicate = true;
						break;
					}
					else
						temp = temp->next;
				}
				if (duplicate)
				{
					cur_edge_ref = cur_edge_ref->next;
					continue;
				}

				Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
				Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
				Point2D p2, p3;
				p2.x = x0;
				p2.y = y;
				p3.x = x1;
				p3.y = y;
				//intersect_count += isIntersect(p0, p1, &p2, &p3);
				bool flag = isIntersect(p0, p1, &p2, &p3);//未考虑边重合的情况
				intersect_count += flag;
				this->stat.prepInterCount++;
				cur_edge_ref = cur_edge_ref->next;
			}

			if ((intersect_count % 2) == 0)	//偶数
				cur_cell->flag = left_property;
			else		//奇数
				cur_cell->flag = !left_property;

			//下一个单元
			x0 = x1;
			x1 += this->grid->cellSize[0];
			left_property = cur_cell->flag;
			left_edge_ref = cur_cell->edgeRef;
			cur_cell++;
		}
		//开始下一行
		x0 = first_x;
		x1 = second_x;
		y += this->grid->cellSize[1];
		left_property = CELL_OUT;
		left_edge_ref = NULL;
	}

	//判定中心点是否位于多边形的边、顶点处。用于健壮性调整。
	if (this->robustMode == PIP_MODE_ROBUST)
	{
		x = first_x+this->grid->cellSize[0];
		y = first_y;
		cur_cell = this->grid->cell;
		for (int j = 0; j < this->grid->resolution[1]; j++) //纵坐标
		{
			for (int i = 0; i < this->grid->resolution[0]; i++)  //横坐标
			{
				EdgeRef2D * cur_edge_ref = cur_cell->edgeRef;
				while (cur_edge_ref)
				{
					Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
					Point2D * p2 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
					/*
					//计算中心点到边的距离
					//直线方程
					FLTYPE A = p1->y - p2->y;
					FLTYPE B = p2->x - p1->x;
					FLTYPE C = p1->x*p2->y - p2->x*p1->y;
					FLTYPE d = (A*x + B*y + C) / sqrt(A*A+B*B);   //太耗时？？？
					if(abs(d) < PIP_ROBUST_EPSILON )
					{
						cur_cell->flag = CELL_SUSPECT;
						break;
					}
					*/

					//计算中心点到线段端点的距离的平方       （以纵轴网格线的中点为中心点，非网格的中点）
					FLTYPE d1 = sqrt((p1->x - x)*(p1->x - x) + (p1->y - y)*(p1->y - y));
					FLTYPE d2 = sqrt((p2->x - x)*(p2->x - x) + (p2->y - y)*(p2->y - y));
					FLTYPE d = sqrt((p2->x - p1->x)*(p2->x - p1->x) + (p2->y - p1->y)*(p2->y - p1->y));
					//if((d1+ d2 - d) < this->robustPrecision )
					if ((d1 + d2 - d) < (this->robustPrecision * this->grid->cellSize[0]))
					{
						cur_cell->flag = CELL_SUSPECT;
						break;
					}

					/*
					//用点积预判
					Point2D p1p2, p1p, p2p; //ab, ac, bc
					double distance;
					p1p2.x = p2->x - p1->x;
					p1p2.y = p2->y - p1->y;
					p1p.x = x - p1->x;
					p1p.y = y - p1->y;
					p2p.x = x - p2->x;
					p2p.y = y - p2->y;
					//p1p2点乘p1p (ab 点乘 ac)
					double k1 = p1p2.x*p1p.x + p1p2.y*p1p.y;
					//p1p2点乘p1p2 (ab 点乘 ab)
					double k2 = p1p2.x + p1p2.x + p1p2.y * p1p2.y;
					if(k1<= 0.0f)
						//p1p点乘p1p (ac 点乘 ac)
						distance = p1p.x * p1p.x + p1p.y * p1p.y;
					else if(k1 >= k2)
						//p2p点乘p2p (bc 点乘 bc)
						distance = p2p.x * p2p.x + p2p.y * p2p.y;
					else
						//(ac 点乘 ac) - k1 * k1 / k2
						distance =  (p1p.x * p1p.x + p1p.y * p1p.y) - k1*k1/k2;
					if(distance < (0.01*this->grid->cellSize[0]) )
					{
						cur_cell->flag = CELL_SUSPECT;
						break;
					}
					*/
					cur_edge_ref = cur_edge_ref->next;
				} //while
				x += this->grid->cellSize[0];
				cur_cell++;
			} //for i 
			x = first_x + this->grid->cellSize[0];
			y += this->grid->cellSize[1];
		} //for j
	}//if
}

//计算网格单元中点处的点包信息（纯内、纯外）
//健壮性调整时，设置为可疑
//一个中心点与其所在单元内的边的相交情况进计算一遍
void GridPIP2D::setCellProp_Fast()
{
	GridCell2D * cur_cell = this->grid->cell;
	FLTYPE first_x = this->grid->boundingBox[0].x - this->grid->cellSize[0]; //最左侧的点的x坐标
	FLTYPE second_x = this->grid->boundingBox[0].x + (this->grid->cellSize[0] / 2.0f);  //最左侧单元的中心点x坐标
	FLTYPE first_y = this->grid->boundingBox[0].y + (this->grid->cellSize[1] / 2.0f); //最左侧单元的中心点y坐标
	FLTYPE x0, x1, x, y;
	x0 = first_x;
	x1 = second_x;
	y = first_y;
	int left_property = CELL_OUT;	//纯外
	EdgeRef2D * left_edge_ref = NULL;
	this->stat.prepInterCount = 0;      //统计预处理时求交计算的总次数
	DynamicArray buffer;

	for (int j = 0; j < this->grid->resolution[1]; j++) //纵坐标
	{
		for (int i = 0; i < this->grid->resolution[0]; i++)  //横坐标
		{
			//计算中点连线与两单元（左侧与当前）中的边的交点个数，
			//偶数则与左侧单元属性相同，奇数则与左侧单元属性相反
			int intersect_count = 0;
			EdgeRef2D * cur_edge_ref = left_edge_ref;

			//计算左侧单元内的边与中点连线段相交的数量
			int edge_index = 0;
			while (cur_edge_ref)
			{
				Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
				Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
				Point2D p2, p3;
				p2.x = x0;
				p2.y = y;
				p3.x = x1;
				p3.y = y;
				//有交为1，无交为0，获取p2与边的位置关系
				intersect_count += isIntersect(p0, p1, &p2, &p3, buffer, edge_index, 0);

				this->stat.prepInterCount++;
				cur_edge_ref = cur_edge_ref->next;
				edge_index++;
			}
			//加上当前单元内的边与中点连线段相交的数量，注意剔除重复的边
			edge_index = 0;
			cur_edge_ref = cur_cell->edgeRef;
			while (cur_edge_ref)
			{
				//判定是否重复
				EdgeRef2D * temp = left_edge_ref;
				bool duplicate = false;
				while (temp)
				{
					if (temp->e == cur_edge_ref->e)
					{
						duplicate = true;
						break;
					}
					else
						temp = temp->next;
				}
				if (duplicate)
				{
					cur_edge_ref = cur_edge_ref->next;
					edge_index++;
					continue;
				}

				Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
				Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
				Point2D p2, p3;
				p2.x = x0;
				p2.y = y;
				p3.x = x1;
				p3.y = y;
				//有交为1，无交为0，记录p3与边的位置关系
				//intersect_count += isIntersect(p0, p1, &p2, &p3);
				bool flag = isIntersect(p0, p1, &p2, &p3, buffer, edge_index, 1);
				intersect_count += flag;
				this->stat.prepInterCount++;
				cur_edge_ref = cur_edge_ref->next;
				edge_index++;
			}

			if ((intersect_count % 2) == 0)	//偶数
				cur_cell->flag = left_property;
			else		//奇数
				cur_cell->flag = !left_property;

			//下一个单元
			x0 = x1;
			x1 += this->grid->cellSize[0];
			left_property = cur_cell->flag;
			left_edge_ref = cur_cell->edgeRef;
			cur_cell++;
		}

		x0 = first_x;
		x1 = second_x;
		y += this->grid->cellSize[1];
		left_property = CELL_OUT;
		left_edge_ref = NULL;
	}

	//判定中心点是否位于多边形的边、顶点处。用于健壮性调整。
	if (this->robustMode == PIP_MODE_ROBUST)
	{
		x = first_x;
		y = first_y;
		cur_cell = this->grid->cell;
		for (int j = 0; j < this->grid->resolution[1]; j++) //纵坐标
		{
			for (int i = 0; i < this->grid->resolution[0]; i++)  //横坐标
			{
				EdgeRef2D * cur_edge_ref = cur_cell->edgeRef;
				while (cur_edge_ref)
				{
					Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
					Point2D * p2 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
					//计算中心点到边的距离
					//直线方程
					FLTYPE A = p1->y - p2->y;
					FLTYPE B = p2->x - p1->x;
					FLTYPE C = p1->x*p2->y - p2->x*p1->y;
					FLTYPE d = (A*x + B * y + C) / sqrt(A*A + B * B);   //太耗时？？？
					//if(abs(d) < PIP_ROBUST_EPSILON )
					if (abs(d) < this->robustPrecision)
					{
						cur_cell->flag = CELL_SUSPECT;
						break;
					}
					cur_edge_ref = cur_edge_ref->next;
				} //while
				x += this->grid->cellSize[0];
				cur_cell++;
			} //for i 
			y += this->grid->cellSize[1];
		} //for j
	}//if	
}


int GridPIP2D::estimateRef(int U, int V, int style)
{
	//计算每条边平均占据的单元数量
	float cx = this->grid_size[0] / U;      //单元网格的长
	float cy = this->grid_size[1] / V;     //单元网格的宽
	float a, b, c;
	a = 1 + this->testedPolygon[curPolygonIndex].dx / cx;
	b = 1 + this->testedPolygon[curPolygonIndex].dy / cy;
	if (style == ESTIMATE_REF_STYLE_REDUNDANT)
		c = a + b + 2;
	else if (style == ESTIMATE_REF_STYLE_ACCURATE)
		c = a + b - 1;

	int totalReferenceCount = c * this->testedPolygon[curPolygonIndex].edgeCount;

	//返回总边指针数
	return totalReferenceCount;
}

void GridPIP2D::PIP()
{
	/*
	//分配测试结果存储数组
	if(this->testedResult != NULL)
	{
		delete [] this->testedResult;
		this->testedResult = NULL;
	}
	this->testedResult = new int[this->testedPointCount];
	*/

	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;
	this->stat.realInterCount = 0;
	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);

		//剔除在包围盒外部的点
		if ((p->x < this->grid->boundingBox[0].x) ||
			(p->x > this->grid->boundingBox[1].x) ||
			(p->y < this->grid->boundingBox[0].y) ||
			(p->y > this->grid->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//确定被测点所在单元
		int cell_index[3];
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

		//计算该单元的中心点位置
		Point2D middle;
		middle.x = this->grid->boundingBox[0].x + this->grid->cellSize[0] * cell_index[0] + half_cell_size[0];
		middle.y = this->grid->boundingBox[0].y + this->grid->cellSize[1] * cell_index[1] + half_cell_size[1];

		//计算middle和当前被测点的连线与当前单元中所有边的相交数
		int intersect_count = 0;
		EdgeRef2D * edge_ref = this->grid->cell[cell_index[2]].edgeRef;
		EdgeRef2D * cur_edge = edge_ref;
		while (cur_edge)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->endIndex]);
			Point2D p2, p3;
			p2.x = middle.x;
			p2.y = middle.y;
			p3.x = p->x;
			p3.y = p->y;
			intersect_count += isIntersect(p0, p1, &p2, &p3);	//有交为1，无交为0
			this->stat.realInterCount++;      //用于最佳分辨率的试验
			cur_edge = cur_edge->next;
		}

		//根据相交个数的奇偶性判定点的位置
		if (intersect_count % 2 == 0)
			this->testedResult[i] = this->grid->cell[cell_index[2]].flag;
		else
			this->testedResult[i] = !this->grid->cell[cell_index[2]].flag;
	}

	this->stat.realInterCount = this->stat.realInterCount / (float)this->testedPointCount;
}


void GridPIP2D::PIP_robust()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;  //计算网格单元长、宽的一半，用以计算中心点坐标
	half_cell_size[1] = this->grid->cellSize[1] / 2;
	this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;
	//统计数据
	this->stat.realInterCount = 0;
	this->stat.extraInterCount = 0;
	this->stat.nonemptyTPCount = 0;
	this->stat.extraCheckedCellCount = 0;
	this->stat.suspectTPCount = 0;

	this->Rgpstat.compareCount++;
	for (int i = 0; i < this->testedPointCount; i++)
	{
		this->Rgpstat.compareCount++; //统计for循环的加法和乘法次数
		this->Rgpstat.addCount++;

		Point2D * p = &(this->testedPoint[i]);

		//剔除在包围盒外部的点
		/*if ((p->x < this->grid->boundingBox[0].x) ||
			(p->x > this->grid->boundingBox[1].x) ||
			(p->y < this->grid->boundingBox[0].y) ||
			(p->y > this->grid->boundingBox[1].y))*/
		if ((this->Rgpstat.compareCount++&&p->x < this->grid->boundingBox[0].x) ||
			(this->Rgpstat.compareCount++&&p->x > this->grid->boundingBox[1].x) ||
			(this->Rgpstat.compareCount++&&p->y < this->grid->boundingBox[0].y) ||
			(this->Rgpstat.compareCount++&&p->y > this->grid->boundingBox[1].y))
		{
			this->Rgpstat.outoftheBoundingbox++;
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

		//统计上述函数中加法和乘法次数
		this->Rgpstat.addCount = this->Rgpstat.addCount + 5;
		this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 3;

		//计算可用参考点的坐标
 		Point2D middle;
		middle.y = this->grid->boundingBox[0].y + this->grid->cellSize[1] * cell_index[1] + half_cell_size[1];
		this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
		this->Rgpstat.multiplicationCount++;
		int x_end = cell_index[0];
		int target = cell_index[2];
		this->Rgpstat.compareCount = this->Rgpstat.compareCount + 2;
		while (this->grid->cell[target].flag == CELL_SUSPECT && x_end < this->grid_res[0])
		{
			target++;
			x_end++;
			this->Rgpstat.addCount = this->Rgpstat.addCount + 2;//加法次数
			this->Rgpstat.compareCount = this->Rgpstat.compareCount + 2;
		}
		middle.x = this->grid->boundingBox[0].x + this->grid->cellSize[0] * x_end + half_cell_size[0];
		this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
		this->Rgpstat.multiplicationCount++;
		this->Rgpstat.compareCount++;
		if (x_end == this->grid_res[0])
		{
			middle.x += this->grid->cellSize[0];
			target--;
			this->Rgpstat.addCount = this->Rgpstat.addCount + 2;//加法次数
		}

		//计算middle和当前被测点的连线与一系列单元中所有边的相交数
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		this->Rgpstat.compareCount++;
		if (cur_cell->edgeRef != NULL)
		{
			this->stat.nonemptyTPCount++;
			this->Rgpstat.inEdgeCellPointCount++;
		}
		else
		{
			this->Rgpstat.noEdgeCellPointCount++;
		}
		this->Rgpstat.compareCount++;
		if (cur_cell->flag == CELL_SUSPECT)
			this->stat.suspectTPCount++;

		int intersect_count = 0;
		Point2D p2, p3;
		p2.x = middle.x;
		p2.y = middle.y;
		p3.x = p->x;
		p3.y = p->y;
		this->Rgpstat.compareCount++;
		for (int k = cell_index[2]; k <= target; k++, cur_cell++)
		{
			this->Rgpstat.compareCount++;
			this->Rgpstat.addCount = this->Rgpstat.addCount + 2;

			EdgeRef2D * edge_ref = cur_cell->edgeRef;
			EdgeRef2D * cur_edge = edge_ref;
			this->Rgpstat.compareCount++;
			while (cur_edge)
			{
				this->Rgpstat.compareCount++;
				Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->startIndex]);
				Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->endIndex]);
				//判断点是否落在多边形边上
				bool flag0= this->JudgeCollineation(p0,p1,p);

				this->Rgpstat.addCount = this->Rgpstat.addCount + 5;
				this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;
				this->Rgpstat.compareCount= this->Rgpstat.compareCount+10; //JudgeCollineation包含9次比较
				
				if (flag0 == TRUE)
				{
					this->Rgpstat.ontheEdgeCount++;
					this->testedResult[i] = BORDER;
					cur_edge = cur_edge->next;
					continue;
				}
				//判断测试点与中心点连线是否与多边形边共线
				flag0 = this->JudgeLineSuperposition(p0, p1, p, &p2);
				this->Rgpstat.compareCount++;
				if (flag0 == TRUE)
				{
					this->Rgpstat.superpositionCount++;
					//计算共线时，交点的个数
					int num = 0;
					this->Rgpstat.compareCount++;
					switch (num)
					{
					case 0:
						this->testedResult[i] = this->grid->cell[target].flag;
						cur_edge = cur_edge->next;
						continue;
						break;
					case 1:
						this->testedResult[i] = !this->grid->cell[target].flag;
						cur_edge = cur_edge->next;
						continue;
						break;
					default:
						this->testedResult[i] = BORDER;
						cur_edge = cur_edge->next;
						continue;
					}
				}
				//正常情况，不共线，点不在多边形边上
				this->Rgpstat.cross_product_Count++;
				intersect_count += isIntersect(p0, p1, &p2, &p3);	//有交为1，无交为0
				//统计isIntersect的加法、乘法、比较次数  
				this->Rgpstat.addCount = this->Rgpstat.addCount + 17;
				this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 8;
				this->Rgpstat.compareCount = this->Rgpstat.compareCount + 6;

				this->stat.realInterCount++;      //用于最佳分辨率的试验
				this->Rgpstat.compareCount++;
				if (k != cell_index[2])
					this->stat.extraInterCount++;	//count the extra intersection tests
				cur_edge = cur_edge->next;
			}//while
			if (k != cell_index[2])
				this->stat.extraCheckedCellCount++;
		}

		//根据相交个数的奇偶性判定点的位置
		this->Rgpstat.compareCount++;
		this->Rgpstat.multiplicationCount++;
		if (intersect_count % 2 == 0)
			this->testedResult[i] = this->grid->cell[target].flag;
		else
			this->testedResult[i] = !this->grid->cell[target].flag;
	} //for
	//this->stat.realInterCount = this->stat.realInterCount / (float)this->testedPointCount;
}

bool GridPIP2D::JudgeCollineation(Point2D* p1, Point2D* p2, Point2D* q)
{
	double v1, v2;
	v1 = (q->x - p1->x)*(p1->y - p2->y);
	v2 = (p1->x - p2->x)*(q->y - p1->y);
	if ((fabs(v1 - v2) < eps)
		&& (q->x >= min(p1->x, p2->x) && q->x <= max(p1->x, p2->x))
		&& (q->y >= min(p1->y, p2->y)) && (q->y <= max(p1->y, p2->y)))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
bool GridPIP2D::JudgeLineSuperposition(Point2D* A1, Point2D* A2, Point2D* B1, Point2D* B2)
{
	double T1 = cross(A1, A2, B1);//包含5次加法，2次乘法
	double T2 = cross(A1, A2, B2);
	double T3 = cross(B1, B2, A1);
	double T4 = cross(B1, B2, A2);
	this->Rgpstat.addCount = this->Rgpstat.addCount + 20;
	this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 8;

	if ((this->Rgpstat.compareCount++&&this->Rgpstat.multiplicationCount++&&
		(T1 * T2) > 0) || 
		(this->Rgpstat.compareCount++ && this->Rgpstat.multiplicationCount++ && 
		(T3 * T4) > 0))
	{   // 一条线段的两个端点在另一条线段的同侧，不相交。（可能需要额外处理以防止乘法溢出，视具体情况而定。）
		return false;
	}
	else if (this->Rgpstat.compareCount++&&
		this->Rgpstat.compareCount++&&
		T1 == 0 && T2 == 0)
	{   // 两条线段共线，利用快速排斥实验进一步判断。此时必有 T3 == 0 && T4 == 0。
		return rectsIntersect(A1, A2, B1, B2);// 12次比较
		this->Rgpstat.compareCount = this->Rgpstat.compareCount + 12;
	}
	else
	{   // 其它情况，两条线段相交。
		return false;//此处是为了判断两条线段是否共线，若是为了判断是否相交，则改为true
	}
}
double GridPIP2D::cross(Point2D* A, Point2D* B, Point2D* C)
{
	double cross1 = (C->x - A->x) * (B->y - A->y);
	double cross2 = (C->y - A->y) * (B->x - A->x);
	return (cross1 - cross2);
}
bool GridPIP2D::rectsIntersect(Point2D* S1, Point2D* E1, Point2D* S2, Point2D* E2)
{
	if (min(S1->y, E1->y) <= max(S2->y, E2->y) &&
		max(S1->y, E1->y) >= min(S2->y, E2->y) &&
		min(S1->x, E1->x) <= max(S2->x, E2->x) &&
		max(S1->x, E1->x) >= min(S2->x, E2->x))
	{
		return true;
	}
	return false;
}

//生成均匀随机分布的被测点
//type=0： 等间隔点
//type=1：随机点
void GridPIP2D::generateTestedPoint(int type, int xcount, int ycount)
{
	if (this->testedPolygon == NULL)
		return;

	//分配被测点数组
	if (this->testedPoint != NULL)
		delete[] testedPoint;
	testedPoint = new Point2D[testedPointCount];

	//分配测试结果存储数组
	if (this->testedResult != NULL)
	{
		delete[] this->testedResult;
		this->testedResult = NULL;
	}
	this->testedResult = new int[this->testedPointCount];

	//被测点赋值
	if (type == 0)    //等间隔点
	{
		int index = 0;
		float stepx = (this->testedPointRegion[1].x - this->testedPointRegion[0].x) / xcount;
		float stepy = (this->testedPointRegion[1].y - this->testedPointRegion[0].y) / ycount;;
		for (int i = 0; i < xcount; i++)
		{
			float curx = this->testedPointRegion[0].x + stepx * i;
			for (int j = 0; j < ycount; j++)
			{
				this->testedPoint[index].x = curx;
				this->testedPoint[index].y = this->testedPointRegion[0].y + stepy * j;
				index++;
			}
		}
	}
	else if (type == 1)
	{
		for (int i = 0; i < this->testedPointCount; i++)
		{
			//this->testedPoint[i].x = get_random(this->testedPolygon[curPolygonIndex].boundingBox[0].x, this->testedPolygon[curPolygonIndex].boundingBox[1].x);
			//this->testedPoint[i].y = get_random(this->testedPolygon[curPolygonIndex].boundingBox[0].y, this->testedPolygon[curPolygonIndex].boundingBox[1].y);
			this->testedPoint[i].x = get_random(this->testedPointRegion[0].x, this->testedPointRegion[1].x);
			this->testedPoint[i].y = get_random(this->testedPointRegion[0].y, this->testedPointRegion[1].y);
		}
	}
}


FLTYPE GridPIP2D::get_random(FLTYPE start, FLTYPE end)
{
	FLTYPE num = rand() / (FLTYPE)RAND_MAX;
	if (start == 0)
		num = num * end;
	else
		num = start + (end - start)*num;
	return num;
}

//统计空间开销
void GridPIP2D::calcStorageCost(int U, int V, int * s_basic, int * s_grid)
{
	if (this->testedPolygon == NULL)
		return;

	//存储多边形、被测点开销
	*s_basic = sizeof(struct GCPPolygon2D) +
		sizeof(struct Point2D)*this->testedPolygon->vertexCount +
		sizeof(struct Edge2D)*this->testedPolygon->edgeCount +
		sizeof(struct Point2D)*this->testedPointCount;

	//网格辅助结构
	*s_grid = sizeof(struct Grid2D) +
		sizeof(struct GridCell2D)*U*V +
		sizeof(struct EdgeRef2D)*this->estimateRef(U, V, ESTIMATE_REF_STYLE_ACCURATE);
}

//Report the statistics in detailed mode
void GridPIP2D::printDetailedStatistics(FILE * fp, TCHAR * plyname)
{
	//基本信息
	char timebuf[128], datebuf[128];
	_strtime_s(timebuf);
	_strdate_s(datebuf);
	fprintf(fp, "\n\n-----------------------%s   %s-----------------------\n", datebuf, timebuf);
	fprintf(fp, "Polygon filename: %s\n", plyname);
	fprintf(fp, "Polygon edges #: %d\n", this->testedPolygon->edgeCount);
	fprintf(fp, "Tested points #: %d\n", this->testedPointCount);
	fprintf(fp, "Test mode: ");
	if (this->robustMode == PIP_MODE_NORMAL)
		fprintf(fp, "%s\n", "Non-Robust mode.");
	else if (this->robustMode == PIP_MODE_ROBUST)
		fprintf(fp, "%s\n", "Robust mode.");
	fprintf(fp, "\n");

	//网格信息
	fprintf(fp, "Grid resolution: %d,%d \n", this->grid->resolution[0], this->grid->resolution[1]);
	fprintf(fp, "Grid size: %f, %f\n", this->grid_size[0], this->grid_size[1]);
	int empty_cell_count = 0;
	int suspect_cell_count = 0;
	int in_cell_count = 0;
	int out_cell_count = 0;
	for (int i = 0; i < this->grid->cellCount; i++)
	{
		if (this->grid->cell[i].edgeCount == 0)
			empty_cell_count++;
		if (this->grid->cell[i].flag == CELL_IN)
			in_cell_count++;
		if (this->grid->cell[i].flag == CELL_OUT)
			out_cell_count++;
		if (this->grid->cell[i].flag == CELL_SUSPECT)
			suspect_cell_count++;
	}
	fprintf(fp, "Inside cells #: %d\n", in_cell_count);
	fprintf(fp, "Outside cells #: %d\n", out_cell_count);
	fprintf(fp, "Suspect cells #: %d\n", suspect_cell_count);
	fprintf(fp, "Empty cells #: %d\n", empty_cell_count);
	fprintf(fp, "\n");

	//时间信息
	fprintf(fp, "Estimated / real preprocessing time: %5.9f / %5.9f\n", this->stat.estimateInitTime, this->stat.realInitTime);
	fprintf(fp, "Estimated / real inclusion test time %5.9f / %5.9f\n", this->stat.estimateTestTime, this->stat.realTestTime);
	fprintf(fp, "Estimated / real total time %5.9f / %5.9f\n",
		this->stat.estimateInitTime + this->stat.estimateTestTime,
		this->stat.realInitTime + this->stat.realTestTime);
	//fprintf(f, "Inclusion time per tested point: %5.9f seconds.\n", this->realTestTime/this->testedPointCount);
	fprintf(fp, "\n");

	//空间信息
	fprintf(fp, "Storage for normal information: %d\n", this->stat.storageNormal);
	fprintf(fp, "Storage for auxiliary information: %d\n", this->stat.storageAS);
	fprintf(fp, "Total storage: %d\n", this->stat.storageAS + this->stat.storageAS);
	fprintf(fp, "\n");

	//参数信息
	fprintf(fp, "Average edge length: %f, %f\n", this->testedPolygon->dx, this->testedPolygon->dy);
	fprintf(fp, "Average reference # per cell: %f\n", (float)this->stat.realRefCount / (this->grid_res[0] * this->grid_res[1]));
	fprintf(fp, "Average visited cells # per empty cells: %f\n", 2 * this->grid->cellCount / (float)empty_cell_count);
	fprintf(fp, "Total intersection test # in preprocessing: %d\n", this->stat.prepInterCount);
	int M0 = (this->grid_size[0] / this->testedPolygon->dx)*(this->grid_size[1] / this->testedPolygon->dy);
	fprintf(fp, "M0: %d\n", M0);
	int Mi;
	if (M0 < this->testedPolygon->edgeCount)
		Mi = M0;
	else
		Mi = this->testedPolygon->edgeCount;
	fprintf(fp, "Mi: %d\n", Mi);
	int Ma;
	if (M0 < this->testedPolygon->edgeCount)
		Ma = this->testedPolygon->edgeCount*this->testedPolygon->edgeCount / M0;
	else
		Ma = this->testedPolygon->edgeCount;
	fprintf(fp, "Ma: %d\n", Ma);
	fprintf(fp, "\n");

	//结果信息
	fprintf(fp, "INSIDE/OUTSIDE points #: %d / %d \n", this->stat.insidePointCount, this->stat.outsidePointCount);

	fclose(fp);
}

void GridPIP2D::printStatistics(char *log_filename, int U, int V, int style)
{
	if (log_filename == NULL)
		return;
	FILE * fp;
	fopen_s(&fp, log_filename, "a");
	if (fp == NULL)
		return;
	/*
	if(style == STATISTICS_STYLE_RECORD)
	{
		fprintf(fp, "%d %d %d %d ",
			this->testedPointCount,
			this->testedPolygon->edgeCount,
			this->grid->resolution[0],
			this->grid->resolution[1]);
		fprintf(fp, "%f %f %f %f %f %f\n",
			this->grid->gridSize[0],
			this->grid->gridSize[1],
			this->testedPolygon->dx,
			this->testedPolygon->dy,
			this->stat->realInitTime,
			this->stat->realTestTime);
		fclose(fp);
		return;
	}

	if(style == STATISTICS_STYLE_EXPERIMENT)
	{
		fprintf(fp, "%d %d %d ", U*V, U, V);
		fprintf(fp, "%5.9f %5.9f %5.9f %5.9f %5.9f %5.9f ",
			this->stat->estimateInitTime,
			this->stat->realInitTime,
			this->stat->estimateTestTime,
			this->stat->realTestTime,
			this->stat->estimateInitTime + this->stat->estimateTestTime,
			this->stat->realInitTime + this->stat->realTestTime);
		fprintf(fp,"%d %d %f %f ",
			this->stat->estimateRefCount,
			this->stat->realRefCount,
			this->stat->estimateInterCount,
			this->stat->realInterCount);
		fprintf(fp,"%d %d %d\n",
			this->stat->storageNormal,
			this->stat->storageGrid,
			this->stat->storageNormal + this->stat->storageGrid);
		fclose(fp);
		return;
	}
	*/
	if (style == STATISTICS_STYLE_DETAIL)
	{
		//基本信息
		char timebuf[128], datebuf[128];
		_strtime_s(timebuf);
		_strdate_s(datebuf);
		fprintf(fp, "\n\n-----------------------%s   %s-----------------------\n", datebuf, timebuf);
		//fprintf(fp, "Polygon filename: %s\n", this->polygon_file_name);
		fprintf(fp, "Polygon edges #: %d\n", this->testedPolygon->edgeCount);
		fprintf(fp, "Tested points #: %d\n", this->testedPointCount);
		fprintf(fp, "Test mode: ");
		if (this->robustMode == PIP_MODE_NORMAL)
			fprintf(fp, "%s\n", "Non-Robust mode.");
		else if (this->robustMode == PIP_MODE_ROBUST)
			fprintf(fp, "%s\n", "Robust mode.");
		fprintf(fp, "\n");

		//网格信息
		fprintf(fp, "Grid resolution: %d,%d \n", this->grid->resolution[0], this->grid->resolution[1]);
		fprintf(fp, "Grid size: %f, %f\n", this->grid_size[0], this->grid_size[1]);
		int empty_cell_count = 0;
		int suspect_cell_count = 0;
		int in_cell_count = 0;
		int out_cell_count = 0;
		for (int i = 0; i < this->grid->cellCount; i++)
		{
			if (this->grid->cell[i].edgeCount == 0)
				empty_cell_count++;
			if (this->grid->cell[i].flag == CELL_IN)
				in_cell_count++;
			if (this->grid->cell[i].flag == CELL_OUT)
				out_cell_count++;
			if (this->grid->cell[i].flag == CELL_SUSPECT)
				suspect_cell_count++;
		}
		fprintf(fp, "Inside cells #: %d\n", in_cell_count);
		fprintf(fp, "Outside cells #: %d\n", out_cell_count);
		fprintf(fp, "Suspect cells #: %d\n", suspect_cell_count);
		fprintf(fp, "Empty cells #: %d\n", empty_cell_count);
		fprintf(fp, "\n");

		//时间信息
		fprintf(fp, "Estimated / real preprocessing time: %5.9f / %5.9f\n", this->stat.estimateInitTime, this->stat.realInitTime);
		fprintf(fp, "Estimated / real inclusion test time %5.9f / %5.9f\n", this->stat.estimateTestTime, this->stat.realTestTime);
		fprintf(fp, "Estimated / real total time %5.9f / %5.9f\n",
			this->stat.estimateInitTime + this->stat.estimateTestTime,
			this->stat.realInitTime + this->stat.realTestTime);
		//fprintf(f, "Inclusion time per tested point: %5.9f seconds.\n", this->realTestTime/this->testedPointCount);
		fprintf(fp, "\n");

		//空间信息
		fprintf(fp, "Storage for normal information: %d\n", this->stat.storageNormal);
		fprintf(fp, "Storage for auxiliary information: %d\n", this->stat.storageAS);
		fprintf(fp, "Total storage: %d\n", this->stat.storageAS + this->stat.storageNormal);
		fprintf(fp, "\n");

		//参数信息
		fprintf(fp, "Average edge length: %f, %f\n", this->testedPolygon->dx, this->testedPolygon->dy);
		fprintf(fp, "Average reference # per cell: %f\n", (float)this->stat.realRefCount / (this->grid_res[0] * this->grid_res[1]));
		fprintf(fp, "Average visited cells # per empty cells: %f\n", 2 * this->grid->cellCount / (float)empty_cell_count);
		fprintf(fp, "Total intersection test # in preprocessing: %d\n", this->stat.prepInterCount);
		int M0 = (this->grid_size[0] / this->testedPolygon->dx)*(this->grid_size[1] / this->testedPolygon->dy);
		fprintf(fp, "M0: %d\n", M0);
		int Mi;
		if (M0 < this->testedPolygon->edgeCount)
			Mi = M0;
		else
			Mi = this->testedPolygon->edgeCount;
		fprintf(fp, "Mi: %d\n", Mi);
		int Ma;
		if (M0 < this->testedPolygon->edgeCount)
			Ma = this->testedPolygon->edgeCount*this->testedPolygon->edgeCount / M0;
		else
			Ma = this->testedPolygon->edgeCount;
		fprintf(fp, "Ma: %d\n", Ma);
		fprintf(fp, "\n");

		//结果信息
		fprintf(fp, "INSIDE/OUTSIDE points #: %d / %d \n", this->stat.insidePointCount, this->stat.outsidePointCount);

		fclose(fp);
		return;
	}
}


//Generate a series of  changing polygons, used by dynamic PIP test demo
//Only support fractal triangle now
void GridPIP2D::generateDynamicPolygons()
{
	// TODO: Add your command handler code here
	int i, j;

	//set polygons
	this->testedPolygonCount = 11;
	this->testedPolygon = new GCPPolygon2D[this->testedPolygonCount];

	for (i = 0; i < this->testedPolygonCount; i++)
	{
		//set vertex table
		this->testedPolygon[i].vertexCount = 12;
		this->testedPolygon[i].vertexTable = new Point2D[this->testedPolygon[i].vertexCount];

		this->testedPolygon[i].vertexTable[0].x = 0.0;
		this->testedPolygon[i].vertexTable[0].y = 0.0;

		this->testedPolygon[i].vertexTable[1].x = 1.0;
		this->testedPolygon[i].vertexTable[1].y = 0.0;

		this->testedPolygon[i].vertexTable[2].x = 1.5;
		this->testedPolygon[i].vertexTable[2].y = -0.0866*i;

		this->testedPolygon[i].vertexTable[3].x = 2.0;
		this->testedPolygon[i].vertexTable[3].y = 0.0;

		this->testedPolygon[i].vertexTable[4].x = 3.0;
		this->testedPolygon[i].vertexTable[4].y = 0.0;

		this->testedPolygon[i].vertexTable[5].x = 2.25 + 0.075*i;
		this->testedPolygon[i].vertexTable[5].y = 1.299 + 0.0433*i;

		this->testedPolygon[i].vertexTable[6].x = 2.5;
		this->testedPolygon[i].vertexTable[6].y = 0.866;

		this->testedPolygon[i].vertexTable[7].x = 2.0;
		this->testedPolygon[i].vertexTable[7].y = 1.732;

		this->testedPolygon[i].vertexTable[8].x = 1.5;
		this->testedPolygon[i].vertexTable[8].y = 2.598;

		this->testedPolygon[i].vertexTable[9].x = 1.0;
		this->testedPolygon[i].vertexTable[9].y = 1.732;

		this->testedPolygon[i].vertexTable[10].x = 0.75 - 0.075*i;
		this->testedPolygon[i].vertexTable[10].y = 1.299 + 0.0433*i;

		this->testedPolygon[i].vertexTable[11].x = 0.5;
		this->testedPolygon[i].vertexTable[11].y = 0.866;

		//set edge table
		this->testedPolygon[i].edgeCount = 12;
		this->testedPolygon[i].edgeTable = new Edge2D[this->testedPolygon[i].edgeCount];
		for (j = 0; j < this->testedPolygon[i].edgeCount; j++)
		{
			this->testedPolygon[i].edgeTable[j].startIndex = j;
			this->testedPolygon[i].edgeTable[j].endIndex = (j + 1) % this->testedPolygon[i].edgeCount;
		}
		/*
				//set bounding box
				this->testedPolygon[i].boundingBox[0].x = 0.0;
				this->testedPolygon[i].boundingBox[0].y = this->testedPolygon[i].vertexTable[2].y;
				this->testedPolygon[i].boundingBox[1].x = 3.0;
				this->testedPolygon[i].boundingBox[1].y = 2.598;

				//compute dx, dy
				this->testedPolygon[i].dx = 0;
				this->testedPolygon[i].dy = 0;
				for(j=0; j<this->testedPolygon[i].edgeCount; j++)
				{
					int s = this->testedPolygon[i].edgeTable[j].startIndex;
					int e = this->testedPolygon[i].edgeTable[j].endIndex;
					this->testedPolygon[i].dx += this->testedPolygon[i].vertexTable[e].x - this->testedPolygon[i].vertexTable[s].x;
					this->testedPolygon[i].dy += this->testedPolygon[i].vertexTable[e].y - this->testedPolygon[i].vertexTable[s].y;
				}
				this->testedPolygon[i].dx /= 12;
				this->testedPolygon[i].dy /= 12;
		*/
	} //for
}

/*
for(int i=0; i<this->testedPolygon->edgeCount; i++)
{
	Edge2D * cur_edge = this->testedPolygon->edgeTable[i];
	this->quadtree->append(cur_edge);
}
l = this->testedPolygon->boundingBox[0].x;
	r = this->testedPolygon->boundingBox[1].x;
	b = this->testedPolygon->boundingBox[0].y;
	u = this->testedPolygon->boundingBox[1].y;

*/

//create quadtree
int GridPIP2D::createQuadTree()
{
	if (this->quadtree != NULL)
	{
		delete this->quadtree;
		this->quadtree = NULL;
	}
	this->asType = AS_TYPE_QUADTREE;

	//create edge reference list for all edges
	EdgeRef2D * dest = NULL;
	for (int i = 0; i < this->testedPolygon->edgeCount; i++)
	{
		Edge2D * cur_edge = &this->testedPolygon->edgeTable[i];
		EdgeRef2D * new_er = new EdgeRef2D;
		new_er->e = cur_edge;
		new_er->next = dest;
		dest = new_er;
	}

	//select a outside reference point
	Point2D reference;
	reference.x = this->testedPolygon->boundingBox[1].x * 1.1;
	reference.y = (this->testedPolygon->boundingBox[0].y + this->testedPolygon->boundingBox[1].y) * 0.5;

	//create recursively
	this->stat.prepInterCount = 0;
	QuadTreeNode * root = this->createQuadTreeRecursive(dest,
		this->testedPolygon->boundingBox[0].x,
		this->testedPolygon->boundingBox[1].x,
		this->testedPolygon->boundingBox[0].y,
		this->testedPolygon->boundingBox[1].y,
		0,
		reference,
		CELL_OUT);
	if (root == NULL)
		return -1;
	else
	{
		this->quadtree = new QuadTree;
		this->quadtree->root = root;
		return 1;
	}
}

//Create quadtree recursively
QuadTreeNode * GridPIP2D::createQuadTreeRecursive(EdgeRef2D * er,
	FLTYPE l,
	FLTYPE r,
	FLTYPE b,
	FLTYPE u,
	int level,
	Point2D ref_point,
	int ref_point_prop)
{
	QuadTreeNode * node = new QuadTreeNode;
	node->boundingbox[0].x = l;
	node->boundingbox[1].x = r;
	node->boundingbox[0].y = b;
	node->boundingbox[1].y = u;
	Point2D cp;
	cp.x = (l + r)*0.5;
	cp.y = (u + b)*0.5;
	int inter_count = 0;

	//create edge refernce list
	EdgeRef2D * cur_source = er;
	EdgeRef2D * dest = NULL;
	int edge_count = 0;
	while (cur_source)
	{
		//bounding box determination
		Point2D * start = &this->testedPolygon->vertexTable[cur_source->e->startIndex];
		Point2D * end = &this->testedPolygon->vertexTable[cur_source->e->endIndex];

		//determine if current edge intersect the tested segment
		inter_count += this->isIntersect(&cp, &ref_point, start, end);
		this->stat.prepInterCount++;

		Point2D min, max;
		if (start->x < end->x)
		{
			min.x = start->x;
			max.x = end->x;
		}
		else
		{
			min.x = end->x;
			max.x = start->x;
		}
		if (start->y < end->y)
		{
			min.y = start->y;
			max.y = end->y;
		}
		else
		{
			min.y = end->y;
			max.y = start->y;
		}
		if (max.x<l || min.x>r || max.y<b || min.y>u)
		{
			cur_source = cur_source->next;
			continue;
		}

		//intersect, add new edge reference
		edge_count++;
		EdgeRef2D * new_er = new EdgeRef2D;
		new_er->e = cur_source->e;
		new_er->next = dest;
		dest = new_er;
		cur_source = cur_source->next;
	}

	int cp_prop;
	if (inter_count % 2 == 0)
		cp_prop = ref_point_prop;
	else
		cp_prop = (!ref_point_prop);

	//return leaf node
	if (level == this->maxDepth || edge_count < this->maxEdgeCount)
	{
		node->flag |= QUADTREE_LEAF_NODE;
		node->flag |= cp_prop;
		node->edgeRef = dest;
		return node;
	}

	//create recursively
	FLTYPE subnode_l, subnode_r, subnode_b, subnode_u;
	node->flag |= QUADTREE_INTERNAL_NODE;

	//left bottom subnode
	subnode_l = l;
	subnode_r = (l + r)*0.5;
	subnode_b = b;
	subnode_u = (b + u)*0.5;
	node->subnode[0] = createQuadTreeRecursive(dest, subnode_l, subnode_r, subnode_b, subnode_u, level + 1, cp, cp_prop);

	//right bottom subnode
	subnode_l = (l + r)*0.5;
	subnode_r = r;
	subnode_b = b;
	subnode_u = (b + u)*0.5;
	node->subnode[1] = createQuadTreeRecursive(dest, subnode_l, subnode_r, subnode_b, subnode_u, level + 1, cp, cp_prop);

	//left up subnode
	subnode_l = l;
	subnode_r = (l + r)*0.5;
	subnode_b = (b + u)*0.5;
	subnode_u = u;
	node->subnode[2] = createQuadTreeRecursive(dest, subnode_l, subnode_r, subnode_b, subnode_u, level + 1, cp, cp_prop);

	//right up subnode
	subnode_l = (l + r)*0.5;
	subnode_r = r;
	subnode_b = (b + u)*0.5;
	subnode_u = u;
	node->subnode[3] = createQuadTreeRecursive(dest, subnode_l, subnode_r, subnode_b, subnode_u, level + 1, cp, cp_prop);

	//delete current edge reference list
	EdgeRef2D * ptr1, *ptr2;
	ptr1 = dest;
	while (ptr1)
	{
		ptr2 = ptr1;
		ptr1 = ptr1->next;
		delete ptr2;
	}

	//return internal node
	return node;
}

//QuadTree's Point-in-Polygon test
void GridPIP2D::quadtreePIP()
{
	memset(this->testedResult, 0, sizeof(int)*this->testedPointCount);
	this->stat.realInterCount = 0;

	QuadTreeNode *root = this->quadtree->root;
	for (int i = 0; i < this->testedPointCount; i++)
	{
		//return points out of the bounding box
		if (this->testedPoint[i].x < this->testedPolygon->boundingBox[0].x ||
			this->testedPoint[i].x > this->testedPolygon->boundingBox[1].x ||
			this->testedPoint[i].y < this->testedPolygon->boundingBox[0].y ||
			this->testedPoint[i].y > this->testedPolygon->boundingBox[1].y)
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//locate current point
		QuadTreeNode * node = this->determineNodeRecursive(root, &this->testedPoint[i]);
		if (node == NULL) //on the boundary
		{
			this->testedPoint[i] = CELL_OUT;
			continue;
		}

		Point2D cp;
		cp.x = (node->boundingbox[0].x + node->boundingbox[1].x)*0.5;
		cp.y = (node->boundingbox[0].y + node->boundingbox[1].y)*0.5;
		EdgeRef2D * cur_edge = node->edgeRef;
		int inter_count = 0;
		while (cur_edge)
		{
			Point2D * start = &this->testedPolygon->vertexTable[cur_edge->e->startIndex];
			Point2D * end = &this->testedPolygon->vertexTable[cur_edge->e->endIndex];
			inter_count += this->isIntersect(&cp, &this->testedPoint[i], start, end);
			cur_edge = cur_edge->next;

			this->stat.realInterCount++;
		}

		if (inter_count % 2 == 0)
			this->testedResult[i] |= (node->flag & 0x01);
		else
			this->testedResult[i] |= !(node->flag & 0x01);
	}
}

//determine the node that a query point fell in
QuadTreeNode * GridPIP2D::determineNodeRecursive(QuadTreeNode * node, Point2D * point)
{
	if (node->flag & QUADTREE_LEAF_NODE)
	{
		if (point->x > node->boundingbox[0].x &&
			point->x < node->boundingbox[1].x &&
			point->y > node->boundingbox[0].y &&
			point->y < node->boundingbox[1].y)
			return node;
		else
			return NULL;	//maybe on the boundary
	}

	//search internal nodes
	QuadTreeNode * ret = NULL;
	for (int i = 0; i < 4; i++)
	{
		if (node->subnode[0] != NULL)
			ret = determineNodeRecursive(node->subnode[i], point);
		if (ret != NULL)
			return ret;
	}

	return NULL;
}

void GridPIP2D::getQuadtreeInfoRecursive(QuadTreeNode * node)
{
	if (node == NULL)
		return;

	if (node == this->quadtree->root)
	{
		this->stat.internalNodeCount = 0;
		this->stat.leafNodeCount = 0;
		this->stat.emptyLeafNodeCount = 0;
		this->stat.realRefCount = 0;
	}

	if (node->flag & QUADTREE_LEAF_NODE)
	{
		int count = 0;
		EdgeRef2D * ptr = node->edgeRef;
		while (ptr)
		{
			count++;
			ptr = ptr->next;
		}
		//this->stat.storageAS += (sizeof(struct QuadTreeNode) + sizeof(struct EdgeRef2D)*count);
		if (count == 0)
			this->stat.emptyLeafNodeCount++;
		this->stat.leafNodeCount++;
		this->stat.realRefCount += count;
		return;
	}

	this->stat.internalNodeCount++;
	this->getQuadtreeInfoRecursive(node->subnode[0]);
	this->getQuadtreeInfoRecursive(node->subnode[1]);
	this->getQuadtreeInfoRecursive(node->subnode[2]);
	this->getQuadtreeInfoRecursive(node->subnode[3]);

	return;
}

void GridPIP2D::getGridInfo()
{
	if (this->grid == NULL)
		return;

	int empty_cell_count = 0;
	int suspect_cell_count = 0;
	int in_cell_count = 0;
	int out_cell_count = 0;
	int ref_count = 0;
	for (int i = 0; i < this->grid->cellCount; i++)
	{
		if (this->grid->cell[i].edgeCount == 0)
			empty_cell_count++;
		if (this->grid->cell[i].flag == CELL_IN)
			in_cell_count++;
		if (this->grid->cell[i].flag == CELL_OUT)
			out_cell_count++;
		if (this->grid->cell[i].flag == CELL_SUSPECT)
			suspect_cell_count++;
		ref_count += this->grid->cell[i].edgeCount;
	}
	this->stat.emptyCellCount = empty_cell_count;
	this->stat.suspectCellCount = suspect_cell_count;
	this->stat.insideCellCount = in_cell_count;
	this->stat.outsideCellCount = out_cell_count;
	this->stat.realRefCount = ref_count;
}

//get the storage costs (only support static polygon by now)
//NOTE: call getGridInfo or getQuadtreeInfoRecursive before calling this function
void GridPIP2D::getStorage()
{
	if (this->testedPolygon == NULL)
		return;

	stat.storageNormal = sizeof(struct GCPPolygon2D) +
		sizeof(struct Point2D) * this->testedPolygon->vertexCount +
		sizeof(struct Edge2D) * this->testedPolygon->edgeCount;

	if (this->asType == AS_TYPE_GRID)
	{
		if (this->grid != NULL)
		{	/*stat.storageAS = sizeof(struct Grid2D) +
								sizeof(struct EdgeRef2D) * this->grid->expectRefCount +
								sizeof(struct GridCell2D) * this->grid->cellCount;*/
								/*
								stat.storageAS = sizeof(struct EdgeRef2D) * grid->expectRefCount +
																this->grid->cellCount * (sizeof(EdgeRef2D*) + sizeof(int));
								*/
								//计算实际边指针数量
			this->stat.realRefCount = 0;
			this->stat.emptyCellCount = 0;
			for (int i = 0; i < this->grid->cellCount; i++)
			{
				EdgeRef2D * edge = this->grid->cell[i].edgeRef;
				while (edge)
				{
					this->stat.realRefCount++;
					edge = edge->next;
				}
				if (this->grid->cell[i].edgeCount == 0)
					stat.emptyCellCount++;
			}
			stat.storageAS = sizeof(struct EdgeRef2D) * this->stat.realRefCount +
				this->grid->cellCount * (sizeof(EdgeRef2D*) + sizeof(int));
			bool flag = true;
		}
	}
	else if (this->asType == AS_TYPE_QUADTREE)
	{
		if (this->quadtree != NULL && this->quadtree->root != NULL)
		{
			this->stat.storageAS = 0;
			this->stat.storageAS = sizeof(struct QuadTree);
			this->stat.storageAS += sizeof(struct EdgeRef2D) * this->stat.realRefCount;
			this->stat.storageAS += sizeof(struct QuadTreeNode) * (this->stat.leafNodeCount + this->stat.internalNodeCount);
		}
	}
	else if (this->asType == AS_TYPE_RECURSIVEGRID)
	{
		if (this->rgrid != NULL)
			this->stat.storageAS = this->getRGStorage(this->rgrid,
				this->stat.realRefCount,
				this->stat.emptyCellCount,
				this->stat.nonEmptyCellCount);
		bool flag = true;
	}
}

//Get the number of inside points (outside points)
void GridPIP2D::getResult()
{
	int in_count = 0;
	int out_count = 0;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		if (this->testedResult[i] == CELL_IN)
			in_count++;
		else if (this->testedResult[i] == CELL_OUT)
			out_count++;
	}
	this->stat.insidePointCount = in_count;
	this->stat.outsidePointCount = out_count;
}

//Comparing method - ray crossing
void GridPIP2D::raycrossingPIP()
{
	if (this->testedPolygon == NULL ||
		this->testedPoint == NULL ||
		this->testedResult == NULL)
		return;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p0 = &(this->testedPoint[i]);

		//determine the query points outside the bounding box
		if ((p0->x < this->testedPolygon->boundingBox[0].x) ||
			(p0->x > this->testedPolygon->boundingBox[1].x) ||
			(p0->y < this->testedPolygon->boundingBox[0].y) ||
			(p0->y > this->testedPolygon->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//determine another point on the current ray
		FLTYPE len = this->testedPolygon[curPolygonIndex].boundingBox[1].x - this->testedPolygon[curPolygonIndex].boundingBox[0].x;
		Point2D p1;
		p1.y = p0->y;
		p1.x = this->testedPolygon[curPolygonIndex].boundingBox[1].x + len * 0.1;

		//count the intersection between current segment and all the polygon edges
		int count = 0;
		for (int j = 0; j < this->testedPolygon->edgeCount; j++)
		{
			Edge2D * edge = &this->testedPolygon[curPolygonIndex].edgeTable[j];
			Point2D * p2 = &(this->testedPolygon[curPolygonIndex].vertexTable[edge->startIndex]);
			Point2D * p3 = &(this->testedPolygon[curPolygonIndex].vertexTable[edge->endIndex]);

			count += isIntersect(p0, &p1, p2, p3);	//有交为1，无交为0
		}
		if (count % 2 == 0)	//even, ouside
			this->testedResult[i] = CELL_OUT;
		else
			this->testedResult[i] = CELL_IN;
	}
}


//Other translating mode for comparison - spiral
void GridPIP2D::setCellProp_spiral()
{
	//统计预处理时求交计算的总次数
	this->stat.prepInterCount = 0;
	for (int i = 0; i < this->grid->cellCount; i++)
		this->grid->cell[i].flag = CELL_UNKNOWN;

	int step[8], last[3], cur[3];
	Point2D last_cp, cur_cp;

	step[0] = 1; step[1] = 0;
	step[2] = 0; step[3] = 1;
	step[4] = -1; step[5] = 0;
	step[6] = 0; step[7] = -1;

	last[0] = -1;
	last[1] = 0;
	last[2] = -1;
	last_cp.x = this->grid->boundingBox[0].x - this->grid->cellSize[0];
	last_cp.y = this->grid->boundingBox[0].y + (this->grid->cellSize[1] / 2.0f);

	cur[0] = 0;
	cur[1] = 0;
	cur[2] = 0;
	cur_cp.x = this->grid->boundingBox[0].x + (this->grid->cellSize[0] / 2.0f);
	cur_cp.y = last_cp.y;

	int processed_count = 0;
	int cur_dir = 0;
	EdgeRef2D * last_edge_ref = NULL;
	int last_property = CELL_OUT;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		//计算左侧单元内的边与中点连线段相交的数量
		while (cur_edge_ref)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}
		//加上当前单元内的边与中点连线段相交的数量，注意剔除重复的边
		cur_edge_ref = this->grid->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->grid->cell[cur[2]].flag = last_property;
		else		//奇数
			this->grid->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->grid->cell[cur[2]].flag;
		last_edge_ref = this->grid->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
		//determine the turning point
		if (cur[0] < 0 || cur[0] >= this->grid->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->grid->resolution[1] ||
			this->grid->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
		}
		cur_cp.x = cur_cp.x + this->grid->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->grid->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == this->grid->cellCount)
			break;
	}//while
}//setCellProp_spiral

//Other translating mode for comparison - zigzag
void GridPIP2D::setCellProp_zigzag()
{
	//统计预处理时求交计算的总次数
	this->stat.prepInterCount = 0;
	for (int i = 0; i < this->grid->cellCount; i++)
		this->grid->cell[i].flag = CELL_UNKNOWN;

	int step[8], last[3], cur[3];
	Point2D last_cp, cur_cp;

	step[0] = 1; step[1] = 0;
	step[2] = -1; step[3] = -1;
	step[4] = 0; step[5] = -1;
	step[6] = 1; step[7] = 1;

	last[0] = -1;
	last[1] = this->grid->resolution[1] - 1;
	last[2] = -1;
	last_cp.x = this->grid->boundingBox[0].x - this->grid->cellSize[0];
	last_cp.y = this->grid->boundingBox[1].y - (this->grid->cellSize[1] / 2.0f);

	cur[0] = 0;
	cur[1] = this->grid->resolution[1] - 1;
	cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
	cur_cp.x = this->grid->boundingBox[0].x + (this->grid->cellSize[0] / 2.0f);
	cur_cp.y = last_cp.y;

	int processed_count = 0;
	int cur_dir = 0;
	EdgeRef2D * last_edge_ref = NULL;
	int last_property = CELL_OUT;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		//计算左侧单元内的边与中点连线段相交的数量
		while (cur_edge_ref)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}
		//加上当前单元内的边与中点连线段相交的数量，注意剔除重复的边
		cur_edge_ref = this->grid->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->grid->cell[cur[2]].flag = last_property;
		else		//奇数
			this->grid->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->grid->cell[cur[2]].flag;
		last_edge_ref = this->grid->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		if ((processed_count != 0) && (cur_dir == 0 || cur_dir == 2))
			cur_dir = (cur_dir + 1) % 4;
		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->grid->resolution[0] + cur[0];

		//determine the turning point
		int dir_count = 0;
		while (cur[0] < 0 || cur[0] >= this->grid->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->grid->resolution[1] ||
			this->grid->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
			dir_count++;
			if (dir_count == 3)
				break;
		}
		cur_cp.x = cur_cp.x + this->grid->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->grid->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == this->grid->cellCount)
			break;
	}//while
}//setCellProp_zigzag

//Other translating mode for comparison - circuitous
void GridPIP2D::setCellProp_circuitous()
{
	//统计预处理时求交计算的总次数
	this->stat.prepInterCount = 0;
	for (int i = 0; i < this->grid->cellCount; i++)
		this->grid->cell[i].flag = CELL_UNKNOWN;

	int step[8], last[3], cur[3];
	Point2D last_cp, cur_cp;

	step[0] = 1; step[1] = 0;
	step[2] = 0; step[3] = 1;
	step[4] = -1; step[5] = 0;
	step[6] = 0; step[7] = 1;

	last[0] = -1;
	last[1] = 0;
	last[2] = -1;
	last_cp.x = this->grid->boundingBox[0].x - this->grid->cellSize[0];
	last_cp.y = this->grid->boundingBox[0].y + (this->grid->cellSize[1] / 2.0f);

	cur[0] = 0;
	cur[1] = 0;
	cur[2] = 0;
	cur_cp.x = this->grid->boundingBox[0].x + (this->grid->cellSize[0] / 2.0f);
	cur_cp.y = last_cp.y;

	int processed_count = 0;
	int cur_dir = 0;
	EdgeRef2D * last_edge_ref = NULL;
	int last_property = CELL_OUT;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		//计算左侧单元内的边与中点连线段相交的数量
		while (cur_edge_ref)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}
		//加上当前单元内的边与中点连线段相交的数量，注意剔除重复的边
		cur_edge_ref = this->grid->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
			this->stat.prepInterCount++;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->grid->cell[cur[2]].flag = last_property;
		else		//奇数
			this->grid->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->grid->cell[cur[2]].flag;
		last_edge_ref = this->grid->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		if (cur_dir == 1 || cur_dir == 3)
			cur_dir = (cur_dir + 1) % 4;
		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
		//determine the turning point
		if (cur[0] < 0 || cur[0] >= this->grid->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->grid->resolution[1] ||
			this->grid->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->grid->resolution[0] + cur[0];
		}
		cur_cp.x = cur_cp.x + this->grid->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->grid->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == this->grid->cellCount)
			break;
	}//while
}//setCellProp_circuitous

//创建递归层次网格
void GridPIP2D::createRGrid()
{
	if (this->testedPolygon == NULL)
		return;

	if (this->rgrid != NULL)
	{
		delete this->rgrid;
		this->rgrid = NULL;
	}

	this->rgrid = new RecursiveGrid2D;

	float ratio = 0.001;
	float range[2];
	range[0] = (this->testedPolygon->boundingBox[1].x - this->testedPolygon->boundingBox[0].x) * ratio;
	range[1] = (this->testedPolygon->boundingBox[1].y - this->testedPolygon->boundingBox[0].y) * ratio;
	this->rgrid->boundingBox[0].x = this->testedPolygon->boundingBox[0].x - range[0];
	this->rgrid->boundingBox[0].y = this->testedPolygon->boundingBox[0].y - range[1];
	this->rgrid->boundingBox[1].x = this->testedPolygon->boundingBox[1].x + range[0];
	this->rgrid->boundingBox[1].y = this->testedPolygon->boundingBox[1].y + range[1];

	this->rgrid->gridSize[0] = this->rgrid->boundingBox[1].x - this->rgrid->boundingBox[0].x;
	this->rgrid->gridSize[1] = this->rgrid->boundingBox[1].y - this->rgrid->boundingBox[0].y;

	this->rgrid->resolution[0] = 1;
	this->rgrid->resolution[1] = 1;

	this->rgrid->cellSize[0] = this->rgrid->gridSize[0];
	this->rgrid->cellSize[1] = this->rgrid->gridSize[1];

	this->rgrid->expectRefCount = this->testedPolygon->edgeCount;
	//size_t size = sizeof(struct EdgeRef2D) * this->rgrid->expectRefCount;
	//this->rgrid->edgeRef = (struct EdgeRef2D *)malloc(size);
	//memset(this->rgrid->edgeRef, 0, size);
	this->rgrid->edgeRef = new EdgeRef2D[this->testedPolygon->edgeCount];
	memset(this->rgrid->edgeRef, 0, sizeof(struct EdgeRef2D) * this->rgrid->expectRefCount);

	EdgeRef2D * cur = this->rgrid->edgeRef;
	for (int i = 0; i < this->rgrid->expectRefCount; i++)
	{
		cur->e = &this->testedPolygon->edgeTable[i];
		cur->next = cur + 1;
		cur++;
	}
	cur--;
	cur->next = NULL;

	this->rgrid->cell = new RecursiveGridCell2D;
	this->rgrid->cell->edgeCount = this->testedPolygon->edgeCount;
	this->rgrid->cell->edgeRef = this->rgrid->edgeRef;		//未创建，使用的是网格的边列表
	this->rgrid->edgeRef = NULL;	//传给单元后就清了
	this->rgrid->expectRefCount = 0;

	//this->rgrid->cell->edgeCount = this->testedPolygon->edgeCount;
	//this->rgrid->cell->edgeRef = new EdgeRef2D[this->testedPolygon->edgeCount];
	//memcpy(this->rgrid->cell->edgeRef, this->rgrid->edgeRef, sizeof(EdgeRef2D) * this->testedPolygon->edgeCount);

	//启动递归细分过程
	int total_cell = 0;
	int total_pt = 0;
	rgrid->subdivision(this->testedPolygon, 0, total_cell, total_pt);

	bool flag = true;
}


void RecursiveGrid2D::subdivision(GCPPolygon2D * ply, int depth, int & total_cell, int & total_pt)
{
	if (depth >= MAX_DEPTH)
		return;

	int i, j;
	for (i = 0; i < this->resolution[1]; i++)		//行
	{
		for (j = 0; j < this->resolution[0]; j++)		//列
		{
			int index = this->resolution[0] * i + j;

			if (this->cell[index].edgeCount > MAX_EDGE)
			{
				int M = this->cell[index].edgeCount;
				float W = this->cellSize[0];
				float H = this->cellSize[1];

				float dx = ply->dx;
				float dy = ply->dy;

				//Mx = (M*(dy*W)/(dx*H))0.5
				//My = Mx*(dx*H)/(dy*W)
				int res[2];
				//res[0] = sqrt( M*(dy*W)/(dx*H));
				//res[1] = res[0] * (dx*H) / (dy*W);
				/*
				int temp = MAX_DEPTH-depth;
				int K = pow(M, (1.0f/(MAX_DEPTH-depth)));
				res[0] = sqrt(K*W/H);
				res[1] = sqrt(K*H/W);
				*/

				//int m = sqrt((float)ply->edgeCount*TOTAL_M);
				int mx, my;
				if (depth == 0)
				{
					//mx = sqrt((float)48 * W / H); //sqrt((float)ply->edgeCount * W * TOTAL_M / H);
					//my = sqrt((float)48 * H  / W); //sqrt((float)ply->edgeCount * H * TOTAL_M / W);	
					//mx = sqrt(4900 * TOTAL_M);//sqrt((float)ply->edgeCount * TOTAL_M);
					//my = mx;
					mx = 9;
					my = 5;
				}
				else
				{
					//mx = sqrt((float)48 * this->gridSize[0] / this->gridSize[1]);//sqrt((float)ply->edgeCount * this->gridSize[0] * TOTAL_M / this->gridSize[1]);
					//my = sqrt((float)48 * this->gridSize[1]  / this->gridSize[0]);//sqrt((float)ply->edgeCount * this->gridSize[1] * TOTAL_M / this->gridSize[0]);	
					//mx = sqrt(4900 * TOTAL_M);//sqrt((float)ply->edgeCount * TOTAL_M);
					//my = mx;
					mx = 9;
					my = 5;
				}

				if (depth == 0)
				{
					//m1= 2**(1/3)*m**(2/3)
					res[0] = 1.25992104989 * pow(mx, (float)(2.0 / 3.0));
					res[1] = 1.25992104989 * pow(my, (float)(2.0 / 3.0));
				}
				else
				{
					//m2 = m/m1 = (0.5m)**(1/3)
					res[0] = pow(0.5*mx, 1.0 / 3.0);
					res[1] = pow(0.5*my, 1.0 / 3.0);
				}

				//保证两者均为奇数
				if (depth != 0)
				{
					if (res[0] % 2 == 0)	res[0] += 1;
					if (res[1] % 2 == 0)	res[1] += 1;
				}

				//是一个单元就不用细分了
				if (!(res[0] == 1 && res[1] == 1))
				{
					RecursiveGrid2D * rg = new RecursiveGrid2D;
					rg->resolution[0] = res[0];
					rg->resolution[1] = res[1];
					rg->boundingBox[0].x = this->boundingBox[0].x + this->cellSize[0] * j;
					rg->boundingBox[0].y = this->boundingBox[0].y + this->cellSize[1] * i;
					rg->boundingBox[1].x = rg->boundingBox[0].x + this->cellSize[0];
					rg->boundingBox[1].y = rg->boundingBox[0].y + this->cellSize[1];
					rg->gridSize[0] = this->cellSize[0];
					rg->gridSize[1] = this->cellSize[1];
					rg->cellSize[0] = rg->gridSize[0] / rg->resolution[0];
					rg->cellSize[1] = rg->gridSize[1] / rg->resolution[1];

					rg->cell = new RecursiveGridCell2D[rg->resolution[0] * rg->resolution[1]];

					//预分配边指针，估算最大边指针数
					//maxA= dx/bx+dy/by+1+2
					if (depth == 0)
						rg->expectRefCount = (dx / (W / res[0]) + dy / (H / res[1]) + 3) * M;
					else
					{
						//计算单元内边的平均边长
						EdgeRef2D * edge = this->cell[index].edgeRef;
						dx = 0; dy = 0;
						while (edge)
						{
							dx += abs(ply->vertexTable[edge->e->endIndex].x - ply->vertexTable[edge->e->startIndex].x);
							dy += abs(ply->vertexTable[edge->e->endIndex].y - ply->vertexTable[edge->e->startIndex].y);
							edge = edge->next;
						}
						dx /= this->cell[index].edgeCount;
						dy /= this->cell[index].edgeCount;
						rg->expectRefCount = (dx / (W / res[0]) + dy / (H / res[1]) + 3) * M;
					}

					rg->edgeRef = new EdgeRef2D[rg->expectRefCount];
					memset(rg->edgeRef, 0, sizeof(struct EdgeRef2D) * rg->expectRefCount);

					//插入边至子网格
					int real_edge_count = rg->addEdgeRef(ply, this->cell[index].edgeRef);
					//just for debug
					if (real_edge_count > rg->expectRefCount)
					{
						bool flag = true;
					}

					//设置子网格单元中心点属性
					if (depth == 0)
						rg->setCellProp_out(ply);	//从网格外部点开始
					else
						rg->setCellProp_center(ply, this->cell[index].flag);	//从网格中心点开始

					//废除原边指针
					this->cell[index].subgrid = rg;
					this->cell[index].edgeCount = -1;	//子网格标记

					//创建下一层网格
					rg->subdivision(ply, depth + 1, total_cell, total_pt);

					//统计本层的单元数和指针数
					total_pt += real_edge_count;
					total_cell += rg->resolution[0] * rg->resolution[1];
				}
			} // if > MAX_EDGE
		} //for j
	}	//for i

	bool flag = true;
}

int RecursiveGrid2D::addEdgeRef(GCPPolygon2D * ply, EdgeRef2D * ef)
{
	struct EdgeRef2D * cur_location = this->edgeRef;
	EdgeRef2D * cur_edge = ef;
	int k = 0;
	int real_edge_count = 0;

	while (cur_edge != NULL)
	{
		//Point2D * start_vertex = & ply->vertexTable[cur_edge->e->startIndex];
		//Point2D * end_vertex = & ply->vertexTable[cur_edge->e->endIndex];
		Point2D start_vertex, end_vertex;
		start_vertex.x = ply->vertexTable[cur_edge->e->startIndex].x;
		start_vertex.y = ply->vertexTable[cur_edge->e->startIndex].y;
		end_vertex.x = ply->vertexTable[cur_edge->e->endIndex].x;
		end_vertex.y = ply->vertexTable[cur_edge->e->endIndex].y;

		//裁剪边，仅限于网格包围盒内。入点和出点打出很小的富裕，以避免误差
		if (start_vertex.x<this->boundingBox[0].x || start_vertex.x>this->boundingBox[1].x ||
			start_vertex.y<this->boundingBox[0].y || start_vertex.y>this->boundingBox[1].y ||
			end_vertex.x<this->boundingBox[0].x || end_vertex.x>this->boundingBox[1].x ||
			end_vertex.y<this->boundingBox[0].y || end_vertex.y>this->boundingBox[1].y)
			this->clipSegment(this->boundingBox, start_vertex, end_vertex);

		//确定起始点、终止点单元,分别记录边指针
		int start_index[3], end_index[3]; //[3]为一维索引

		start_index[2] = this->locatePoint(start_vertex.x, start_vertex.y, start_index[0], start_index[1]);
		cur_location->e = cur_edge->e;
		cur_location->next = this->cell[start_index[2]].edgeRef;
		this->cell[start_index[2]].edgeRef = cur_location;
		this->cell[start_index[2]].edgeCount++;

		cur_location++;
		real_edge_count++;

		end_index[2] = this->locatePoint(end_vertex.x, end_vertex.y, end_index[0], end_index[1]);
		if (end_index[2] == start_index[2])  //整条边在一个单元内
		{
			cur_edge = cur_edge->next;
			k++;
			continue;
		}
		cur_location->e = cur_edge->e;
		cur_location->next = this->cell[end_index[2]].edgeRef;
		this->cell[end_index[2]].edgeRef = cur_location;
		this->cell[end_index[2]].edgeCount++;

		cur_location++;
		real_edge_count++;

		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		//x=x0+x1*t_x
		//y=y0+y1*t_y
		FLTYPE start_coord[2];
		int signx = 1;
		int signy = 1;
		if (end_vertex.x > start_vertex.x) //向右
			start_coord[0] = this->boundingBox[0].x + this->cellSize[0] * (start_index[0] + 1);
		else //向左
		{
			start_coord[0] = this->boundingBox[0].x + this->cellSize[0] * start_index[0];
			signx = -1;
		}

		if (end_vertex.y > start_vertex.y) //向上
			start_coord[1] = this->boundingBox[0].y + this->cellSize[1] * (start_index[1] + 1);
		else //向下
		{
			start_coord[1] = this->boundingBox[0].y + this->cellSize[1] * start_index[1];
			signy = -1;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(end_vertex.x - start_vertex.x);
		y_length = abs(end_vertex.y - start_vertex.y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - start_vertex.x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - start_vertex.y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}
			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->resolution[0] + cur_index[0];
			if (cur_index[2] == end_index[2])		//截止退出
				break;
			cur_location->e = cur_edge->e;
			cur_location->next = this->cell[cur_index[2]].edgeRef;
			this->cell[cur_index[2]].edgeRef = cur_location;
			this->cell[cur_index[2]].edgeCount++;

			cur_location++;
			real_edge_count++;

		} //while
		cur_edge = cur_edge->next;
		k++;
	}

	return real_edge_count;
}

//用box裁剪线段
void RecursiveGrid2D::clipSegment(Point2D * box, Point2D & start, Point2D & end)
{
	Point2D mini, maxi;
	float t;
	float ep0 = (box[1].x - box[0].x) * (1.0e-4);
	float ep1 = (box[1].y - box[0].y) * (1.0e-4);

	mini.x = box[0].x; mini.y = box[0].y;
	maxi.x = box[1].x; maxi.y = box[1].y;

	float epsilon1 = 1.0e-8;

	if ((start.x - end.x) < epsilon1 && (start.x - end.x) > -epsilon1)
	{
		if (start.y < mini.y)	start.y = mini.y + ep1;
		if (start.y > maxi.y)	start.y = maxi.y - ep1;
		if (end.y < mini.y)	 end.y = mini.y + ep1;
		if (end.y > maxi.y)	end.y = maxi.y - ep1;
	}

	if ((start.y - end.y) < epsilon1 && (start.y - end.y) > -epsilon1)
	{
		if (start.x < mini.x)	start.x = mini.x + ep0;
		if (start.x > maxi.x)	start.x = maxi.x - ep0;
		if (end.x < mini.x)	 end.x = mini.x + ep0;
		if (end.x > maxi.x)	end.x = maxi.x - ep0;
	}

	if ((start.x < end.x) && (start.y < end.y))
	{
		if (start.x < mini.x)
		{
			t = (mini.x - start.x) / (end.x - start.x);
			start.x = mini.x + ep0;
			start.y = start.y + (end.y - start.y) * t;
		}

		if (end.x > maxi.x)
		{
			t = (maxi.x - start.x) / (end.x - start.x);
			end.x = maxi.x - ep0;
			end.y = start.y + (end.y - start.y) * t;
		}

		if (start.y < mini.y)
		{
			t = (mini.y - start.y) / (end.y - start.y);
			start.y = mini.y + ep1;
			start.x = start.x + (end.x - start.x) * t;
		}

		if (end.y > maxi.y)
		{
			t = (maxi.y - start.y) / (end.y - start.y);
			end.y = maxi.y - ep1;
			end.x = start.x + (end.x - start.x) * t;
		}
	}
	else if ((start.x < end.x) && (start.y > end.y))
	{
		if (start.x < mini.x)
		{
			t = (mini.x - start.x) / (end.x - start.x);
			start.x = mini.x + ep0;
			start.y = start.y + (end.y - start.y) * t;
		}

		if (end.x > maxi.x)
		{
			t = (maxi.x - start.x) / (end.x - start.x);
			end.x = maxi.x - ep0;
			end.y = start.y + (end.y - start.y) * t;
		}

		if (end.y < mini.y)
		{
			t = (mini.y - start.y) / (end.y - start.y);
			end.y = mini.y + ep1;
			end.x = start.x + (end.x - start.x) * t;
		}

		if (start.y > maxi.y)
		{
			t = (maxi.y - start.y) / (end.y - start.y);
			start.y = maxi.y - ep1;
			start.x = start.x + (end.x - start.x) * t;
		}
	}
	else if ((start.x > end.x) && (start.y < end.y))
	{
		if (end.x < mini.x)
		{
			t = (mini.x - start.x) / (end.x - start.x);
			end.x = mini.x + ep0;
			end.y = start.y + (end.y - start.y) * t;
		}

		if (start.x > maxi.x)
		{
			t = (maxi.x - start.x) / (end.x - start.x);
			start.x = maxi.x - ep0;
			start.y = start.y + (end.y - start.y) * t;
		}

		if (start.y < mini.y)
		{
			t = (mini.y - start.y) / (end.y - start.y);
			start.y = mini.y + ep1;
			start.x = start.x + (end.x - start.x) * t;
		}

		if (end.y > maxi.y)
		{
			t = (maxi.y - start.y) / (end.y - start.y);
			end.y = maxi.y - ep1;
			end.x = start.x + (end.x - start.x) * t;
		}
	}
	else if ((start.x > end.x) && (start.y > end.y))
	{
		if (end.x < mini.x)
		{
			t = (mini.x - start.x) / (end.x - start.x);
			end.x = mini.x + ep0;
			end.y = start.y + (end.y - start.y) * t;
		}

		if (start.x > maxi.x)
		{
			t = (maxi.x - start.x) / (end.x - start.x);
			start.x = maxi.x - ep0;
			start.y = start.y + (end.y - start.y) * t;
		}

		if (end.y < mini.y)
		{
			t = (mini.y - start.y) / (end.y - start.y);
			end.y = mini.y + ep1;
			end.x = start.x + (end.x - start.x) * t;
		}

		if (start.y > maxi.y)
		{
			t = (maxi.y - start.y) / (end.y - start.y);
			start.y = maxi.y - ep1;
			start.x = start.x + (end.x - start.x) * t;
		}
	}
}

//清除空间
void RecursiveGrid2D::clear(int depth)
{
	int i;

	//单元
	if (this->cell != NULL)
	{
		//子网格
		for (i = 0; i < this->resolution[0] * resolution[1]; i++)
		{
			if (this->cell[i].edgeCount == -1)
			{
				//delete this->cell[i].subgrid;
				this->cell[i].subgrid->clear(depth + 1);
				delete this->cell[i].subgrid;
				this->cell[i].subgrid = NULL;
			}
		}
		if (i == 1)
			delete cell;
		else
			delete[] cell;
		cell = NULL;
	}

	//边指针
	if (this->edgeRef != NULL)
	{
		delete[] edgeRef;
		edgeRef = NULL;
	}
}

//以网格中心点作为起始点
void RecursiveGrid2D::setCellProp_center(GCPPolygon2D * ply, int center_prop)
{
	for (int i = 0; i < this->resolution[0] * this->resolution[1]; i++)
		this->cell[i].flag = CELL_UNKNOWN;

	int step[8], last[3], cur[3], cur_dir;
	Point2D last_cp, cur_cp;

	//先算一半
	if (this->resolution[1] != 1)
	{
		step[0] = 0; step[1] = 1;
		step[2] = 1; step[3] = 0;
		step[4] = 0; step[5] = -1;
		step[6] = 1; step[7] = 0;
	}
	else
	{
		step[0] = 1; step[1] = 0;
		step[2] = 0; step[3] = 1;
		step[4] = -1; step[5] = 0;
		step[6] = 1; step[7] = 0;
	}

	cur_dir = 0;

	last[0] = this->resolution[0] >> 1;
	last[1] = this->resolution[1] >> 1;
	last[2] = this->resolution[0] * last[1] + last[0];
	last_cp.x = this->boundingBox[0].x + this->cellSize[0] * (last[0] + 0.5);
	last_cp.y = this->boundingBox[0].y + this->cellSize[1] * (last[1] + 0.5);
	int last_property = center_prop;
	this->cell[last[2]].flag = center_prop;

	cur[0] = last[0] + step[cur_dir << 1];
	cur[1] = last[1] + step[(cur_dir << 1) + 1];
	cur[2] = cur[1] * this->resolution[0] + cur[0];
	cur_cp.x = last_cp.x + this->cellSize[0] * step[cur_dir << 1];
	cur_cp.y = last_cp.y + this->cellSize[1] * step[(cur_dir << 1) + 1];

	int processed_count = 0;
	EdgeRef2D * last_edge_ref = this->cell[last[2]].edgeRef;
	int k = 0;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		while (cur_edge_ref)
		{
			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
		}

		cur_edge_ref = this->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->cell[cur[2]].flag = last_property;
		else		//奇数
			this->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->cell[cur[2]].flag;
		last_edge_ref = this->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		if (cur_dir == 1 || cur_dir == 3)
			cur_dir = (cur_dir + 1) % 4;
		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->resolution[0] + cur[0];

		//determine the turning point
		if (cur[0] < 0 || cur[0] >= this->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->resolution[1] ||
			this->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->resolution[0] + cur[0];
		}

		cur_cp.x = cur_cp.x + this->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == ((this->resolution[0] * this->resolution[1]) >> 1))
			break;
	}//while

	//再处理另一半
	if (this->resolution[1] != 1)
	{
		step[0] = 0; step[1] = -1;
		step[2] = -1; step[3] = 0;
		step[4] = 0; step[5] = 1;
		step[6] = -1; step[7] = 0;
	}
	else
	{
		step[0] = -1; step[1] = 0;
		step[2] = 0; step[3] = -1;
		step[4] = 1; step[5] = 0;
		step[6] = 0; step[7] = -1;
	}

	cur_dir = 0;

	last[0] = this->resolution[0] >> 1;
	last[1] = this->resolution[1] >> 1;
	last[2] = this->resolution[0] * last[1] + last[0];
	last_cp.x = this->boundingBox[0].x + this->cellSize[0] * (last[0] + 0.5);
	last_cp.y = this->boundingBox[0].y + this->cellSize[1] * (last[1] + 0.5);
	last_property = center_prop;

	cur[0] = last[0] + step[cur_dir << 1];
	cur[1] = last[1] + step[(cur_dir << 1) + 1];
	cur[2] = cur[1] * this->resolution[0] + cur[0];
	cur_cp.x = last_cp.x + this->cellSize[0] * step[cur_dir << 1];
	cur_cp.y = last_cp.y + this->cellSize[1] * step[(cur_dir << 1) + 1];

	processed_count = 0;
	last_edge_ref = this->cell[last[2]].edgeRef;
	k = 0;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		while (cur_edge_ref)
		{
			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
		}

		cur_edge_ref = this->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->cell[cur[2]].flag = last_property;
		else		//奇数
			this->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->cell[cur[2]].flag;
		last_edge_ref = this->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		if (cur_dir == 1 || cur_dir == 3)
			cur_dir = (cur_dir + 1) % 4;
		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->resolution[0] + cur[0];

		//determine the turning point
		if (cur[0] < 0 || cur[0] >= this->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->resolution[1] ||
			this->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->resolution[0] + cur[0];
		}

		cur_cp.x = cur_cp.x + this->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == ((this->resolution[0] * this->resolution[1]) >> 1))
			break;
	}//while
}

void RecursiveGrid2D::setCellProp_out(GCPPolygon2D * ply)
{
	for (int i = 0; i < this->resolution[0] * this->resolution[1]; i++)
		this->cell[i].flag = CELL_UNKNOWN;

	int step[8], last[3], cur[3], cur_dir;
	Point2D last_cp, cur_cp;

	step[0] = 1; step[1] = 0;
	step[2] = 0; step[3] = 1;
	step[4] = -1; step[5] = 0;
	step[6] = 0; step[7] = 1;

	cur_dir = 0;

	last[0] = -1;
	last[1] = 0;
	last[2] = -1;
	last_cp.x = this->boundingBox[0].x - this->cellSize[0];
	last_cp.y = this->boundingBox[0].y + this->cellSize[1] * 0.5;
	int last_property = CELL_OUT;

	cur[0] = 0;
	cur[1] = 0;
	cur[2] = 0;
	cur_cp.x = this->boundingBox[0].x + this->cellSize[0] * 0.5;
	cur_cp.y = this->boundingBox[0].y + this->cellSize[1] * 0.5;

	int processed_count = 0;
	EdgeRef2D * last_edge_ref = NULL;
	int k = 0;
	while (1)
	{
		//connect last and current cell's center point
		int intersect_count = 0;
		EdgeRef2D * cur_edge_ref = last_edge_ref;

		while (cur_edge_ref)
		{
			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);	//有交为1，无交为0
			cur_edge_ref = cur_edge_ref->next;
		}

		cur_edge_ref = this->cell[cur[2]].edgeRef;
		while (cur_edge_ref)
		{
			//判定是否重复
			EdgeRef2D * temp = last_edge_ref;
			bool duplicate = false;
			while (temp)
			{
				if (temp->e == cur_edge_ref->e)
				{
					duplicate = true;
					break;
				}
				else
					temp = temp->next;
			}
			if (duplicate)
			{
				cur_edge_ref = cur_edge_ref->next;
				continue;
			}

			Point2D * p0 = &(ply->vertexTable[cur_edge_ref->e->startIndex]);
			Point2D * p1 = &(ply->vertexTable[cur_edge_ref->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &last_cp, &cur_cp);
			cur_edge_ref = cur_edge_ref->next;
		}

		if ((intersect_count % 2) == 0)	//偶数
			this->cell[cur[2]].flag = last_property;
		else		//奇数
			this->cell[cur[2]].flag = !last_property;

		//next cell
		last[0] = cur[0];
		last[1] = cur[1];
		last[2] = cur[2];
		last_property = this->cell[cur[2]].flag;
		last_edge_ref = this->cell[cur[2]].edgeRef;
		last_cp.x = cur_cp.x;
		last_cp.y = cur_cp.y;

		if (cur_dir == 1 || cur_dir == 3)
			cur_dir = (cur_dir + 1) % 4;
		cur[0] = cur[0] + step[cur_dir << 1];
		cur[1] = cur[1] + step[(cur_dir << 1) + 1];
		cur[2] = cur[1] * this->resolution[0] + cur[0];

		//determine the turning point
		if (cur[0] < 0 || cur[0] >= this->resolution[0] ||
			cur[1] < 0 || cur[1] >= this->resolution[1] ||
			this->cell[cur[2]].flag != CELL_UNKNOWN)
		{
			cur_dir = (cur_dir + 1) % 4;
			cur[0] = last[0] + step[cur_dir << 1];
			cur[1] = last[1] + step[(cur_dir << 1) + 1];
			cur[2] = cur[1] * this->resolution[0] + cur[0];
		}

		cur_cp.x = cur_cp.x + this->cellSize[0] * step[cur_dir << 1];
		cur_cp.y = cur_cp.y + this->cellSize[1] * step[(cur_dir << 1) + 1];

		//terminating criterion
		processed_count++;
		if (processed_count == (this->resolution[0] * this->resolution[1]))
			break;
	}//while
}

//判断p0p1和p2p3是否相交
bool RecursiveGrid2D::isIntersect(Point2D * p0, Point2D * p1, Point2D * p2, Point2D *p3)
{
	//p2是否在p0p1的左侧
	Point2D p2p0, p1p0;
	FLTYPE temp;
	int p2_side;
	p2p0.x = p2->x - p0->x;
	p2p0.y = p2->y - p0->y;
	p1p0.x = p1->x - p0->x;
	p1p0.y = p1->y - p0->y;
	temp = p2p0.x*p1p0.y - p2p0.y*p1p0.x;
	if (temp < 0)
		p2_side = 1;	//左侧
	else
		p2_side = 0;	//右侧

	//p3是否在p0p1的左侧
	Point2D p3p0;
	int p3_side;
	p3p0.x = p3->x - p0->x;
	p3p0.y = p3->y - p0->y;
	temp = p3p0.x*p1p0.y - p3p0.y*p1p0.x;
	if (temp < 0)
		p3_side = 1;	//左侧
	else
		p3_side = 0;	//右侧

	//p0是否位于p2p3的两侧
	Point2D p3p2, p0p2;
	int p0_side;
	p3p2.x = p3->x - p2->x;
	p3p2.y = p3->y - p2->y;
	p0p2.x = p0->x - p2->x;
	p0p2.y = p0->y - p2->y;
	temp = p0p2.x*p3p2.y - p0p2.y*p3p2.x;
	if (temp < 0)
		p0_side = 1;	//左侧
	else
		p0_side = 0;	//右侧

	//p1是否位于p3p2的两侧
	Point2D p1p2;
	int p1_side;
	p1p2.x = p1->x - p2->x;
	p1p2.y = p1->y - p2->y;
	temp = p1p2.x*p3p2.y - p1p2.y*p3p2.x;
	if (temp < 0)
		p1_side = 1;	//左侧
	else
		p1_side = 0;	//右侧

	//p2,p3同在左侧，无交；否则有交
	if ((p2_side != p3_side) && (p0_side != p1_side))
		return true;
	else
		return false;
}

//判定
void GridPIP2D::RGridPIP()
{
	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);

		//剔除在包围盒外部的点
		if ((p->x < this->rgrid->boundingBox[0].x) ||
			(p->x > this->rgrid->boundingBox[1].x) ||
			(p->y < this->rgrid->boundingBox[0].y) ||
			(p->y > this->rgrid->boundingBox[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}

		//确定被测点所在单元
		int cell_index[3];
		RecursiveGrid2D * rg = this->rgrid;
		while (1)
		{
			cell_index[2] = rg->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);
			if (rg->cell[cell_index[2]].edgeCount != -1)
				break;
			else
				rg = rg->cell[cell_index[2]].subgrid;
		}

		FLTYPE half_cell_size[2];
		half_cell_size[0] = rg->cellSize[0] / 2;
		half_cell_size[1] = rg->cellSize[1] / 2;

		//计算该单元的中心点位置
		Point2D middle;
		middle.x = rg->boundingBox[0].x + rg->cellSize[0] * cell_index[0] + half_cell_size[0];
		middle.y = rg->boundingBox[0].y + rg->cellSize[1] * cell_index[1] + half_cell_size[1];

		//计算middle和当前被测点的连线与当前单元中所有边的相交数
		int intersect_count = 0;
		EdgeRef2D * edge_ref = rg->cell[cell_index[2]].edgeRef;
		EdgeRef2D * cur_edge = edge_ref;
		while (cur_edge)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->endIndex]);
			Point2D p2, p3;
			p2.x = middle.x;
			p2.y = middle.y;
			p3.x = p->x;
			p3.y = p->y;
			intersect_count += isIntersect(p0, p1, &p2, &p3);	//有交为1，无交为0
			cur_edge = cur_edge->next;
		}

		//根据相交个数的奇偶性判定点的位置
		if (intersect_count % 2 == 0)
			this->testedResult[i] = rg->cell[cell_index[2]].flag;
		else
			this->testedResult[i] = !rg->cell[cell_index[2]].flag;
	}
}

int GridPIP2D::getRGStorage(RecursiveGrid2D * rg,
	int & reference_count,
	int & empty_cell_count,
	int & non_empty_cell_count)
{
	if (rg == NULL) return 0;

	int storage = 0;

	storage += rg->resolution[0] * rg->resolution[1] * (sizeof(EdgeRef2D*) + sizeof(int));

	//实际指针数
	int c1 = 0;		//指针数
	int c2 = 0;		//叶节点数
	int c3 = 0;		//非空单元数
	for (int i = 0; i < rg->resolution[0] * rg->resolution[1]; i++)
	{
		int subc1 = 0;
		int subc2 = 0;
		int subc3 = 0;
		if (rg->cell[i].edgeCount == 0)
		{
			c2++;
		}
		else if (rg->cell[i].edgeCount == -1)		//中间节点
		{
			storage += this->getRGStorage(rg->cell[i].subgrid, subc1, subc2, subc3);
			c1 += subc1;
			c2 += subc2;
			c3 += subc3;
			c3++;
		}
		else	//叶节点
		{
			storage += sizeof(EdgeRef2D) * rg->cell[i].edgeCount;
			c1 += rg->cell[i].edgeCount;
			c3++;
		}
	}

	reference_count = c1;
	empty_cell_count = c2;
	non_empty_cell_count = c3;

	return storage;
}

Segment2D & DynamicPointArray::operator [](int i)
{
	if (i >= maxSize)
	{
		int old_maxSize = maxSize;
		while (i >= maxSize)
			maxSize <<= 1;
		Segment2D * new_segments = new Segment2D[maxSize];
		memcpy(new_segments, segments, sizeof(Segment2D) * old_maxSize);
		delete[] segments;
		segments = new_segments;
	}
	return segments[i];
}


//多边形线裁剪，返回线段顶点列表和列表中顶点的数量（2个一组）
void  GridPIP2D::segmentClip(Point2D  & v0, Point2D & v1)
{
	//计算线段包围盒是否与网格相交
	Point2D bb[2];
	if (v0.x < v1.x)
	{
		bb[0].x = v0.x;
		bb[1].x = v1.x;
	}
	else
	{
		bb[0].x = v1.x;
		bb[1].x = v0.x;
	}
	if (v0.y < v1.y)
	{
		bb[0].y = v0.y;
		bb[1].y = v1.y;
	}
	else
	{
		bb[0].y = v1.y;
		bb[1].y = v0.y;
	}

	if (bb[1].x < this->grid_boundingbox[0].x ||
		bb[0].x > this->grid_boundingbox[1].x ||
		bb[1].y < this->grid_boundingbox[0].y ||
		bb[0].y > this->grid_boundingbox[1].y)
		return;

	//判定是否有交, 2DDDA算法
	//确定v0所在单元,与单元内的边求交
	FLTYPE * rlist = new FLTYPE[this->testedPolygon->edgeCount];	//记录交点参数
	bool * mailbox = new bool[this->testedPolygon->edgeCount];
	memset(mailbox, 0, sizeof(bool)*this->testedPolygon->edgeCount);
	int rcount = 0;

	//求交点，被测线段与当前单元内的所有线段求交，记录交点t值并设置mailbox
	//求与起始点单元内边的交点
	int start_index[3];
	start_index[2] = this->grid->locatePoint(v0.x, v0.y, start_index[0], start_index[1]);
	getIntersectionInCell(&v0, &v1, start_index[2], rlist, rcount, mailbox);

	int end_index[3];
	end_index[2] = this->grid->locatePoint(v1.x, v1.y, end_index[0], end_index[1]);

	//整条边不在一个单元内，检测下一个单元，DDA算法
	if (end_index[2] != start_index[2])
	{
		//计算从起始点开始首次相交的边，是x方向还是y方向上的,并设递进符号
		//x=x0+x1*t_x
		//y=y0+y1*t_y
		FLTYPE start_coord[2];
		int signx = 1;
		int signy = 1;
		if (v1.x > v0.x) //向右
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * (start_index[0] + 1);
		else //向左
		{
			start_coord[0] = this->grid->boundingBox[0].x + this->grid->cellSize[0] * start_index[0];
			signx = -1;
		}

		if (v1.y > v0.y) //向上
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * (start_index[1] + 1);
		else //向下
		{
			start_coord[1] = this->grid->boundingBox[0].y + this->grid->cellSize[1] * start_index[1];
			signy = -1;
		}

		//计算跨越一个单元时，x,y方向上的t增量
		FLTYPE x_length, y_length;
		x_length = abs(v1.x - v0.x);
		y_length = abs(v1.y - v0.y);
		if (x_length == 0) x_length = ZERO_OFFSET;
		if (y_length == 0) y_length = ZERO_OFFSET;
		FLTYPE dtx = this->grid->cellSize[0] / x_length;		//dtx.dty均为正
		FLTYPE dty = this->grid->cellSize[1] / y_length;

		//确定边从单元的哪个方向的边穿出，计算穿出点的t值
		FLTYPE t_h, t_v;
		t_h = abs(start_coord[0] - v0.x) / x_length;		//t_h,t_v均为正
		t_v = abs(start_coord[1] - v0.y) / y_length;

		//开始递增行进
		int cur_index[3];
		cur_index[0] = start_index[0];
		cur_index[1] = start_index[1];
		cur_index[2] = start_index[2];
		while (1)
		{
			//计算下一个单元索引
			if (t_h < t_v) {
				cur_index[0] += signx;
				t_h += dtx;
			}
			else {
				cur_index[1] += signy;
				t_v += dty;
			}

			//记录当前单元的边指针
			cur_index[2] = cur_index[1] * this->grid->resolution[0] + cur_index[0];
			getIntersectionInCell(&v0, &v1, cur_index[2], rlist, rcount, mailbox);

			if (cur_index[2] == end_index[2])		//截止退出
				break;
		} //while
	}

	//对交点排序
	insertSort(rlist, rcount);

	//返回裁剪结果片段
	Point2D *points = new Point2D[rcount + 2];
	points[0] = v0;
	int i;
	for (i = 0; i < rcount; i++)
	{
		Point2D intersection;
		intersection.x = v0.x + (v1.x - v0.x) * rlist[i];
		intersection.y = v0.y + (v1.y - v0.y) * rlist[i];
		points[i + 1] = intersection;
	}
	points[i + 1] = v1;

	int seg_count = (rcount + 2) / 2;
	//Segment2D * segs = new Segment2D[seg_count];
	int cur;
	if (this->isInside(&v0) == CELL_OUT)
	{
		cur = 1;
		if (rcount % 2 == 0)
			seg_count--;
	}
	else
		cur = 0;
	for (i = 0; i < seg_count; i++)
	{
		(*this->segmentClipResult)[this->segmentCount + i].v0.x = points[cur].x;
		(*this->segmentClipResult)[this->segmentCount + i].v0.y = points[cur].y;
		cur++;
		(*this->segmentClipResult)[this->segmentCount + i].v1.x = points[cur].x;
		(*this->segmentClipResult)[this->segmentCount + i].v1.y = points[cur].y;
		cur++;

		//just for debug
		if (this->segmentCount + i >= 1024)
		{
			bool flag = true;
		}
	}
	this->segmentCount += seg_count;

	delete[] points;
}

//简单插入排序
void GridPIP2D::insertSort(FLTYPE  * rlist, int  rcount)
{
	int i, j, k;
	FLTYPE t;

	for (i = 1; i < rcount; i++)
	{
		for (j = 0; j < i; j++)
		{
			if (rlist[j] > rlist[i])
			{
				t = rlist[i];
				for (k = i; k > j; k--)
					rlist[k] = rlist[k - 1];
				rlist[j] = t;
				break;
			}
		}
	}
}


//求某条线段与某个单元内边的交点
void GridPIP2D::getIntersectionInCell(Point2D * v0, Point2D * v1, int cell_index, FLTYPE * rlist, int & rcount, bool * mailbox)
{
	int i;
	Point2D p0, p1;
	EdgeRef2D * cur_edge = this->grid->cell[cell_index].edgeRef;

	for (i = 0; i < this->grid->cell[cell_index].edgeCount; i++)
	{
		p0 = this->testedPolygon->vertexTable[cur_edge->e->startIndex];
		p1 = this->testedPolygon->vertexTable[cur_edge->e->endIndex];

		int cur_edge_index = cur_edge->e - this->testedPolygon->edgeTable;
		if (mailbox[cur_edge_index] == 0)
		{
			mailbox[cur_edge_index] = true;
			FLTYPE r, s;
			int ret = this->getIntersection(v0, v1, &p0, &p1, r, s);
			if (ret == 1) //有交
			{
				rlist[rcount] = r;
				rcount++;
			}
		}
		cur_edge = cur_edge->next;
	}
}


//判定是否相交，返回交点的参数值
//r为v0v1的参数，s为v2v3的参数
//奇异情况未做处理
int GridPIP2D::getIntersection(Point2D *A, Point2D *B, Point2D *C, Point2D *D, FLTYPE & r, FLTYPE & s)
{
	r = (double)((A->y - C->y) * (D->x - C->x) - (A->x - C->x) * (D->y - C->y)) / (double)((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
	s = (double)((A->y - C->y) * (B->x - A->x) - (A->x - C->x) * (B->y - A->y)) / (double)((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
	if ((r >= 0 && r <= 1) && (s >= 0 && s <= 1))
		return 1;
	else
		return 0;
}

//判定一个点是否位于多边形内（robust）
int GridPIP2D::isInside(Point2D * p)
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	//剔除在包围盒外部的点
	if ((p->x < this->grid->boundingBox[0].x) ||
		(p->x > this->grid->boundingBox[1].x) ||
		(p->y < this->grid->boundingBox[0].y) ||
		(p->y > this->grid->boundingBox[1].y))
		return CELL_OUT;

	//确定被测点所在单元
	int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
	cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);

	//计算可用参考点的坐标
	Point2D middle;
	middle.y = this->grid->boundingBox[0].y + this->grid->cellSize[1] * cell_index[1] + half_cell_size[1];
	int x_end = cell_index[0];
	int target = cell_index[2];
	while (this->grid->cell[target].flag == CELL_SUSPECT && x_end < this->grid_res[0])
	{
		target++;
		x_end++;
	}
	middle.x = this->grid->boundingBox[0].x + this->grid->cellSize[0] * x_end + half_cell_size[0];
	if (x_end == this->grid_res[0])
	{
		middle.x += this->grid->cellSize[0];
		target--;
	}

	//计算middle和当前被测点的连线与一系列单元中所有边的相交数
	GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
	int intersect_count = 0;

	Point2D p2, p3;
	p2.x = middle.x;
	p2.y = middle.y;
	p3.x = p->x;
	p3.y = p->y;

	for (int k = cell_index[2]; k <= target; k++, cur_cell++)
	{
		EdgeRef2D * edge_ref = cur_cell->edgeRef;
		EdgeRef2D * cur_edge = edge_ref;
		while (cur_edge)
		{
			Point2D * p0 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->startIndex]);
			Point2D * p1 = &(this->testedPolygon[curPolygonIndex].vertexTable[cur_edge->e->endIndex]);
			intersect_count += isIntersect(p0, p1, &p2, &p3);	//有交为1，无交为0
			cur_edge = cur_edge->next;
		}
	}

	//根据相交个数的奇偶性判定点的位置
	if (intersect_count % 2 == 0)
		return this->grid->cell[target].flag;
	else
		return !this->grid->cell[target].flag;
}

void GridPIP2D::export2OBJ(const char * filename, int type)
{
	if (filename == NULL)
		return;

	FILE * fp;
	fopen_s(&fp, filename, "w");
	for (int i = 0; i < this->testedPolygon->vertexCount; i++)
		fprintf(fp, "v %f %f %f\n", this->testedPolygon->vertexTable[i].x, this->testedPolygon->vertexTable[i].y, 0.0);
	for (int i = 0; i < this->testedPolygon->edgeCount; i++)
		fprintf(fp, "f %d %d %d\n", this->testedPolygon->edgeTable[i].startIndex, this->testedPolygon->edgeTable[i].endIndex, this->testedPolygon->edgeTable[i].startIndex);

	for (int i = 0; i < this->testedPointCount; i++)
	{
		if (this->testedResult[i] == type)
			fprintf(fp, "v %f %f %f\n", this->testedPoint[i].x, this->testedPoint[i].y, 0.0);
	}
}


//OGP method 代码中含有统计数据
void GridPIP2D::OgpCheckPoint_Segment()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;
	//统计乘法次数
	this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		this->Rgpstat.compareCount++;//for 循环的比较
		this->Rgpstat.addCount++;//for循环的加法
		Point2D * p = &(this->testedPoint[i]);
		//剔除在包围盒外部的点
		if ((this->Rgpstat.compareCount++&&p->x < this->grid->boundingBox[0].x) ||
			(this->Rgpstat.compareCount++&&p->x > this->grid->boundingBox[1].x) ||
			(this->Rgpstat.compareCount++&&p->y < this->grid->boundingBox[0].y) ||
			(this->Rgpstat.compareCount++&&p->y > this->grid->boundingBox[1].y))
			//if ((p->x < this->testedPointRegion[0].x) ||
			//(p->x > this->testedPointRegion[1].x) ||
			//(p->y < this->testedPointRegion[0].y) ||
			//(p->y > this->testedPointRegion[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);
		//统计上述函数中加法和乘法次数
		this->Rgpstat.addCount = this->Rgpstat.addCount + 5;
		this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 3;
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		//统计比较次数
		this->Rgpstat.compareCount++;
		if (cell_flag == CELL_OUT)
		{
			//this->Rgpstat.outsideCellPointCount++;
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		//统计比较次数
		this->Rgpstat.compareCount++;
		if (cell_flag == CELL_IN)
		{
			//this->Rgpstat.insideCellPointCount++;
			this->testedResult[i] = CELL_IN;
			continue;
		}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		//this->Rgpstat.inEdgeCellPointCount++;
		int point_index[3];                                                                                      //用于临时存储网格交点的坐标
		int x_segment_index, y_segment_index;                                                   //用于表示网格线片段的坐标位置
		int up_or_down;                                                                                         //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		double tempy;                                                                                            //考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;         //网格中点坐标y值
		this->Rgpstat.compareCount++;
		if (p->y <= tempy1)                                                                                  //向上做垂线
		{
			tempy = tempy1 - half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1];
			point_index[2] = cell_index[2] + cell_index[1];
			up_or_down = 0;
			//统计加法次数
			this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
		}//if
		else                                                                                                           //向下做垂线
		{
			tempy = tempy1 + half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1] + 1;
			point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
			up_or_down = 1;
			//统计加法次数
			this->Rgpstat.addCount = this->Rgpstat.addCount + 5;
		}//else
		//获取临近网格交点的坐标
		FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;
		int  markflag = this->RgpPoint[point_index[2]].flag;                                                   //获取网格点的属性
		int countNum = 0;                                                                                                        //记录交点数总数
		
		//平行X轴
		this->Rgpstat.compareCount++;
		if (Equal_stat(y1, p->y, 10e-10) == 1)
		{
			//this->Rgpstat.onthe_x_GridEdge++;
			if ((this->Rgpstat.compareCount++&&Equal_stat(p->y, this->grid->boundingBox[0].y, 10e-6) == 1)|| 
				(this->Rgpstat.compareCount++&&Equal_stat(p->y, this->grid->boundingBox[1].y, 10e-6) == 1))
			{
				this->testedResult[i] = markflag;
				continue;
			}//if
			else
			{
				this->Rgpstat.compareCount++;
				if (up_or_down == 0)
				{
					x_segment_index = cell_index[2] - this->grid->resolution[0];
					this->Rgpstat.addCount++;
				}
				else
				{
					x_segment_index = cell_index[2];
				}
				countNum = this->GetCountNumInSegment(p->x, 0, x_segment_index, 0);                            //获取指定网格线片段中，指定两坐标值之间的数量。  0表示横轴，1表示纵轴
				//根据交点总数判断测试点的位置属性
				int mark = countNum % 2;
				this->Rgpstat.multiplicationCount++;  //统计乘法次数
				this->Rgpstat.compareCount++;
				switch (mark)
				{
				case 1:
					this->Rgpstat.compareCount++; //统计比较次数
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				//continue;
			}//else
		}
	

		//平行Y轴
		this->Rgpstat.compareCount++;
		if (Equal_stat(x1, p->x, 10e-10) == 1)   
		{
			//this->Rgpstat.onthe_y_GridEdge++;
			//if (Equal(p->x,this->grid->boundingBox[0].x, 10e-6)==1
			//	||Equal(p->x,this->grid->boundingBox[1].x, 10e-6)==1)
			if ((this->Rgpstat.compareCount++&&Equal_stat(p->x, this->grid->boundingBox[0].x, 10e-6) == 1)
				|| (this->Rgpstat.compareCount++&&Equal_stat(p->x, this->grid->boundingBox[1].x, 10e-6) == 1))
			{
				this->testedResult[i] = markflag;
				continue;
			}
			else
			{
				y_segment_index = (cell_index[0] - 1)*this->grid->resolution[1] + cell_index[1];
				countNum = this->GetCountNumInSegment(p->y, y1, y_segment_index, 1);
				int mark = countNum % 2;
				this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
				this->Rgpstat.multiplicationCount = this->Rgpstat.multiplicationCount + 2;
				this->Rgpstat.compareCount++;
				switch (mark)
				{
				case 1:
					this->Rgpstat.compareCount++;
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				continue;
			}//else
		}//if  


		//与网格线不重合
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		FLTYPE k, B;//斜率和截距
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;
		//获取当前网格中所含边的个数
		int edgenum = cur_cell->edgeCount;
		double min_a, max_a, min_b, max_b;

		this->Rgpstat.compareCount++;
		switch (edgenum)
		{
		case 1:                //网格中含一条边 //判断点在边的右侧还是左侧，写一个函数来判断  根据边的方向
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			min_a = p1->x < p2->x ? p1->x : p2->x;
			max_a = p1->x > p2->x ? p1->x : p2->x;
			min_b = p1->y < p2->y ? p1->y : p2->y;
			max_b = p1->y > p2->y ? p1->y : p2->y;
			this->Rgpstat.compareCount = this->Rgpstat.compareCount + 4;
			int num;
			//if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b) || (p->y >= max_b && tempy >= max_b))
			if ((this->Rgpstat.compareCount++&&p->x <= min_a) ||
				(this->Rgpstat.compareCount++&&p->x >= max_a) ||
				(this->Rgpstat.compareCount++&&p->y <= min_b &&
					this->Rgpstat.compareCount++&&tempy <= min_b) ||
				(this->Rgpstat.compareCount++&&p->y >= max_b &&
					this->Rgpstat.compareCount++&&tempy >= max_b))
			{
				if ((this->Rgpstat.compareCount++&&point_index[1] == 0) ||
					(this->Rgpstat.compareCount++&&point_index[1] == this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					this->Rgpstat.compareCount++;
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
						this->Rgpstat.addCount++;
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					this->Rgpstat.multiplicationCount++;
					this->Rgpstat.compareCount++;
					switch (mark)
					{
					case 1:
						this->Rgpstat.compareCount++;
						markflag = InvertFlag(markflag);
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
						break;
					}
				}//else
			}//if   测试边与多边形边不相交
			else                                                                                                           //测试边与多边形边相交  网格含一条边
			{
				this->testedResult[i] = this->GetRelativeLocation(cur_Rgp_edge_ref, p);
				//上述函数中包含的数据计算次数
				this->Rgpstat.compareCount = this->Rgpstat.compareCount + 3;
				this->Rgpstat.addCount = this->Rgpstat.addCount + 2;
				this->Rgpstat.multiplicationCount++;
				continue;
				break; 
			} //case1
		default:              //网格中含多条边
			int count = 0;
			FLTYPE miny = MAX_DOUBLE;
			FLTYPE tempvalue;
			//Point2D *minP1 = new Point2D();
			//Point2D *minP2 = new Point2D();

			//this->Rgpstat.compareCount++;
			while (cur_Rgp_edge_ref != NULL)
			{
				this->Rgpstat.compareCount++;
				p1 = cur_Rgp_edge_ref->start_e;
				p2 = cur_Rgp_edge_ref->end_e;
				//确定最大最小值
				min_a = p1->x < p2->x ? p1->x : p2->x;
				max_a = p1->x > p2->x ? p1->x : p2->x;
				min_b = p1->y < p2->y ? p1->y : p2->y;
				max_b = p1->y > p2->y ? p1->y : p2->y;
				this->Rgpstat.compareCount = this->Rgpstat.compareCount + 4;

				if ((this->Rgpstat.compareCount++&&p->x <= min_a) ||
					(this->Rgpstat.compareCount++&&p->x >= max_a) ||
					(this->Rgpstat.compareCount++&&p->y <= min_b &&
						this->Rgpstat.compareCount++&&tempy <= min_b) ||
					(this->Rgpstat.compareCount++&&p->y >= max_b && 
						this->Rgpstat.compareCount++&&tempy >= max_b))
				{
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					count++;
					this->Rgpstat.addCount++;
					continue;
				}//if 表示多边形边与测试边不相交
				else
				{
					k = cur_Rgp_edge_ref->k;
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					FLTYPE diff_y = fabs(temp_y_value - p->y);
					this->Rgpstat.addCount= this->Rgpstat.addCount+2;
					if (this->Rgpstat.multiplicationCount++&&diff_y < miny)
					{
						minP1 = p1;
						minP2 = p2;
						miny = diff_y;
						tempvalue = temp_y_value;
					}
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
				}//else  表示多边形边与测试边相交  获取最近的边
			}//while
			if (this->Rgpstat.compareCount++&&count == edgenum) //表示多边形边与测试边不相交
			{
				if ((this->Rgpstat.compareCount++&&point_index[1] == 0) || 
					(this->Rgpstat.compareCount++&&point_index[1] == this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					this->Rgpstat.compareCount++;
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
						this->Rgpstat.addCount++;
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					//统计数据
					this->Rgpstat.multiplicationCount++;
					this->Rgpstat.compareCount++;
					switch (mark)
					{
					case 1:
						markflag = InvertFlag(markflag);
						this->Rgpstat.compareCount++;
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
					}//switch
				}
			}//if//表示多边形边与测试边不相交
			else// 表示多边形边与测试边相交
			{
				int flag0 = minP1->x < minP2->x ? 0 : 1;
				int flag1 = p->y < tempvalue ? 2 : 4;
				int flag2 = flag0 + flag1;
				this->Rgpstat.compareCount = this->Rgpstat.compareCount + 3;
				this->Rgpstat.addCount++;
				switch (flag2)
				{
				case 2:
					this->testedResult[i] = 0;
					break;
				case 4:
					this->testedResult[i] = 1;
					break;
				case 3:
					this->testedResult[i] = 1;
					break;
				default:
					this->testedResult[i] = 0;
					break;
				}
			}
			break;  //default
		}//switch case
	}
	this->Rgpstat.compareCount++;
}

void GridPIP2D::OgpCheckPoint_Segment_noStat()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);
		if ((p->x < this->testedPointRegion[0].x) ||
			(p->x > this->testedPointRegion[1].x) ||
			(p->y < this->testedPointRegion[0].y) ||
			(p->y > this->testedPointRegion[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		if (cell_flag == CELL_OUT)
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		if (cell_flag == CELL_IN)
		{
			this->testedResult[i] = CELL_IN;
			continue;
		}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		int point_index[3];                                                                                      //用于临时存储网格交点的坐标
		int x_segment_index, y_segment_index;                                                   //用于表示网格线片段的坐标位置
		int up_or_down;                                                                                         //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		double tempy;                                                                                            //考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;         //网格中点坐标y值
		
		if (p->y <= tempy1)                                                                                  //向上做垂线
		{
			tempy = tempy1 - half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1];
			point_index[2] = cell_index[2] + cell_index[1];
			up_or_down = 0;
		}//if
		else                                                                                                           //向下做垂线
		{
			tempy = tempy1 + half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1] + 1;
			point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
			up_or_down = 1;
		}//else
		
		 //获取临近网格交点的坐标
		FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;
		int  markflag = this->RgpPoint[point_index[2]].flag;                                                   //获取网格点的属性
		int countNum = 0;                                                                                                        //记录交点数总数

		//平行X轴
		if (Equal(y1, p->y, 10e-10) == 1)
		{
			if (Equal(p->y, this->grid->boundingBox[0].y, 10e-6) == 1
				|| Equal(p->y, this->grid->boundingBox[1].y, 10e-6) == 1)
			{
				this->testedResult[i] = markflag;
				continue;
			}//if
			else
			{
				if (up_or_down == 0)
				{
					x_segment_index = cell_index[2] - this->grid->resolution[0];
				}
				else
				{
					x_segment_index = cell_index[2];
				}
				countNum = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);                            //获取指定网格线片段中，指定两坐标值之间的数量。  0表示横轴，1表示纵轴
				//根据交点总数判断测试点的位置属性
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				//continue;
			}//else
		}

		//平行Y轴
		if (Equal(x1, p->x, 10e-10) == 1)
		{
			if (Equal(p->x, this->grid->boundingBox[0].x, 10e-6) == 1
				|| Equal(p->x, this->grid->boundingBox[1].x, 10e-6) == 1)
			{
				this->testedResult[i] = markflag;
				continue;
			}
			else
			{
				y_segment_index = (cell_index[0] - 1)*this->grid->resolution[1] + cell_index[1];
				countNum = this->GetCountNumInSegment_noStat(p->y, y1, y_segment_index, 1);
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				continue;
			}//else
		}//if  

		//与网格线不重合
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		FLTYPE k, B;//斜率和截距
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;
		//获取当前网格中所含边的个数
		int edgenum = cur_cell->edgeCount;
		double min_a, max_a, min_b, max_b;
		switch (edgenum)
		{
		case 1:                //网格中含一条边 //判断点在边的右侧还是左侧，写一个函数来判断  根据边的方向
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			min_a = p1->x < p2->x ? p1->x : p2->x;
			max_a = p1->x > p2->x ? p1->x : p2->x;
			min_b = p1->y < p2->y ? p1->y : p2->y;
			max_b = p1->y > p2->y ? p1->y : p2->y;
			int num;
			if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b) || (p->y >= max_b && tempy >= max_b))
			{
				if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					switch (mark)
					{
					case 1:
						markflag = InvertFlag(markflag);
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
						break;
					}
				}//else
			}//if   测试边与多边形边不相交
			else                                                                                                           //测试边与多边形边相交  网格含一条边
			{
				this->testedResult[i] = this->GetRelativeLocation(cur_Rgp_edge_ref, p);
				continue;
				break;
			} //case1
		default:              //网格中含多条边
			int count = 0;
			FLTYPE miny = MAX_DOUBLE;
			FLTYPE tempvalue;
			/*Point2D *minP1 = new Point2D();
			Point2D *minP2 = new Point2D();*/
			while (cur_Rgp_edge_ref != NULL)
			{
				p1 = cur_Rgp_edge_ref->start_e;
				p2 = cur_Rgp_edge_ref->end_e;
				//确定最大最小值
				min_a = p1->x < p2->x ? p1->x : p2->x;
				max_a = p1->x > p2->x ? p1->x : p2->x;
				min_b = p1->y < p2->y ? p1->y : p2->y;
				max_b = p1->y > p2->y ? p1->y : p2->y;
				if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b) || (p->y >= max_b && tempy >= max_b))
				{
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					count++;
					continue;
				}//if 表示多边形边与测试边不相交
				else
				{
					k = cur_Rgp_edge_ref->k;   ///考虑不用定义k和B 直接调用指针指向的数据
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					FLTYPE diff_y = fabs(temp_y_value - p->y);
					if (diff_y < miny)
					{
						minP1 = p1;
						minP2 = p2;
						miny = diff_y;
						tempvalue = temp_y_value;
					}
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
				}//else  表示多边形边与测试边相交  获取最近的边
			}//while

			if (count == edgenum) //表示多边形边与测试边不相交
			{
				if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					switch (mark)
					{
					case 1:
						markflag = InvertFlag(markflag);
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
					}//switch
				}
			}//if//表示多边形边与测试边不相交
			else// 表示多边形边与测试边相交
			{
				int flag0 = minP1->x < minP2->x ? 0 : 1;
				int flag1 = p->y < tempvalue ? 2 : 4;
				int flag2 = flag0 + flag1;
				switch (flag2)
				{
				case 2:
					this->testedResult[i] = 0;
					break;
				case 4:
					this->testedResult[i] = 1;
					break;
				case 3:
					this->testedResult[i] = 1;
					break;
				default:
					this->testedResult[i] = 0;
					break;
				}
			}//else
			break;  //default
		}//switch case
	}
}

//Group handle method  based on orentation
void GridPIP2D::G_OgpCheckPoint_Segment_noStat()
{
	FLTYPE half_cell_size[2];
	half_cell_size[0] = this->grid->cellSize[0] / 2;
	half_cell_size[1] = this->grid->cellSize[1] / 2;

	for (int i = 0; i < this->testedPointCount; i++)
	{
		Point2D * p = &(this->testedPoint[i]);
		if ((p->x < this->testedPointRegion[0].x) ||
			(p->x > this->testedPointRegion[1].x) ||
			(p->y < this->testedPointRegion[0].y) ||
			(p->y > this->testedPointRegion[1].y))
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//确定被测点所在单元
		int cell_index[3]; //0：x方向索引；1：y方向索引；2：总索引
		cell_index[2] = this->grid->locatePoint(p->x, p->y, cell_index[0], cell_index[1]);
		int cell_flag = this->grid->cell[cell_index[2]].cell_InofOut;
		// 如果网格单元为多边形外，直接返回测试点的位置属性为外
		if (cell_flag == CELL_OUT)
		{
			this->testedResult[i] = CELL_OUT;
			continue;
		}
		//如果网格单元为多边形内，直接返回测试的位置属性为内
		if (cell_flag == CELL_IN)
		{
			this->testedResult[i] = CELL_IN;
			continue;
		}

		//含有多边形边的网格
		//调整垂线的趋向：向上或向下
		int point_index[3];                                                                                      //用于临时存储网格交点的坐标
		int x_segment_index, y_segment_index;                                                   //用于表示网格线片段的坐标位置
		int up_or_down;                                                                                         //记录向上做垂线还是向下做垂线   0表示向上，1表示向下
		double tempy;                                                                                            //考虑预处理存储在网格单元中
		FLTYPE tempy1 = this->grid->cell[cell_index[2]].cell_middle_y;         //网格中点坐标y值

		if (p->y <= tempy1)                                                                                  //向上做垂线
		{
			tempy = tempy1 - half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1];
			point_index[2] = cell_index[2] + cell_index[1];
			up_or_down = 0;
		}//if
		else                                                                                                           //向下做垂线
		{
			tempy = tempy1 + half_cell_size[1];
			point_index[0] = cell_index[0];
			point_index[1] = cell_index[1] + 1;
			point_index[2] = cell_index[2] + cell_index[1] + this->grid->resolution[0] + 1;
			up_or_down = 1;
		}//else

		 //获取临近网格交点的坐标
		FLTYPE x1 = this->RgpPoint[point_index[2]].gridPoint.x;
		FLTYPE y1 = this->RgpPoint[point_index[2]].gridPoint.y;
		int  markflag = this->RgpPoint[point_index[2]].flag;                                                   //获取网格点的属性
		int countNum = 0;                                                                                                        //记录交点数总数

		//平行X轴
		if (Equal(y1, p->y, 10e-10) == 1)
		{
			if (Equal(p->y, this->grid->boundingBox[0].y, 10e-6) == 1
				|| Equal(p->y, this->grid->boundingBox[1].y, 10e-6) == 1)
			{
				this->testedResult[i] = markflag;
				continue;
			}//if
			else
			{
				if (up_or_down == 0)
				{
					x_segment_index = cell_index[2] - this->grid->resolution[0];
				}
				else
				{
					x_segment_index = cell_index[2];
				}
				countNum = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);                            //获取指定网格线片段中，指定两坐标值之间的数量。  0表示横轴，1表示纵轴
				//根据交点总数判断测试点的位置属性
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				//continue;
			}//else
		}

		//平行Y轴
		if (Equal(x1, p->x, 10e-10) == 1)
		{
			if (Equal(p->x, this->grid->boundingBox[0].x, 10e-6) == 1
				|| Equal(p->x, this->grid->boundingBox[1].x, 10e-6) == 1)
			{
				this->testedResult[i] = markflag;
				continue;
			}
			else
			{
				y_segment_index = (cell_index[0] - 1)*this->grid->resolution[1] + cell_index[1];
				countNum = this->GetCountNumInSegment_noStat(p->y, y1, y_segment_index, 1);
				int mark = countNum % 2;
				switch (mark)
				{
				case 1:
					markflag = InvertFlag(markflag);
					this->testedResult[i] = markflag;
					continue;
					break;
				default:
					this->testedResult[i] = markflag;
					continue;
					break;
				}
				continue;
			}//else
		}//if  

		//与网格线不重合
		GridCell2D *cur_cell = &(this->grid->cell[cell_index[2]]);
		//此处得到网格单元中均含有边
		Point2D* p1, *p2;
		FLTYPE k, B;//斜率和截距
		RGPEdgeRef2D* Rgp_edge_ref = cur_cell->RgpEdgeRef;
		RGPEdgeRef2D* cur_Rgp_edge_ref = Rgp_edge_ref;
		//获取当前网格中所含边的个数
		int edgenum = cur_cell->edgeCount;
		double min_a, max_a, min_b, max_b;
		switch (edgenum)
		{
		case 1:                //网格中含一条边 //判断点在边的右侧还是左侧，写一个函数来判断  根据边的方向
			p1 = cur_Rgp_edge_ref->start_e;
			p2 = cur_Rgp_edge_ref->end_e;
			//确定最大最小值
			min_a = p1->x < p2->x ? p1->x : p2->x;
			max_a = p1->x > p2->x ? p1->x : p2->x;
			min_b = p1->y < p2->y ? p1->y : p2->y;
			max_b = p1->y > p2->y ? p1->y : p2->y;
			int num;
			if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b) || (p->y >= max_b && tempy >= max_b))
			{
				if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					switch (mark)
					{
					case 1:
						markflag = InvertFlag(markflag);
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
						break;
					}
				}//else
			}//if   测试边与多边形边不相交
			else                                                                                                           //测试边与多边形边相交  网格含一条边
			{
				this->testedResult[i] = this->GetRelativeLocation(cur_Rgp_edge_ref, p);
				continue;
				break;
			} //case1
		default:              //网格中含多条边
			int count = 0;
			FLTYPE miny = MAX_DOUBLE;
			FLTYPE tempvalue;
			/*Point2D *minP1 = new Point2D();
			Point2D *minP2 = new Point2D();*/
			while (cur_Rgp_edge_ref != NULL)
			{
				p1 = cur_Rgp_edge_ref->start_e;
				p2 = cur_Rgp_edge_ref->end_e;
				//确定最大最小值
				min_a = p1->x < p2->x ? p1->x : p2->x;
				max_a = p1->x > p2->x ? p1->x : p2->x;
				min_b = p1->y < p2->y ? p1->y : p2->y;
				max_b = p1->y > p2->y ? p1->y : p2->y;
				if (p->x <= min_a || p->x >= max_a || (p->y <= min_b && tempy <= min_b) || (p->y >= max_b && tempy >= max_b))
				{
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
					count++;
					continue;
				}//if 表示多边形边与测试边不相交
				else
				{
					k = cur_Rgp_edge_ref->k;   ///考虑不用定义k和B 直接调用指针指向的数据
					B = cur_Rgp_edge_ref->B;
					FLTYPE temp_y_value = k * p->x + B;   //垂线与多边形边片段的交点坐标
					FLTYPE diff_y = fabs(temp_y_value - p->y);
					if (diff_y < miny)
					{
						minP1 = p1;
						minP2 = p2;
						miny = diff_y;
						tempvalue = temp_y_value;
					}
					cur_Rgp_edge_ref = cur_Rgp_edge_ref->next;
				}//else  表示多边形边与测试边相交  获取最近的边
			}//while

			if (count == edgenum) //表示多边形边与测试边不相交
			{
				if (point_index[1] == 0 || point_index[1] == (this->grid->resolution[1]))
				{
					this->testedResult[i] = markflag;
					continue;
				}
				else
				{
					if (up_or_down == 0)
					{
						x_segment_index = cell_index[2] - this->grid->resolution[0];
					}
					else
					{
						x_segment_index = cell_index[2];
					}
					num = this->GetCountNumInSegment_noStat(p->x, 0, x_segment_index, 0);// 0表示横轴，1表示纵轴
					int mark = num % 2;
					switch (mark)
					{
					case 1:
						markflag = InvertFlag(markflag);
						this->testedResult[i] = markflag;
						continue;
						break;
					default:
						this->testedResult[i] = markflag;
						continue;
					}//switch
				}
			}//if//表示多边形边与测试边不相交
			else// 表示多边形边与测试边相交
			{
				int flag0 = minP1->x < minP2->x ? 0 : 1;
				int flag1 = p->y < tempvalue ? 2 : 4;
				int flag2 = flag0 + flag1;
				switch (flag2)
				{
				case 2:
					this->testedResult[i] = 0;
					break;
				case 4:
					this->testedResult[i] = 1;
					break;
				case 3:
					this->testedResult[i] = 1;
					break;
				default:
					this->testedResult[i] = 0;
					break;
				}
			}//else
			break;  //default
		}//switch case
	}
}

//判断测试点在临近边的左侧还是右侧，有向边的右侧为多边形内部
//return 0代表外部，return 1代表内部
int GridPIP2D::GetRelativeLocation(RGPEdgeRef2D* cur_Ogp_edge_ref, Point2D*q)
{
	Point2D* p1= cur_Ogp_edge_ref->start_e;
	Point2D* p2= cur_Ogp_edge_ref->end_e;

	int flag0, flag1, flag2;
	FLTYPE k, B;//斜率和截距
	k = cur_Ogp_edge_ref->k;
	B = cur_Ogp_edge_ref->B;
	FLTYPE y0=k * q->x + B;   //垂线与多边形边片段的交点坐标
	flag0 = p1->x < p2->x?0:1;
	flag1 = q->y < y0?2:4;
	flag2 = flag0 + flag1;
	switch (flag2)
	{
	case 2:
		return 0;
		break;
	case 4:
		return 1;
		break;
	case 3:
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

//快排
void GridPIP2D::quicksort(FLTYPE a[], int left, int right)
{
	int i, j;
	FLTYPE temp, t;
	if (left > right)   //（递归过程先写结束条件）
		return;

	temp = a[left]; //temp中存的就是基准数  
	i = left;
	j = right;
	while (i != j)
	{
		//顺序很重要，要先从右边开始找（最后交换基准时换过去的数要保证比基准小，因为基准                               
		//选取数组第一个数，在小数堆中） 
		while (a[j] >= temp && i < j)
			j--;
		//再找右边的  
		while (a[i] <= temp && i < j)
			i++;
		//交换两个数在数组中的位置  
		if (i < j)
		{
			t = a[i];
			a[i] = a[j];
			a[j] = t;
		}
	}
	//最终将基准数归位 （之前已经temp=a[left]过了，交换只需要再进行两步）
	a[left] = a[i];
	a[i] = temp;

	quicksort(a,left, i - 1);//继续处理左边的，这里是一个递归的过程  
	quicksort(a,i + 1, right);//继续处理右边的 ，这里是一个递归的过程  
	//原文：https ://blog.csdn.net/yushiyi6453/article/details/76407640 
}