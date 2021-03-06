#include <vector>
#include <ros/ros.h>
#include <list>
#include <cstddef>
 
const int kCost1=10; 
const int kCost2=14; 
 
struct Point
{
	int x,y; 
	int F,G,H; 
	Point *parent; 
	Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL) 
	{
	}
};
 
 
class Astar
{
public:
	void InitAstar(std::vector<std::vector<int>> &_maze);
	std::list<Point *> GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
 
private:
	Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
	std::vector<Point *> getSurroundPoints(const Point *point,bool isIgnoreCorner) ;
	bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) ; 
	Point *isInList(const std::list<Point *> &list,const Point *point) ; 
	Point *getLeastFpoint(); 
	int calcG(Point *temp_start,Point *point);
	int calcH(Point *point,Point *end);
	int calcF(Point *point);
private:
	std::vector<std::vector<int>> maze;
	std::list<Point *> openList;  
	std::list<Point *> closeList; 
};
