/*
 * 模块功能：SDK点云、二维几何和标定数据结构声明。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#ifndef LIDAR_CALIBRATION
#define LIDAR_CALIBRATION

#include "lidar_information.h"
#include <vector>

using namespace std;

const int map_ratio = 40;

// 定义二维向量结构体
struct Vector2D {
    double x;
    double y;

    // 计算向量的模长
    // 功能：实现length的基础运算。
    double Length() {
        return sqrt(x * x + y * y);
    }

    // 计算向量的点积
    // 功能：实现dot的基础运算。
    double Dot(Vector2D v) {
        return x * v.x + y * v.y;
    }

    // 计算向量的叉积
    // 功能：实现cross的基础运算。
    double Cross(Vector2D v) {
        return x * v.y - y * v.x;
    }

    // 计算向量的夹角（单位：弧度）
    // 功能：实现角度的基础运算。
    double Angle(Vector2D v) {
        double dot = Dot(v);
        double cross = Cross(v);
        return atan2(cross, dot);
    }
};

// 定义线段结构体
struct LineSegment {
    Vector2D p1;
    Vector2D p2;
};


typedef struct 
{
    float x,y;
    float theta;
}pos_t;

typedef struct _point_
{
    float x , y , q ;
    int index ;
    float angle;
    float range;
    bool change = false;
    uint16_t intensity;
}point_t;

typedef struct _point_cloud_
{
    std::vector<point_t> data;
    pos_t origin;
    point_t origin_point;
}pcloud_t;

struct point_lines{
  point_t line_start;
  point_t line_end;
};

typedef struct _border_frame_
{
    int top,bottom;
    int left,right;
}border_frame_t;

class coord_tt
{
    public:
    int x,y;
    float theta;
    // 功能：执行`coord_tt`，负责本模块对应的coord、tt处理。
    coord_tt(int x = 0, int y = 0, float theta = 0.0)
        :x(x), y(y), theta(theta){}
    // 按栅格偏移量移动当前坐标。
    void move(coord_tt);
    // 将当前栅格坐标转换为带角度的位姿结构。
    pos_t pos();
};

struct Line_new {
    double slope;
    double intercept;
};

//栅格坐标
typedef struct grid
{
    int x,y;
    // 功能：实现operator的基础运算。
    bool operator==(const grid &b)const{return x == b.x && y == b.y;}
    // 功能：实现operator的基础运算。
    bool operator!=(const grid &b)const{return x != b.x || y != b.y;}
    // 功能：实现operator的基础运算。
    bool operator<(const grid &b)const {if(x == b.x) return y < b.y; return x < b.x;}
    // 功能：实现operator的基础运算。
    bool operator>(const grid &b)const {if(x == b.x) return y > b.y; return x > b.x;}
    // 功能：实现operator的基础运算。
    grid operator+(const grid &b) {return {x + b.x, y + b.y};}
    // 功能：实现operator的基础运算。
    grid operator-(const grid &b) {return {x - b.x, y - b.y};}
    
    // 功能：执行`grid`，负责本模块对应的占用栅格处理。
    grid () {x = y = 0;}
    // 功能：执行`grid`，负责本模块对应的占用栅格处理。
    grid (int x_, int y_) {x = x_; y = y_;}
    // 功能：执行`grid`，负责本模块对应的占用栅格处理。
    grid (coord_tt coord){x = coord.x; y = coord.y;}
    
    // 功能：执行`near`，负责本模块对应的near处理。
    bool near(grid &b)  {return abs(x - b.x) <= 1 && abs(y - b.y) <= 1;}
    //bool vaild() {return x >= 0 && y >= 0 && x < map_width_limit && y < map_width_limit;}
    // 功能：执行move相关的运动或时序辅助处理。
    void move(grid b) {x += b.x; y += b.y;} 
}grid_t;

typedef struct
{
    float dist;
    float ratio;
    grid_t point;
}p2seg_ret_t;


typedef struct _calibration_params_
{
  float distortion_processing_distance = 500;
 
  point_t origin_point;

  int calibration_type = 0;   // 0：剔除畸变，1：拉直畸变。

  int line_judgment_point_num;//判断提取一条直线的最小点数
}calibration_params_t;

extern calibration_params_t calibration_params;

/*畸变处理入口函数*/
void lidar_calibration(LaserScan &outscan);

/*获取直线的斜率和截距*/
void getLineKB(point_t start_point, point_t end_point, double &k, double &b);

/*点到直线的距离*/
double pointToLineDist(point_t point, double k, double b);

/*雷达数据转换为点云数据*/
void pcloud_build_by_pos_scan(LaserScan &outscan ,pcloud_t &pcloud);

/*拟合直线进行畸变处理的入口*/
void fitting_line(pcloud_t &pcloud);


/*直线的过程判断*/
bool line_judgment(std::vector<point_t> line, point_t point);

/*求相邻线段的夹角*/
void neighbor_lines_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud);

/*进行直角畸变处理*/
void Point_cloud_straightening_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud);

/*求两点间距离*/
float pointTopointDist(point_t a, point_t b);

/*判断点是否在直线的左侧*/
int isLeft(point_t p, point_t a, point_t b);

/*针对直线进行拉直处理*/
void straightening_process_new(std::vector<point_t> line_point, point_t a, point_t b, pcloud_t &pcloud);

/*求两个直线是否相交，并计算交点*/
bool intersect(LineSegment line1, LineSegment line2, Vector2D &intersection);

/*求两条直线的交点*/
point_t calculateIntersection(Line_new line1, Line_new line2);



#endif
