#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "GridNode.hpp"

using std::min;
using std::max;

namespace UV
{

/*typedef std::function<double(Status,Status)> HeuFunction;
typedef std::function<void(Status ,std::vector<Status> & ,std::vector<double> & )> SExpandFunction;
typedef std::function<void(Vector3i ,std::vector<Vector3i> & ,std::vector<double> & )> IExpandFunction;*/

class PathSearcher
{
public:
    virtual void reset() = 0;
    virtual void setMap(uint8_t* pmap,int l,int w,int h=1) = 0;
    void setScale(double ls, double ws, double hs=1){scaleLenth=ls,scaleWidth=ws,scaleHight=hs;};
    void setOri(double x, double y, double z){ori_x=x;ori_y=y;ori_z=z;};
    /*void setHeu(HeuFunction h){getHeu = h;};
    void setExpand(IExpandFunction e){getNeighbour = e;};*/
    //virtual void setObs(int x,int y,int z=0);
    //virtual void plan(GridNode& start, GridNode& goal) = 0;
    virtual bool solve(Status start, Status goal) = 0;
    Status cvtIndex2Status(Vector3i & index)
    {
        Status st;

        st.x() = ((double)index(0)) * scaleLenth + ori_x;
        st.y() = ((double)index(1)) * scaleWidth + ori_y;
        st.z() = ((double)index(2)) * scaleHight + ori_z;

        return st;
    }
    Vector3i cvtStatus2Index(Status & st)
    {
        Vector3i idx;
        idx <<  int( (st.x()-ori_x) / scaleLenth),
                int( (st.y()-ori_y) / scaleWidth),
                int( (st.z()-ori_z) / scaleHight);                  
  
        return idx;
    }


protected:
    int mapLength=0,mapWidth=0,mapHight=0;
    double scaleLenth=1,scaleWidth=1,scaleHight=1;
    double ori_x=0,ori_y=0,ori_z=0;
    /*HeuFunction getHeu;
    IExpandFunction getNeighbour;*/
    uint8_t* mapData=nullptr;
public:
    std::vector<Status> path;
};

} // namespace UV

#include "AStar.hpp"
