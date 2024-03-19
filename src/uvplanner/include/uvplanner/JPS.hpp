#pragma once

#include "SearchBased.hpp"

namespace UV
{
///Search and prune neighbors for JPS 3D
struct JPS3DNeib 
{
	// for each (dx,dy,dz) these contain:
	//    ns: neighbors that are always added
	//    f1: forced neighbors to check
	//    f2: neighbors to add if f1 is forced
	int ns[27][3][26];
	int f1[27][3][12];
	int f2[27][3][12];
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
	JPS3DNeib();
	private:
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz,
	    int& nx, int& ny, int& nz);
};

class JPSNode;
typedef JPSNode* JPSNodePtr;
class JPSNode
{
public:
    JPSNode(Status st)
    {
        status = st;
        cameFrom = nullptr;
    };
    JPSNode(Vector3i idx)
    {
        index = idx;
        cameFrom = nullptr;
    };
    JPSNode(int x, int y, int z=0)
    {
        index = Vector3i(x,y,z);
        cameFrom = nullptr;
    };
    JPSNodePtr cameFrom;
    int flag=0;
    double gScore=inf_d;
    double fScore=inf_d;
    Status status;
    Vector3i index;
    Vector3i dir;
};

class JPS :   public PathSearcher
{
public:
    JPS(){jn3d = new JPS3DNeib();}
    ~JPS(){delete jn3d;}
    void reset();
    void setMap(uint8_t* pmap,int l,int w,int h=1);
    bool solve(Status start, Status goal);
    double getHeu(Vector3i p1, Vector3i p2);
    void getNeighbour(Vector3i cur,std::vector<JPSNodePtr> &nlist,std::vector<double> &ncost);
    bool hasForced(const Vector3i & idx, const Vector3i & dir);
    bool jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx);
private:
    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isOccupied(const Vector3i & index) const;
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    bool isFree(const Vector3i & index) const;
    Vector3i goalIdx;
    JPS3DNeib * jn3d;
    JPSNode* currentPtr;
    std::multimap<double, JPSNodePtr> openSet;
    JPSNodePtr*** nodeMap=nullptr;
};
}