#include "JPS.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace UV;

void JPS::reset()
{
    path.clear();
    openSet.clear();
    for(int height = 0; height < mapHight; height++)
    {
        for(int length = 0; length < mapLength; length++)
        {
            for(int width = 0; width < mapWidth; width++)
            {
                nodeMap[height][width][length]->cameFrom = nullptr;
                nodeMap[height][width][length]->flag = 0;
                nodeMap[height][width][length]->fScore = inf_d;
                nodeMap[height][width][length]->gScore = inf_d;
            }
        }
    }
}

void JPS::setMap(uint8_t *pmap, int l, int w, int h)
{
    mapHight = h;
    mapLength = l;
    mapWidth = w;
    if(mapData != nullptr) delete[] mapData;
    mapData = new uint8_t[l*w*h];
    memcpy(mapData,pmap,l*w*h);
    if(nodeMap != nullptr) delete[] nodeMap;
    nodeMap = new JPSNodePtr** [mapHight];
    for(int height = 0; height < mapHight; height++)
    {
        nodeMap[height] = new JPSNodePtr* [mapWidth];
        for(int width = 0; width < mapWidth; width++)
        {
            nodeMap[height][width] = new JPSNodePtr [mapLength];
            for(int length = 0; length < mapLength; length++)
            {
                nodeMap[height][width][length] = new JPSNode(length,width,height);
            }
        }
    }
}

bool JPS::solve(Status start, Status goal)
{
    Vector3i startIndex = cvtStatus2Index(start);
    Vector3i endIndex   = cvtStatus2Index(goal);
    goalIdx = endIndex;

    std::cout << startIndex << endIndex << std::endl;

    JPSNodePtr startPtr = new JPSNode(startIndex);
    JPSNodePtr endPtr   = new JPSNode(endIndex);
    currentPtr = nullptr;
    
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr->index,endPtr->index);
    startPtr -> flag = 1; 

    startPtr->status = start;
    endPtr->status = goal;

    openSet.insert(std::make_pair(startPtr -> fScore, startPtr));

    std::vector<JPSNodePtr> neighborSets;
    std::vector<double> edgeCostSets;

    RCLCPP_INFO(rclcpp::get_logger("jps"),"test");

    while ( !openSet.empty() )
    {
        auto lowCostPair = openSet.begin();
        currentPtr = lowCostPair->second;
        openSet.erase(lowCostPair);
        currentPtr->flag = -1;
        if( currentPtr->index == endIndex )
        {
            ///TODO: generate statue flow
            path.push_back(goal);
            if(currentPtr->cameFrom!=nullptr)
                currentPtr = currentPtr->cameFrom;
            while (currentPtr->cameFrom!=nullptr)
            {
                Status p = cvtIndex2Status(currentPtr->index);
                path.push_back(p);
                currentPtr = currentPtr->cameFrom;
            }
            path.push_back(start);
            return true;
        }

        getNeighbour(currentPtr->index,neighborSets,edgeCostSets);
        RCLCPP_INFO(rclcpp::get_logger("jps"),"test");

        for(int i = 0; i < (int)neighborSets.size(); i++)
        {
            JPSNodePtr neighborPtr = neighborSets.at(i);
            
            if(neighborPtr->flag == 0)
            {
                neighborPtr->cameFrom = currentPtr;
                
                neighborPtr -> gScore = getHeu(neighborPtr->index,currentPtr->index) + currentPtr->gScore;
                neighborPtr -> fScore = getHeu(neighborPtr->index,endPtr->index)+neighborPtr -> gScore;
        
                neighborPtr -> flag = 1;
                openSet.insert( std::make_pair(neighborPtr -> fScore, neighborPtr));
                continue;
            }
            else if(neighborPtr -> flag == 1)
            {
                double newGScore = currentPtr->gScore + edgeCostSets.at(i);
                if(neighborPtr->gScore > newGScore)
                {
                    auto neighborRange = openSet.equal_range(neighborPtr->fScore);
                    neighborPtr->gScore = newGScore;
                    neighborPtr->fScore = newGScore + getHeu(neighborPtr->index,endPtr->index);
                    neighborPtr->cameFrom = currentPtr;
                    //if(neighborRange.first == end(openSet)) continue;
                    for(auto i = neighborRange.first; i != neighborRange.second; i++)
                    {
                        if(i->second == neighborPtr)
                        {
                            openSet.erase(i);
                            openSet.insert(std::make_pair(neighborPtr->fScore,neighborPtr));
                            break;
                        }
                    }
                }
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
                continue;
            }
        }
    }

    return false;
}

double UV::JPS::getHeu(Vector3i p1, Vector3i p2)
{
    /*Vector3d delta = Vector3d(p1.x()-p2.x(),p1.y()-p2.y(),p1.z()-p2.z());
    double heu = sqrt(delta.x()*delta.x()+delta.y()*delta.y()+delta.z()*delta.z());*/
    double dx = abs((double)(p1.x()-p2.x()));
    double dy = abs((double)(p1.y()-p2.y()));
    double dz = abs((double)(p1.z()-p2.z()));

    double go_3 = min(dx,min(dy,dz));

    dx -= go_3;
    dy -= go_3;
    dz -= go_3;

    double go_2 = dx + dy + dz - max(dx,max(dy,dz));

    dx -= go_2;
    dy -= go_2;
    dz -= go_2;

    double go_1 = max(dx,max(dy,dz));

    return sqrt(3)*go_3 + sqrt(2)*go_2 + go_1;
}

void UV::JPS::getNeighbour(Vector3i cur, std::vector<JPSNodePtr> &nlist, std::vector<double> &ncost)
{
    nlist.clear();
    ncost.clear();
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if( dev < num_neib ) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        JPSNodePtr nodePtr = nodeMap[neighborIdx(2)][neighborIdx(1)][neighborIdx(0)];
        nodePtr->dir = expandDir;
        
        nlist.push_back(nodePtr);
        ncost.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

bool UV::JPS::hasForced(const Vector3i &idx, const Vector3i &dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

bool UV::JPS::jump(const Vector3i &curIdx, const Vector3i &expDir, Vector3i &neiIdx)
{
    neiIdx = curIdx + expDir;

    if( !isFree(neiIdx) )
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

bool UV::JPS::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return  (idx_x >= 0 && idx_x < mapLength && idx_y >= 0 && idx_y < mapWidth && idx_z >= 0 && idx_z < mapHight && 
            (mapData[idx_z * mapLength*mapWidth + idx_y * mapWidth + idx_x] == 1));
}

bool UV::JPS::isOccupied(const Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

bool UV::JPS::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < mapLength && idx_y >= 0 && idx_y < mapWidth && idx_z >= 0 && idx_z < mapHight && 
           (mapData[idx_z * mapLength*mapWidth + idx_y * mapWidth + idx_x] != 1));
}

bool UV::JPS::isFree(const Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

constexpr int JPS3DNeib::nsz[4][2];
JPS3DNeib::JPS3DNeib() 
{
    int id = 0;
    for(int dz = -1; dz <= 1; ++ dz) {
        for(int dy = -1; dy <= 1; ++ dy) {
            for(int dx = -1; dx <= 1; ++ dx) {
                int norm1 = abs(dx) + abs(dy) + abs(dz);
            
                for(int dev = 0; dev < nsz[norm1][0]; ++ dev)
                    Neib(dx,dy,dz,norm1,dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
            
                for(int dev = 0; dev < nsz[norm1][1]; ++ dev){
                    FNeib(dx,dy,dz,norm1,dev,
                    f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
                    f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
                }
                
                id ++;
            }
        }
    }
}


void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
    switch(norm1){
        case 0:
            switch(dev){
                case 0:  tx=1;  ty=0;  tz=0;  return;
                case 1:  tx=-1; ty=0;  tz=0;  return;
                case 2:  tx=0;  ty=1;  tz=0;  return;
                case 3:  tx=1;  ty=1;  tz=0;  return;
                case 4:  tx=-1; ty=1;  tz=0;  return;
                case 5:  tx=0;  ty=-1; tz=0;  return;
                case 6:  tx=1;  ty=-1; tz=0;  return;
                case 7:  tx=-1; ty=-1; tz=0;  return;
                case 8:  tx=0;  ty=0;  tz=1;  return;
                case 9:  tx=1;  ty=0;  tz=1;  return;
                case 10: tx=-1; ty=0;  tz=1;  return;
                case 11: tx=0;  ty=1;  tz=1;  return;
                case 12: tx=1;  ty=1;  tz=1;  return;
                case 13: tx=-1; ty=1;  tz=1;  return;
                case 14: tx=0;  ty=-1; tz=1;  return;
                case 15: tx=1;  ty=-1; tz=1;  return;
                case 16: tx=-1; ty=-1; tz=1;  return;
                case 17: tx=0;  ty=0;  tz=-1; return;
                case 18: tx=1;  ty=0;  tz=-1; return;
                case 19: tx=-1; ty=0;  tz=-1; return;
                case 20: tx=0;  ty=1;  tz=-1; return;
                case 21: tx=1;  ty=1;  tz=-1; return;
                case 22: tx=-1; ty=1;  tz=-1; return;
                case 23: tx=0;  ty=-1; tz=-1; return;
                case 24: tx=1;  ty=-1; tz=-1; return;
                case 25: tx=-1; ty=-1; tz=-1; return;
            }
        case 1:
            tx = dx; ty = dy; tz = dz; return;
        case 2:
            switch(dev){
                case 0:
                    if(dz == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = 0; ty = 0; tz = dz; return;
                    }
                case 1:
                    if(dx == 0){
                        tx = 0; ty = dy; tz = 0; return;
                    }else{
                        tx = dx; ty = 0; tz = 0; return;
                    }
                case 2:
                    tx = dx; ty = dy; tz = dz; return;
            }
        case 3:
            switch(dev){
                case 0: tx = dx; ty =  0; tz =  0; return;
                case 1: tx =  0; ty = dy; tz =  0; return;
                case 2: tx =  0; ty =  0; tz = dz; return;
                case 3: tx = dx; ty = dy; tz =  0; return;
                case 4: tx = dx; ty =  0; tz = dz; return;
                case 5: tx =  0; ty = dy; tz = dz; return;
                case 6: tx = dx; ty = dy; tz = dz; return;
            }
    }
}

void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
    switch(norm1){
        case 1:
            switch(dev){
                case 0: fx= 0; fy= 1; fz = 0; break;
                case 1: fx= 0; fy=-1; fz = 0; break;
                case 2: fx= 1; fy= 0; fz = 0; break;
                case 3: fx= 1; fy= 1; fz = 0; break;
                case 4: fx= 1; fy=-1; fz = 0; break;
                case 5: fx=-1; fy= 0; fz = 0; break;
                case 6: fx=-1; fy= 1; fz = 0; break;
                case 7: fx=-1; fy=-1; fz = 0; break;
            }
            nx = fx; ny = fy; nz = dz;
            // switch order if different direction
            if(dx != 0){
                fz = fx; fx = 0;
                nz = fz; nx = dx;
            }

            if(dy != 0){
                fz = fy; fy = 0;
                nz = fz; ny = dy;
            }
            return;
        case 2:
            if(dx == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = 0; ny = dy; nz = -dz;
                        return;
                    case 1:
                        fx = 0; fy = -dy; fz = 0;
                        nx = 0; ny = -dy; nz = dz;
                        return;
                    case 2:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = dz;
                        return;
                    case 3:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = dz;
                        return;
                    case 4:
                        fx = 1; fy = 0; fz = -dz;
                        nx = 1; ny = dy; nz = -dz;
                        return;
                    case 5:
                        fx = 1; fy = -dy; fz = 0;
                        nx = 1; ny = -dy; nz = dz;
                        return;
                    case 6:
                        fx = -1; fy = 0; fz = -dz;
                        nx = -1; ny = dy; nz = -dz;
                        return;
                    case 7:
                        fx = -1; fy = -dy; fz = 0;
                        nx = -1; ny = -dy; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = dy; nz = 0;
                        return;
                    case 9:
                        fx = 1; fy = 0; fz = 0;
                        nx = 1; ny = 0; nz = dz;
                        return;
                    case 10:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = dy; nz = 0;
                        return;
                    case 11:
                        fx = -1; fy = 0; fz = 0;
                        nx = -1; ny = 0; nz = dz;
                        return;
                }
            }
            else if(dy == 0){
                switch(dev){
                    case 0:
                        fx = 0; fy = 0; fz = -dz;
                        nx = dx; ny = 0; nz = -dz;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = 0; nz = dz;
                        return;
                    case 2:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = dz;
                        return;
                    case 3:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1;nz = dz;
                        return;
                    case 4:
                        fx = 0; fy = 1; fz = -dz;
                        nx = dx; ny = 1; nz = -dz;
                        return;
                    case 5:
                        fx = -dx; fy = 1; fz = 0;
                        nx = -dx; ny = 1; nz = dz;
                        return;
                    case 6:
                        fx = 0; fy = -1; fz = -dz;
                        nx = dx; ny = -1; nz = -dz;
                        return;
                    case 7:
                        fx = -dx; fy = -1; fz = 0;
                        nx = -dx; ny = -1; nz = dz;
                        return;
                    // Extras
                    case 8:
                        fx = 0; fy = 1; fz = 0;
                        nx = dx; ny = 1; nz = 0;
                        return;
                    case 9:
                        fx = 0; fy = 1; fz = 0;
                        nx = 0; ny = 1; nz = dz;
                        return;
                    case 10:
                        fx = 0; fy = -1; fz = 0;
                        nx = dx; ny = -1; nz = 0;
                        return;
                    case 11:
                        fx = 0; fy = -1; fz = 0;
                        nx = 0; ny = -1; nz = dz;
                        return;
                }
            }
            else{// dz==0
                switch(dev){
                    case 0:
                        fx = 0; fy = -dy; fz = 0;
                        nx = dx; ny = -dy; nz = 0;
                        return;
                    case 1:
                        fx = -dx; fy = 0; fz = 0;
                        nx = -dx; ny = dy; nz = 0;
                        return;
                    case 2:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = dy; nz = 1;
                        return;
                    case 3:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = dy; nz = -1;
                        return;
                    case 4:
                        fx = 0; fy = -dy; fz = 1;
                        nx = dx; ny = -dy; nz = 1;
                        return;
                    case 5:
                        fx = -dx; fy = 0; fz = 1;
                        nx = -dx; ny = dy; nz = 1;
                        return;
                    case 6:
                        fx = 0; fy = -dy; fz = -1;
                        nx = dx; ny = -dy; nz = -1;
                        return;
                    case 7:
                        fx = -dx; fy = 0; fz = -1;
                        nx = -dx; ny = dy; nz = -1;
                        return;
                    // Extras
                    case 8:
                        fx =  0; fy = 0; fz = 1;
                        nx = dx; ny = 0; nz = 1;
                        return;
                    case 9:
                        fx = 0; fy = 0; fz = 1;
                        nx = 0; ny = dy; nz = 1;
                        return;
                    case 10:
                        fx =  0; fy = 0; fz = -1;
                        nx = dx; ny = 0; nz = -1;
                        return;
                    case 11:
                        fx = 0; fy = 0; fz = -1;
                        nx = 0; ny = dy; nz = -1;
                        return;
                }
            }
        case 3:
            switch(dev){
                case 0:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = dz;
                    return;
                case 1:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = dz;
                    return;
                case 2:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = dy; nz = -dz;
                    return;
                // Need to check up to here for forced!
                case 3:
                    fx = 0; fy = -dy; fz = -dz;
                    nx = dx; ny = -dy; nz = -dz;
                    return;
                case 4:
                    fx = -dx; fy = 0; fz = -dz;
                    nx = -dx; ny = dy; nz = -dz;
                    return;
                case 5:
                    fx = -dx; fy = -dy; fz = 0;
                    nx = -dx; ny = -dy; nz = dz;
                    return;
                // Extras
                case 6:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = 0; nz = dz;
                    return;
                case 7:
                    fx = -dx; fy = 0; fz = 0;
                    nx = -dx; ny = dy; nz = 0;
                    return;
                case 8:
                    fx = 0; fy = -dy; fz = 0;
                    nx = 0; ny = -dy; nz = dz;
                    return;
                case 9:
                    fx = 0; fy = -dy; fz = 0;
                    nx = dx; ny = -dy; nz = 0;
                    return;
                case 10:
                    fx = 0; fy = 0; fz = -dz;
                    nx = 0; ny = dy; nz = -dz;
                    return;
                case 11:
                    fx = 0; fy = 0; fz = -dz;
                    nx = dx; ny = 0; nz = -dz;
                    return;
            }
    }
}
