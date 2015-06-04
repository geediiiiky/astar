//
//  Environment.h
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#pragma once
#include <vector>

class Environment
{
public:
    struct Neighbor
    {
        int ID;
        int cost;
    };
    virtual std::vector<Neighbor> GetNeighbors(int nodeID) const = 0;
    virtual int GetHeuristicValue(int from, int to) const = 0;
};
