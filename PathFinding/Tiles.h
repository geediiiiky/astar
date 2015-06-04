//
//  Tiles.h
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#pragma once

#include "Environment.h"
#include <unordered_set>

class Tiles : public Environment
{
private:
    int rows;
    int columns;
    std::unordered_set<int> untraverseable;

public:
    Tiles(int r, int c) : rows(r), columns(c){}

    // implemenation
    virtual std::vector<Neighbor> GetNeighbors(int nodeID) const override;
    virtual int GetHeuristicValue(int from, int to) const override;
    //
    
    void SetUntraverseable(int nodeID) {untraverseable.insert(nodeID);}
    
    // just for test
    void DrawMap(int start, int end) const;
    void DrawSolution(const std::vector<int>& path) const;
    // end
    
private:
    // just for test
    char GetPathSign(int nodeID, int previousNodeID) const;
    // end
    
    bool IsTraverseable(int nodeID) const {return untraverseable.count(nodeID) == 0;}
    
    int MakeIDFromCoord(int x, int y) const { return x + y * columns;}
    void SplitIDIntoCoord(int nodeID, int&x, int& y) const { x = nodeID % columns; y = nodeID / columns; }
};