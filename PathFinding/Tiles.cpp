//
//  Tiles.cpp
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#include "Tiles.h"
#include <stdlib.h>
#include <iostream>

static const int COST_ONE = 100;
static const int COST_SQRT_TWO = 141;

std::vector<Environment::Neighbor> Tiles::GetNeighbors(int nodeID) const
{
    std::vector<Environment::Neighbor> neighbors;
    int x, y;
    SplitIDIntoCoord(nodeID, x, y);
    
    static const int DIRECTIONS = 8;
    static const int x_inc[DIRECTIONS] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int y_inc[DIRECTIONS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static const int cost[] = {COST_ONE, COST_SQRT_TWO};
    
    for (int i = 0; i < DIRECTIONS; ++i)
    {
        int neighborX = x + x_inc[i];
        int neighborY = y + y_inc[i];
        
        if (neighborX >= 0 && neighborX < columns && neighborY >= 0 && neighborY < rows)
        {
            int neighborID = MakeIDFromCoord(neighborX, neighborY);
            if (IsTraverseable(neighborID))
            {
                const int costIndex = abs(x_inc[i] * y_inc[i]);
                neighbors.push_back(Environment::Neighbor{neighborID, cost[costIndex]});
            }
        }
    }
    
    return std::move(neighbors);
}

int Tiles::GetHeuristicValue(int from, int to) const
{
    int fromX, fromY;
    SplitIDIntoCoord(from, fromX, fromY);
    
    int toX, toY;
    SplitIDIntoCoord(to, toX, toY);
    
    int diff1 = abs(fromX - toX);
    int diff2 = abs(fromY - toY);
    
    if (diff1 > diff2)
    {
        std::swap(diff1, diff2);
    }
    
    return diff1 * COST_SQRT_TWO + (diff2 - diff1) * COST_ONE;
}

void Tiles::DrawMap(int start, int end) const
{
    for (int nodeID = 0; nodeID < columns * rows; ++nodeID)
    {
        if (nodeID == start)
        {
            std::cout << 'S';
        }
        else if (nodeID == end)
        {
            std::cout << 'E';
        }
        else
        {
            bool isPassable = IsTraverseable(nodeID);
            std::cout << (isPassable ? '.' : 'X');
        }
        
        if ((nodeID + 1) % columns == 0)
        {
            std::cout << std::endl;
        }
    }
}

char Tiles::GetPathSign(int nodeID, int previousNodeID) const
{
    int x, y;
    SplitIDIntoCoord(nodeID, x, y);
    int previousX, previousY;
    SplitIDIntoCoord(previousNodeID, previousX, previousY);
    
    int diffX = previousX - x;
    int diffY = previousY - y;
    if (diffX == 0)
    {
        return '|';
    }
    else if (diffY == 0)
    {
        return '-';
    }
    else if (diffX * diffY > 0)
    {
        return '\\';
    }
    else
    {
        return '/';
    }
    
    return '?';
}

void Tiles::DrawSolution(const std::vector<int>& path) const
{
    std::for_each(path.begin(), path.end(), [](int x){std::cout << x << " ";});
    std::cout << "\n\n\n";
    
    for (int nodeID = 0; nodeID < columns * rows; ++nodeID)
    {
        auto part = std::find(path.begin(), path.end(), nodeID);
        bool isPassable = IsTraverseable(nodeID);
        if (part != path.end())
        {
            if (isPassable == false)
            {
                std::cout << "Error!!!" << *part << " is not passable." << std::endl;
            }
            
            if (part == path.begin())
            {
                std::cout << 'G';
            }
            else if (part == path.end() - 1)
            {
                std::cout << 'S';
            }
            else
            {
                std::cout << GetPathSign(nodeID, *(part - 1));
            }
        }
        else
        {
            std::cout << (isPassable ? '.' : 'X');
        }
        if ((nodeID+1) % columns == 0)
        {
            std::cout << std::endl;
        }
    }
}



