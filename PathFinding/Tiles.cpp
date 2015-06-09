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
#include <stdio.h>

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

bool Tiles::IsThrough(int start, int end) const
{
    if (start == end)
    {
        return true;
    }
    
    // construct a line from center of the start tile to the center of the end tile, each tile edge = 2 units
    // therefore the center of tile(0, 0) is (1, 1), tile(3, 4) is (7, 9), tile(x, y) has center (2x+1, 2y+1)
    // the line can be represented by the following formular
    //      (x2 - x1)y = (y2 - y1)x + ((x2 - x1)y1 - (y2 - y1)x1)
    // where x ranges between [x1, x2], y ranges between [y1, y2]， and coordinate(x1, x2) and (y1, y2) are the start and end point of the line
    // this is not in the form of y = kx + b is because of the possibility of x2 == x1
    int startX, startY, endX, endY;
    SplitIDIntoCoord(start, startX, startY);
    SplitIDIntoCoord(end, endX, endY);
    
    // now check if the slope is greater or less than 45 degrees
    // if it's less than 45 degrees, we check every intersect against y = c, where c is even and ranges between (x1, x2)
    // otherwise we check every intersect against x = c, where c is even and ranges between (y1, y2)
    // for each intersection, we test its two neighbor tiles are obstacles or not, (left and right if the slope is less than 45 degrees, up and down otherwise)
    // if and only if the slope is exactly 45 degrees, we just need to test one tile
    bool is45Degree = abs(startX - endX) == abs(startY - endY);
    bool alongX = abs(startX - endX) >= abs(startY - endY);
    if ((alongX && startX > endX) || (!alongX && startY > endY))
    {
        std::swap(startX, endX);
        std::swap(startY, endY);
    }
    
    const int x1 = startX * 2 + 1;
    const int y1 = startY * 2 + 1;
    const int x2 = endX * 2 + 1;
    const int y2 = endY * 2 + 1;
    
    const int offsetX = x2 - x1;
    const int offsetY = y2 - y1;
    const int b = (offsetX * y1 - offsetY * x1);
    
    // now we have all the ingredients, the equation is now
    //      offsetX * y = offsetY * x + b
    // Note: offsetX or offsetY is guaranteed to be greater than 0, if it's the denominator
    // Note2: after the division the value is floored (unless the slope is 45 degrees), it is what we expect and that why we need to test the 2 neighbor tiles unless the slope of the line is 45 degrees.
    bool isThrough = true;
    int nodeToCheck1 = 0, nodeToCheck2 = 0;
    if (alongX)
    {
        int checkX = x1 + 1;
        while (checkX < x2)
        {
            int checkY = (offsetY * checkX + b) / offsetX;
            int tileY = checkY / 2;
            int tileXRight = checkX / 2;
            nodeToCheck1 = MakeIDFromCoord(tileXRight, tileY);
            
            if (!is45Degree)
            {
                int tileXLeft = checkX / 2 - 1;
                nodeToCheck2 = MakeIDFromCoord(tileXLeft, tileY);
            }
            checkX += 2;
            
            if (!IsTraverseable(nodeToCheck1) || (!is45Degree && !IsTraverseable(nodeToCheck2)))
            {
                isThrough = false;
                break;
            }
        }
    }
    else
    {
        int checkY = y1 + 1;
        while (checkY < y2)
        {
            int checkX = (offsetX * checkY - b) / offsetY;
            int tileX = checkX / 2;
            int tileYUp = checkY / 2;
            nodeToCheck1 = MakeIDFromCoord(tileX, tileYUp);
            if (!is45Degree)
            {
                int tileYDown = checkY / 2 - 1;
                nodeToCheck2 = MakeIDFromCoord(tileX, tileYDown);
            }
            checkY += 2;
            
            if (!IsTraverseable(nodeToCheck1) || (!is45Degree && !IsTraverseable(nodeToCheck2)))
            {
                isThrough = false;
                break;
            }
        }
    }
    
    return isThrough;
}


std::vector<int> Tiles::GetSmoothedPath(std::vector<int> paths) const
{
    if (paths.size() > 3)
    {
        std::vector<int> smoothedPath;
        smoothedPath.push_back(paths[0]);
        for (int i = 0; i < paths.size() - 2; )
        {
            int nextIndex = i + 1;
            for (int j = i + 2; j < paths.size(); ++j)
            {
                nextIndex = j - 1;
                if (!IsThrough(paths[i], paths[j]))
                {
                    std::cout << "unable to pass from " << paths[i] << " to " << paths[j] << std::endl;
                    smoothedPath.push_back(paths[j - 1]);
                    break;
                }
            }
            
            i = nextIndex;
        }
        smoothedPath.push_back(paths[paths.size() - 1]);
        return std::move(smoothedPath);
    }
    
    return paths;
}
