//
//  AStar.cpp
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#include "AStar.h"
#include <iostream>

static const int NODE_NONE = -1;

//=========== open list ==========
void AStar::OpenList::Clear()
{
    nodes.clear();
    id2Nodes.clear();
}

void AStar::OpenList::Insert(const AStar::AStarNode &node)
{
    const int nodeID = node.ID;
    auto itr = id2Nodes.find(nodeID);
    if (itr != id2Nodes.end())
    {
        auto found = itr->second;
        if (found->f <= node.f)
        {
            return;
        }
        nodes.erase(found);
    }

    auto result = nodes.insert(node);
    id2Nodes[node.ID] = result.first;
}

bool AStar::OpenList::PopNodeWithLowestF(AStar::AStarNode& outNode)
{
    if (nodes.empty())
    {
        return false;
    }
    outNode = *(nodes.begin());
    nodes.erase(nodes.begin());
    id2Nodes.erase(outNode.ID);
    return true;
}

//================================

//=========== close list ==========
void AStar::CloseList::Clear()
{
    nodes.clear();
}

void AStar::CloseList::Add(const AStar::AStarNode &node)
{
    nodes[node.ID] = node;
}

bool AStar::CloseList::Contains(int nodeID)
{
    return nodes.count(nodeID) != 0;
}

//================================

bool AStar::findPath(const Environment &env, int start, int goal)
{
    result.ID = NODE_NONE;
    result.parentID = NODE_NONE;
    openList.Clear();
    closeList.Clear();
    
    int startH = env.GetHeuristicValue(start, goal);
    AStarNode startNode{start, NODE_NONE, 0, startH, startH};
    openList.Insert(startNode);
    
    AStarNode currentNode;
    while (openList.PopNodeWithLowestF(currentNode))
    {
        if (currentNode.ID == goal)
        {
            result = currentNode;
            break;
        }
        closeList.Add(currentNode);
        
        // for each passable neighbor of the current node
        for (const auto& neighbor : env.GetNeighbors(currentNode.ID))
        {
            if (closeList.Contains(neighbor.ID))
            {
                continue;
            }
            
            int newG = currentNode.g + neighbor.cost;
            int newH = env.GetHeuristicValue(neighbor.ID, goal);
            AStarNode newNode{neighbor.ID, currentNode.ID, newG, newH, newG + newH};
            
            openList.Insert(newNode);
            
        }
        
//        std::cout << "[" << currentNode.ID << "] ";
//        std::for_each(openList.nodes.begin(), openList.nodes.end(), [](const AStarNode& node){std::cout << node.ID << "(g:" << node.g << " h:" << node.h << " f:" << node.f << ") ";});
//        std::cout << std::endl;

    }
    
    return true;
}

std::vector<int> AStar::getResultPath() const
{
    std::vector<int> resultPath;
    AStarNode node = result;
    while (node.parentID != NODE_NONE)
    {
        resultPath.push_back(node.ID);
        node = closeList.nodes.at(node.parentID);
    }
    
    // the start node
    resultPath.push_back(node.ID);
    return std::move(resultPath);
}


static const int cost[] = {100, 141};
std::vector<Environment::Neighbor> Environment::GetNeighbors(int nodeID) const
{
    std::vector<Environment::Neighbor> neighbors;
    int x = nodeID % columns;
    int y = nodeID / columns;
    static const int x_inc[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int y_inc[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    for (int i = 0; i < 8; ++i)
    {
        int neighborX = x + x_inc[i];
        int neighborY = y + y_inc[i];
        
        if (neighborX >=0 && neighborX < columns && neighborY >=0 && neighborY < rows && IsTraverseable(neighborX + neighborY * columns))
        {
            neighbors.push_back(Environment::Neighbor{neighborX + neighborY * columns, cost[abs(x_inc[i]) + abs(y_inc[i]) - 1]});
        }
    }
    
    return std::move(neighbors);
}

int Environment::GetHeuristicValue(int from, int to) const
{
    int fromX = from % columns;
    int fromY = from / columns;
    
    int toX = to % columns;
    int toY = to / columns;
    
    int diff1 = abs(fromX - toX);
    int diff2 = abs(fromY - toY);

    if (diff1 > diff2)
    {
        std::swap(diff1, diff2);
    }
    
    return diff1 * cost[1] + (diff2 - diff1) * cost[0];
}

void Environment::DrawMap() const
{
    for (int nodeID = 0; nodeID < columns * rows; ++nodeID)
    {
        bool isPassable = IsTraverseable(nodeID);
        std::cout << (isPassable ? 'O' : 'X');
        if ((nodeID + 1) % columns == 0)
        {
            std::cout << std::endl;
        }
    }
}

void Environment::DrawSolution(const std::vector<int>& path) const
{
    std::for_each(path.begin(), path.end(), [](int x){std::cout << x << " ";});
    std::cout << "\n\n\n";
    
    for (int nodeID = 0; nodeID < columns * rows; ++nodeID)
    {
        auto part = std::find(path.begin(), path.end(), nodeID);
        if (part != path.end())
        {
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
                std::cout << '~';
            }
        }
        else
        {
            bool isPassable = IsTraverseable(nodeID);
            std::cout << (isPassable ? 'O' : 'X');
        }
        if ((nodeID+1) % columns == 0)
        {
            std::cout << std::endl;
        }
    }
}
