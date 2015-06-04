//
//  AStar.cpp
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#include "AStar.h"
#include "Environment.h"
#include <iostream>

const int AStar::AStarNode::NODE_NONE = -1;

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
    result.ID = AStarNode::NODE_NONE;
    result.parentID = AStarNode::NODE_NONE;
    openList.Clear();
    closeList.Clear();
    
    int startH = env.GetHeuristicValue(start, goal);
    AStarNode startNode(start, AStarNode::NODE_NONE, 0, startH);
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
            AStarNode newNode(neighbor.ID, currentNode.ID, newG, newH);
            
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
    while (node.parentID != AStarNode::NODE_NONE)
    {
        resultPath.push_back(node.ID);
        node = closeList.nodes.at(node.parentID);
    }
    
    // the start node
    resultPath.push_back(node.ID);
    return std::move(resultPath);
}
