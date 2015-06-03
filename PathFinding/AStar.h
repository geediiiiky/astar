//
//  AStar.h
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#pragma once

#include <set>
#include <unordered_map>
#include <vector>
#include <unordered_set>

class Environment;


class AStar
{
private:
    struct AStarNode
    {
        int ID;
        int parentID;
        int g;
        int h;
        int f;
        
        bool operator<(const AStarNode& other) const
        {
            // TODO: this is not deterministic enough, it would be relying on the implementation of stl container
            if (f != other.f)
            {
                return f < other.f;
            }
            
            if (g != other.g)
            {
                return g < other.g;
            }
            
            return ID < other.ID;
        }
    };
    
    struct OpenList
    {
        std::set<AStarNode> nodes;
        std::unordered_map<int, std::set<AStarNode>::iterator> id2Nodes;
        
        void Clear();
        void Insert(const AStarNode& node);
        bool PopNodeWithLowestF(AStarNode& outNode);
    };
    
    struct CloseList
    {
        std::unordered_map<int, AStarNode> nodes;
        
        void Clear();
        void Add(const AStarNode& node);
        bool Contains(int nodeID);
    };
    
    OpenList openList;
    CloseList closeList;
    
    AStarNode result;
public:
    bool findPath(const Environment& env, int start, int goal);
    std::vector<int> getResultPath() const;
};


class Environment
{
private:
    int rows;
    int columns;
    std::unordered_set<int> untraverseable;
public:
    Environment(int r, int c) : rows(r), columns(c){}

    // actually needed
    struct Neighbor
    {
        int ID;
        int cost;
    };
    std::vector<Neighbor> GetNeighbors(int nodeID) const;
    int GetHeuristicValue(int from, int to) const;
    bool IsTraverseable(int nodeID) const {return untraverseable.count(nodeID) == 0;}
    // end
    
    void SetUntraverseable(int nodeID) {untraverseable.insert(nodeID);}
    
    void DrawMap() const;
    void DrawSolution(const std::vector<int>& path) const;
};
