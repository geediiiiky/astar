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

class Environment;


class AStar
{
private:
    struct AStarNode
    {
        static const int NODE_NONE;
        
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
        
        AStarNode(int ID = NODE_NONE, int parentID = NODE_NONE, int g = 0, int h = 0) : ID{ID}, parentID{parentID}, g{g}, h{h}, f{g+h}
        {
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


