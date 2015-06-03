//
//  main.cpp
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#include "AStar.h"
#include <iostream>

int main(int argc, const char * argv[])
{
    AStar astar;
    Environment env(10, 10);
    for (int i = 15; i <= 85; i += 10)
    {
        env.SetUntraverseable(i);
    }
    
    env.DrawMap();
    
    std::cout<<"-----------\n\n\n\n";
    
    astar.findPath(env, 31, 69);
//    astar.findPath(env, 11, 18);
    env.DrawSolution(astar.getResultPath());
    return 0;
}
