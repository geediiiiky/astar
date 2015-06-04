//
//  main.cpp
//  PathFinding
//
//  Created by Shihai Wang on 6/3/15.
//  Copyright (c) 2015 Shihai Wang. All rights reserved.
//

#include "AStar.h"
#include "Tiles.h"
#include <iostream>
#include <random>
#include <vector>

using namespace std;

int main(int argc, const char * argv[])
{
    AStar astar;
    int rows = 40, columns = 50;
    Tiles env(rows, columns);
    random_device generator;
    uniform_int_distribution<int> distribution(1,4);
    
    vector<int> impassableNodes;
    for (int i = 0; i < columns * rows; i++)
    {
        int dice_roll = distribution(generator);
        if (dice_roll == 3)
        {
            impassableNodes.push_back(i);
        }
    }
    
    cout << "blocks(" << impassableNodes.size() << "): ";
    for (auto i : impassableNodes)
    {
        env.SetUntraverseable(i);
        cout << i << ", ";
    }
    cout << endl;

    
    std::uniform_int_distribution<int> startEndDistribution(0,rows * columns-1);
    int start, end;
    while (true) {
        start = startEndDistribution(generator);
        if (std::find(impassableNodes.begin(), impassableNodes.end(), start) == impassableNodes.end())
        {
            break;
        }
    }
    while (true) {
        end = startEndDistribution(generator);
        if (std::find(impassableNodes.begin(), impassableNodes.end(), end) == impassableNodes.end())
        {
            break;
        }
    }
    cout << "start = " << start << ", end  = " << end << ";" << endl;

    env.DrawMap(start, end);
    
    std::cout<<"-----------\n\n\n\n";
    
    astar.findPath(env, start, end);
//    astar.findPath(env, 11, 18);
    env.DrawSolution(astar.getResultPath());
    return 0;
}
