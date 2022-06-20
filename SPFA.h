//
// Created by ZwYu on 2022-06-20.
//

#ifndef EV_PROJECT_SPFA_H
#define EV_PROJECT_SPFA_H

#define maxn 1000
#define inf 0x42

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <string>
#include <stack>
#include <queue>
#include <cstring>
#include <map>
#include <iterator>
#include <list>
#include <set>
#include <functional>
#include <memory.h>

class SPFA
{
public:
    int n, m, s, t;
    double graph[maxn][maxn];
    double dis[maxn];
    int pre[maxn];
    bool visited[maxn];
    SPFA(){};
    SPFA(int n, int m);
    void InputAdjMat(int u, int v, double weight);
    void SetSourceAndTank(int s, int t);
    void Getpath();
    double Getdistance();
};

#endif //EV_PROJECT_SPFA_H
