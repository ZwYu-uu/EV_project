//
// Created by ZwYu on 2022-06-20.
//
#include "SPFA.h"

SPFA::SPFA(int n, int m)
{
    this->n = n;
    this->m = m;
    memset(graph, inf, sizeof(graph));
    memset(dis, inf, sizeof(dis));
    memset(visited, false, sizeof(visited));
    memset(pre, -1, sizeof(pre));
}

void SPFA::InputAdjMat(int u, int v, double weight)
{
    this->graph[u][v] = weight;
}

void SPFA::SetSourceAndTank(int s, int t)
{
    this->s = s;
    this->t = t;
}

void SPFA::Getpath()
{
    std::queue<int> q;
    dis[this->s] = 0;
    q.push(this->s);
    int temp;
    this->visited[this->s] = true;
    while(!q.empty())
    {
        temp = q.front();
        q.pop();
        this->visited[temp] = false;
        for(int i = 0; i < n; i++)
        {
            if(this->dis[i] > this->dis[temp] + this->graph[temp][i])
            {
                this->dis[i] = this->dis[temp] + this->graph[temp][i];
                this->pre[i] = temp;
                if(!this->visited[i])
                {
                    q.push(i);
                    this->visited[i] = true;
                }
            }
        }
    }
}

double SPFA::Getdistance()
{
    return dis[this->t];
}

//SPFA test(7, 9);
//test.InputAdjMat(0, 1, -1);
//test.InputAdjMat(0, 2, -2);
//test.InputAdjMat(1, 3, -3);
//test.InputAdjMat(1, 4, -4);
//test.InputAdjMat(2, 4, -1);
//test.InputAdjMat(2, 5, -2);
//test.InputAdjMat(3, 6, -3);
//test.InputAdjMat(4, 6, -4);
//test.InputAdjMat(5, 6, -1);
//test.SetSourceAndTank(0, 6);
//test.Getpath();
