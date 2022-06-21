//
// Created by ZwYu on 2022-06-20.
//
#include "SPFA.h"

SPFA::SPFA(int n)
{
    this->n = n;
    graph = new double *[n];
    *graph = new double [n*n];
    for (int i = 0; i < n; ++i)
        *(graph+i) = *graph + i*n;
    memset(*graph, inf, sizeof(double)*n*n);
    dis = new double [n];
    visited = new bool [n];
    pre = new int [n];
    memset(*graph, inf, sizeof(double)*n*n);
    memset(dis, inf, sizeof(double)*n);
    memset(visited, false, sizeof(bool)*n);
    memset(pre, -1, sizeof(int)*n);
//    std::cout<<"I am constructor!"<<std::endl;
}

SPFA::~SPFA()
{
    delete [] graph[0];
    delete [] graph;
    delete [] dis;
    delete [] visited;
    delete [] pre;
//    std::cout<<"I am destructor!"<<std::endl;
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
