#ifndef NBV_SIMULATION_MCMF_H
#define NBV_SIMULATION_MCMF_H

#pragma once

#include <vector>
#include <cmath>
#include <queue>

class MCMF {

public:
    struct Edge
    {
        int from, to, cap, flow;
        double cost;
        Edge(int u, int v, int c, int f, double w)
                : from(u)
                , to(v)
                , cap(c)
                , flow(f)
                , cost(w)
        {}
    };

    const int INF = 0x3f3f3f3f;
    int n, m;
    std::vector<Edge> edges;
    std::vector<std::vector<int>> G;
    std::vector<int> inq, p, a;
    std::vector<double> d;
    double eps = 1e-3;
    [[nodiscard]] bool isZero(const double& val) const;
    void init(int _n);
    void AddEdge(int from, int to, int cap, double cost);
    bool BellmanFord(int s, int t, int& flow, double& cost);
    std::vector<int> work(const std::vector<std::vector<std::pair<int, double>>>& vec);

};


#endif //NBV_SIMULATION_MCMF_H
