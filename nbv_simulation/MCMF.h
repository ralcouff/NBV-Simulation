#ifndef NBV_SIMULATION_MCMF_H
#define NBV_SIMULATION_MCMF_H

#pragma once

#include <cmath>
#include <queue>
#include <vector>

class MCMF {

public:
    struct Edge {
        int from, to, cap, flow;
        double cost;

        Edge(int u, int v, int c, int f, double w)
                : from(u), to(v), cap(c), flow(f), cost(w) {}
    };

    const int INF = 0x3f3f3f3f;
    int n, m;
    std::vector<Edge> edges;
    std::vector<std::vector<int>> G;
    std::vector<int> inq, p, a;
    std::vector<double> d;
    double eps = 1e-3;

    /**
     * Test if a value is Zero or not
     * @param val The value to test
     * @return TRUE if val < EPS, else FALSE
     */
    [[nodiscard]] bool isZero(const double &val) const;

    /**
     * Initialize the MF solver
     * @param _n The number of element in the Graph
     */
    void init(int _n);

    void AddEdge(int from, int to, int cap, double cost);

    bool BellmanFord(int s, int t, int &flow, double &cost);

    /**
     * Solve the MCMF
     * @param vec The bipartite list
     * @return A list of views satisfying the set covering problem by MF.
     */
    std::vector<int> work(const std::vector<std::vector<std::pair<int, double>>> &vec);

};


#endif //NBV_SIMULATION_MCMF_H
