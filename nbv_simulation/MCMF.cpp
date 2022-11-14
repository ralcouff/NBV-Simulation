#include "MCMF.h"

bool MCMF::isZero(const double &val) const {
    return std::abs(val) < eps;
}

void MCMF::init(int _n) {
    n = _n;
    G.resize(_n);
    d.resize(_n);
    a.resize(_n);
    p.resize(_n);
    inq.resize(_n);

    for (int i = 0; i < _n; i++) {
        G[i].clear();
        d[i] = a[i] = p[i] = inq[i] = 0;
    }
    edges.clear();
}

void MCMF::AddEdge(int from, int to, int cap, double cost) {
    edges.emplace_back(from, to, cap, 0, cost);
    edges.emplace_back(to, from, 0, 0, -cost);
    m = (int) edges.size();
    G[from].push_back(m - 2);
    G[to].push_back(m - 1);
}

bool MCMF::BellmanFord(int s, int t, int &flow, double &cost) {
    for (int i = 0; i < n; i++)
        d[i] = INF;
    for (int i = 0; i < n; i++)
        inq[i] = 0;
    d[s] = 0;
    inq[s] = 1;
    p[s] = 0;
    a[s] = INF;
    std::queue<int> Q;
    Q.push(s);
    while (!Q.empty()) {
        int u = Q.front();
        Q.pop();
        inq[u] = 0;
        for (int i = 0; i < G[u].size(); i++) {
            Edge &e = edges[G[u][i]];
            if (e.cap > e.flow && d[e.to] > d[u] + e.cost) {
                d[e.to] = d[u] + e.cost;
                p[e.to] = G[u][i];
                a[e.to] = std::min(a[u], e.cap - e.flow);
                if (!inq[e.to]) {
                    Q.push(e.to);
                    inq[e.to] = 1;
                }
            }
        }
    }
    // Exit when there are no more roads to expand
    if (d[t] == INF)
        return false;
    flow += a[t];
    cost += d[t] * a[t];
    for (int u = t; u != s; u = edges[p[u]].from) {
        edges[p[u]].flow += a[t];
        edges[p[u] ^ 1].flow -= a[t];
    }
    return true;
}

std::vector<int> MCMF::work(const std::vector<std::vector<std::pair<int, double>>> &vec) {
    int nn = (int) vec.size();
    int S = nn, T = nn + 1;
    init(T + 2);

    std::vector<bool> vis(nn);
    for (int u = 0; u < nn; u++) {
        for (auto &e: vec[u]) {
            int v = e.first;
            double w = e.second;
            if (!isZero(w)) {
                if (!vis[u]) {
                    AddEdge(S, u, 1, 0);
                    vis[u] = true;
                }
            } else {
                if (!vis[v]) {
                    AddEdge(v, T, INF, 0);
                    vis[v] = true;
                }
            }
            AddEdge(u, v, INF, -w);
        }
    }
    int flow = 0;
    double cost = 0;
    while (BellmanFord(S, T, flow, cost));
    std::vector<int> ret;
    for (auto &e: edges)
        if (e.to == T)
            if (e.flow > 0) {
                ret.push_back(e.from);
            }
    return ret;
}
