#pragma once
#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include <iostream>

#include "models.hpp"       // 提供全局 ::LinkId
#include "of_controller.hpp"
#include "topo_viewer.hpp"
#include "monitor.hpp"
#include "forecast.hpp"
#include "milp_te.hpp"      // te::MILP_TE / te::GraphCaps / te::Path / te::Flow / te::TE_Output / te::Weights / te::LinkId

class HybridSDNApp {
public:
  struct Paths {
    std::string graph_json;
    std::string flows_csv;
    // 用預設建構子設定預設值，避免 in-class initializer + 預設參數的干擾
    Paths() : graph_json("config/NSFNET.json"),
              flows_csv("config/flows.csv") {}
  };

  // 只有一個建構子簽名；用 const 引用 + 預設參數
  explicit HybridSDNApp(uint16_t of_port, const Paths& paths = Paths());

  // 只在 public 宣告一次
  void run();
  void stop();

private:
  // ----------- Runtime graph model（對應 JSON）-----------
  struct RuntimeGraph {
    std::vector<int> nodes;
    std::set<int>    sdn_nodes;
    std::map<::LinkId,double> cap_mbps;   // 用全局 ::LinkId
    std::map<::LinkId,double> power_cost;
    std::map<::LinkId,bool>   is_sdn;
  };

  // ---------- 小工具：::LinkId <-> te::LinkId ----------
  static ::LinkId  mk_edge_(int a,int b)      { if(a>b) std::swap(a,b); return ::LinkId{a,b}; }
  static te::LinkId to_te(::LinkId id)        { return te::LinkId{id.u,id.v}; }
  static ::LinkId   from_te(te::LinkId id)    { return ::LinkId{id.u,id.v}; }

  static std::string read_all_(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) throw std::runtime_error("Cannot open: " + path);
    std::ostringstream ss; ss << ifs.rdbuf(); return ss.str();
  }

  static double quantile_(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = std::clamp(q, 0.0, 1.0) * (v.size() - 1);
    const size_t i = static_cast<size_t>(idx);
    const double f = idx - i;
    return (i + 1 < v.size()) ? (v[i] * (1.0 - f) + v[i + 1] * f) : v.back();
  }

  static int to_int_(const std::string& s) { return std::stoi(s); }

  RuntimeGraph load_graph_json_(const std::string& path) {
    using json = nlohmann::json;
    auto j = json::parse(read_all_(path));
    RuntimeGraph G;
    for (auto& s : j.at("nodes"))      G.nodes.push_back(to_int_(s.get<std::string>()));
    for (auto& s : j.at("sdn_nodes"))  G.sdn_nodes.insert(to_int_(s.get<std::string>()));
    for (auto& e : j.at("links")) {
      int u = to_int_(e.at("u").get<std::string>());
      int v = to_int_(e.at("v").get<std::string>());
      double cap = e.at("cap").get<double>() * 1000.0; // 假設 JSON 給 Gbps，內部存 Mbps
      auto id = mk_edge_(u, v);
      G.cap_mbps[id]   = cap;
      G.power_cost[id] = cap * 0.1;
      G.is_sdn[id]     = (G.sdn_nodes.count(u) && G.sdn_nodes.count(v));
    }
    return G;
  }

  // 存活鏈路 → MILP 圖能力（key 用 te::LinkId）
  te::GraphCaps make_caps_from_runtime_(const RuntimeGraph& RG,
                                        const std::vector<TopoViewer::Edge>& alive) {
    te::GraphCaps GC;
    for (const auto& e : alive) {
      ::LinkId g = mk_edge_(e.u, e.v);
      auto it = RG.cap_mbps.find(g);
      if (it == RG.cap_mbps.end()) continue;
      te::LinkId k = to_te(g);
      GC.capacity_mbps[k] = it->second;
      GC.power_cost[k]    = RG.power_cost.at(g);
      GC.is_sdn[k]        = RG.is_sdn.at(g);
    }
    return GC;
  }

  // K 條無環 BFS 路徑（te::Path.edges: vector<te::LinkId>）
  static void bfs_k_paths_(const std::map<int,std::vector<int>>& adj, int s, int d, int K,
                           std::vector<te::Path>& out_paths, int& next_pid) {
    struct P { int node; std::vector<int> seq; };
    std::queue<P> q; q.push({s,{s}});
    std::set<std::vector<int>> seen;
    while (!q.empty() && static_cast<int>(out_paths.size()) < K) {
      auto cur = q.front(); q.pop();
      if (static_cast<int>(cur.seq.size()) > 10) continue; // 深度限制
      if (cur.node == d) {
        if (!seen.count(cur.seq)) {
          std::vector<te::LinkId> edges;
          for (size_t i = 1; i < cur.seq.size(); ++i)
            edges.push_back(te::LinkId{std::min(cur.seq[i-1], cur.seq[i]),
                                       std::max(cur.seq[i-1], cur.seq[i])});
          out_paths.push_back(te::Path{next_pid++, std::move(edges)});
          seen.insert(cur.seq);
        }
        continue;
      }
      auto it = adj.find(cur.node);
      if (it == adj.end()) continue;
      for (int nb : it->second) {
        if (std::find(cur.seq.begin(), cur.seq.end(), nb) != cur.seq.end()) continue; // 禁止成環
        auto nxt = cur.seq; nxt.push_back(nb);
        q.push({nb, std::move(nxt)});
      }
    }
  }

  std::vector<te::Path> build_paths_(const std::vector<TopoViewer::Edge>& alive,
                                     const std::vector<te::Flow>& flows, int K) {
    std::map<int, std::vector<int>> adj;
    for (const auto& e : alive) {
      adj[e.u].push_back(e.v);
      adj[e.v].push_back(e.u);
    }
    std::set<std::pair<int,int>> need;
    for (auto f : flows) { int s=f.s, d=f.d; if (s>d) std::swap(s,d); need.insert({s,d}); }

    std::vector<te::Path> paths; int next_pid = 100;
    for (auto [s,d] : need) bfs_k_paths_(adj, s, d, K, paths, next_pid);
    return paths;
  }

  // 路徑列表 → (s,d) → 候選 path id
  static std::map<std::pair<int,int>, std::vector<int>>
  map_paths_to_sd_(const std::vector<te::Path>& paths) {
    std::map<std::pair<int,int>, std::vector<int>> mp;
    for (const auto& p : paths) {
      std::map<int,int> deg;
      for (const auto& e : p.edges) { deg[e.u]++; deg[e.v]++; }
      int s=-1, d=-1;
      for (auto& kv : deg) if (kv.second % 2 == 1) { if (s==-1) s=kv.first; else d=kv.first; }
      if (s==-1 || d==-1) { // 兜底：用邊序列兩端
        if (!p.edges.empty()) { s = p.edges.front().u; d = p.edges.back().v; }
      }
      if (s==-1 || d==-1) continue;
      if (s>d) std::swap(s,d);
      mp[{s,d}].push_back(p.id);
    }
    return mp;
  }

  // 監控需要的容量查詢（輸入 ::LinkId）
  double cap_lookup_(const ::LinkId& e) const {
    auto it = runtime_graph_.cap_mbps.find(e);
    return (it == runtime_graph_.cap_mbps.end()) ? 1000.0 : it->second;
  }

  // 套用 β（plan.beta 的 key 是 te::LinkId）
  void apply_beta_(const te::TE_Output& plan,
                   const std::vector<TopoViewer::Edge>& alive) {
    std::map<te::LinkId, std::pair<int,int>> ports; // (u_port, v_port)
    std::map<te::LinkId, std::pair<int,int>> nodes; // (u, v)
    for (const auto& e : alive) {
      int u = e.u, v = e.v, u_port = e.u_port, v_port = e.v_port;
      if (u > v) { std::swap(u, v); std::swap(u_port, v_port); }
      te::LinkId id{u, v};
      ports[id] = {u_port, v_port};
      nodes[id] = {u, v}; // 假設 swid == node id
    }

    for (const auto& kv : plan.beta) {
      const te::LinkId id = kv.first;
      const int beta = kv.second;
      auto pit = ports.find(id);
      auto sit = nodes.find(id);
      if (pit == ports.end() || sit == nodes.end()) continue;

      const int u = sit->second.first;
      const int v = sit->second.second;
      const int u_port = pit->second.first;
      const int v_port = pit->second.second;

      const bool up = (beta == 1);
      const int speed = up ? 10000 : 0;   // 例：開→10G，關→admin-down
      ctl_.port_mod(u, u_port, up, speed);
      ctl_.port_mod(v, v_port, up, speed);
    }
  }

private:
  uint16_t of_port_{6633};
  Paths paths_{};

  bool running_{true};

  // 模組
  OFController ctl_;
  TopoViewer   topo_;
  Monitor      mon_;
  std::unique_ptr<Forecast> forecast_;

  // 运行態
  RuntimeGraph runtime_graph_;
  std::vector<te::Flow> flows_;
  std::map<::LinkId, std::vector<double>> hist_mbps_;

  // 僅宣告，定義放在 .cpp
  static std::vector<te::Flow> load_flows_csv_or_default_(const std::string& path,
                                                          const std::vector<int>& nodes);
};

