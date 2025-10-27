// src/models.hpp
#pragma once
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

struct LinkId {
  int u{-1}, v{-1};
  bool operator<(const LinkId& o) const { return std::tie(u,v) < std::tie(o.u,o.v); }
  bool operator==(const LinkId& o) const { return u==o.u && v==o.v; }
};

struct Path {
  int id{0};
  std::vector<LinkId> edges;
};

struct Flow {
  int id{0};
  int s{0}, d{0};
  double demand_mbps{0.0};
  std::vector<int> cand_path_ids;
};

struct GraphCaps {
  std::map<LinkId, double> capacity_mbps;  // C_e
  std::map<LinkId, bool>   is_sdn;         // link under SDN control?
  std::map<LinkId, double> power_cost;     // P_e
  double cap(const LinkId& e) const {
    auto it = capacity_mbps.find(e); return it==capacity_mbps.end()? 0.0 : it->second;
  }
  bool sdn(const LinkId& e) const {
    auto it = is_sdn.find(e); return it!=is_sdn.end() && it->second;
  }
  double power(const LinkId& e) const {
    auto it = power_cost.find(e);
    if (it != power_cost.end()) return it->second;
    auto c = cap(e); return (c>0.0)? c*0.1 : 1.0;
  }
};

struct Weights { double ewr{0.5}; double lwr{0.5}; };

struct TE_Output {
  std::map<int, int>            chosen_path;
  std::map<LinkId, int>         beta;
  std::map<LinkId, double>      load_mbps;
  double                        objective{0.0};
  bool                          optimal{false};
  std::string                   status_text;
};

