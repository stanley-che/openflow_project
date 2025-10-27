#pragma once
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <string>

// 放在 namespace 以避免和其他檔案的同名型別衝突
namespace te {

// ------------ 基本型別（可依你專案改到自己的 models.hpp） ------------
struct LinkId {
  int u{-1}, v{-1};
  bool operator<(const LinkId& o) const { return std::tie(u,v) < std::tie(o.u,o.v); }
  bool operator==(const LinkId& o) const { return u==o.u && v==o.v; }
};

struct Path {
  int id{0};                    // 唯一路徑 ID
  std::vector<LinkId> edges;    // 這條 path 上的 link 列表（無向）
};

struct Flow {
  int id{0};
  int s{0}, d{0};
  double demand_mbps{0.0};      // 流量需求
  std::vector<int> cand_path_ids; // 候選路徑 id
};

struct GraphCaps {
  // link 屬性
  std::map<LinkId, double> capacity_mbps;  // C_e
  std::map<LinkId, bool>   is_sdn;         // SDN link 才有 β_e 決策；legacy 固定 1
  std::map<LinkId, double> power_cost;     // P_e（能耗權重；可自定）

  double cap(const LinkId& e) const {
    auto it = capacity_mbps.find(e);
    return it==capacity_mbps.end()? 0.0 : it->second;
  }
  bool sdn(const LinkId& e) const {
    auto it = is_sdn.find(e);
    return it!=is_sdn.end() && it->second;
  }
  double power(const LinkId& e) const {
    auto it = power_cost.find(e);
    if (it != power_cost.end()) return it->second;
    auto c = cap(e); return (c>0.0)? c*0.1 : 1.0;
  }
};

struct Weights { double ewr{0.5}; double lwr{0.5}; };

struct TE_Output {
  std::map<int /*flow_id*/, int /*chosen_path_id*/> chosen_path;
  std::map<LinkId, int /*0/1*/> beta;      // link 開關（legacy 預設 1）
  std::map<LinkId, double> load_mbps;      // 各 link 的負載
  double objective{0.0};
  bool optimal{false};
  std::string status_text;
};

// ---------------- MILP 類別介面 ----------------
class MILP_TE {
public:
  MILP_TE(const GraphCaps& g,
          const std::vector<Path>& paths,
          const std::vector<Flow>& flows);

  // 求解；time_limit_sec=0 表示不限時
  bool solve(const Weights& w, TE_Output* out, double time_limit_sec = 0.0);

private:
  // incidence key：這個 (flow, path) 是否使用到 link e
  struct FPE {
    int f, p; LinkId e;
    bool operator<(const FPE& o) const {
      if (f!=o.f) return f<o.f;
      if (p!=o.p) return p<o.p;
      return e<o.e;
    }
  };
  struct XP {
    int f, p;
    bool operator<(const XP& o) const { return std::tie(f,p) < std::tie(o.f,o.p); }
  };

  void build_fp_incidence_();
  void build_variable_index_();

private:
  const GraphCaps G_;
  std::map<int, Path> P_;
  std::map<int, Flow> F_;
  std::vector<LinkId> links_;

  std::set<FPE> fp_use_;             // (f,p,e) 使用關係

  // 欄位索引
  std::vector<XP> x_index_;
  std::vector<LinkId> be_index_;
  std::map<XP, int> x_col_;          // x_{f,p} -> col
  std::map<LinkId, int> be_col_;     // β_e -> col
};

} // namespace te

