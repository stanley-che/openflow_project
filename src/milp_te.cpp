#include "milp_te.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <coin/OsiClpSolverInterface.hpp>
#include <coin/CbcModel.hpp>
#include <coin/CoinPackedMatrix.hpp>
#include <coin/CoinPackedVector.hpp>

namespace te {

MILP_TE::MILP_TE(const GraphCaps& g,
                 const std::vector<Path>& paths,
                 const std::vector<Flow>& flows)
  : G_(g)
{
  for (const auto& p : paths) P_[p.id] = p;
  for (const auto& f : flows) F_[f.id] = f;
  for (const auto& kv : G_.capacity_mbps) links_.push_back(kv.first);
  build_fp_incidence_();
}

bool MILP_TE::solve(const Weights& w, TE_Output* out, double time_limit_sec) {
  // 欄位：先 x_{f,p} 再 β_e
  build_variable_index_();
  const int ncols = int(x_index_.size() + be_index_.size());

  // 目標係數
  std::vector<double> obj(ncols, 0.0);

  // x：Σ_e (Df/Ce)*x_{f,p}
  for (const auto& fk : F_) {
    const auto& f = fk.second;
    const double Df = std::max(0.0, f.demand_mbps);
    for (int pid : f.cand_path_ids) {
      double coef = 0.0;
      for (const auto& e : P_.at(pid).edges) {
        const double Ce = std::max(1e-9, G_.cap(e));
        coef += (Df / Ce);
      }
      obj[x_col_.at({f.id, pid})] = w.lwr * coef;
    }
  }
  // β：Σ_e P_e*β_e (僅 SDN link)
  for (const auto& e : links_) {
    if (!G_.sdn(e)) continue;
    obj[be_col_.at(e)] = w.ewr * std::max(0.0, G_.power(e));
  }

  // 以 row-major 組合矩陣
  CoinPackedMatrix mat(false, 0, 0);
  std::vector<double> rowLower, rowUpper;

  // 1) 每個 flow 恰選一條 path：sum_p x_{f,p} = 1
  for (const auto& fk : F_) {
    const auto& f = fk.second;
    CoinPackedVector row;
    for (int pid : f.cand_path_ids) row.insert(x_col_.at({f.id, pid}), 1.0);
    mat.appendRow(row);
    rowLower.push_back(1.0);
    rowUpper.push_back(1.0);
  }

  // 2) Link capacity
  // SDN: Σ_f Σ_{p∋e} Df*x_{f,p} - Ce*β_e ≤ 0
  // Legacy: Σ_f Σ_{p∋e} Df*x_{f,p} ≤ Ce
  for (const auto& e : links_) {
    CoinPackedVector row;
    const double Ce = G_.cap(e);

    for (const auto& fk : F_) {
      const auto& f = fk.second;
      const double Df = std::max(0.0, f.demand_mbps);
      for (int pid : f.cand_path_ids) {
        if (fp_use_.count({f.id, pid, e})) row.insert(x_col_.at({f.id, pid}), Df);
      }
    }

    if (G_.sdn(e)) {
      row.insert(be_col_.at(e), -Ce);
      mat.appendRow(row);
      rowLower.push_back(-COIN_DBL_MAX);
      rowUpper.push_back(0.0);
    } else {
      mat.appendRow(row);
      rowLower.push_back(-COIN_DBL_MAX);
      rowUpper.push_back(Ce);
    }
  }

  // 載入到求解器
  OsiClpSolverInterface si;
  std::vector<double> colLower(ncols, 0.0), colUpper(ncols, 1.0);
  si.setObjSense(1.0); // minimize
  si.loadProblem(mat, colLower.data(), colUpper.data(),
                 obj.data(), rowLower.data(), rowUpper.data());

  // 整數變數（x 與 β）
  std::vector<int> intIdx; intIdx.reserve(x_col_.size()+be_col_.size());
  for (const auto& kv : x_col_)  intIdx.push_back(kv.second);
  for (const auto& kv : be_col_) intIdx.push_back(kv.second);
  if (!intIdx.empty()) si.setInteger(intIdx.data(), (int)intIdx.size());

  // CBC
  CbcModel model(si);
  if (time_limit_sec > 0.0) model.setMaximumSeconds(time_limit_sec);
  model.setLogLevel(1);
  model.setIntegerTolerance(1e-6);
  model.branchAndBound();

  out->optimal = (model.status()==0) || model.isProvenOptimal();
  out->objective = model.getObjValue();
  out->status_text = out->optimal ? "optimal"
                     : (model.isProvenInfeasible() ? "infeasible" : "feasible");

  const double* sol = model.bestSolution();
  if (!sol) return false;

  // β 決策
  out->beta.clear();
  for (const auto& e : links_) {
    if (G_.sdn(e)) out->beta[e] = (sol[be_col_.at(e)] >= 0.5) ? 1 : 0;
    else           out->beta[e] = 1;
  }

  // 每個 flow 選到的 path
  out->chosen_path.clear();
  for (const auto& fk : F_) {
    const auto& f = fk.second;
    int best_pid = -1; double best = -1.0;
    for (int pid : f.cand_path_ids) {
      double val = sol[x_col_.at({f.id, pid})];
      if (val > best) { best = val; best_pid = pid; }
    }
    out->chosen_path[f.id] = best_pid;
  }

  // link 負載
  out->load_mbps.clear();
  for (const auto& e : links_) out->load_mbps[e] = 0.0;
  for (const auto& fk : F_) {
    const auto& f = fk.second; const double Df = std::max(0.0, f.demand_mbps);
    for (int pid : f.cand_path_ids) {
      double x = sol[x_col_.at({f.id, pid})];
      if (x <= 1e-9) continue;
      for (const auto& e : P_.at(pid).edges) out->load_mbps[e] += Df * x;
    }
  }
  return true;
}

void MILP_TE::build_fp_incidence_() {
  fp_use_.clear();
  for (const auto& fk : F_) {
    const auto& f = fk.second;
    for (int pid : f.cand_path_ids) {
      const auto& p = P_.at(pid);
      for (const auto& e : p.edges) fp_use_.insert(FPE{f.id, pid, e});
    }
  }
}

void MILP_TE::build_variable_index_() {
  x_index_.clear(); be_index_.clear();
  x_col_.clear();   be_col_.clear();
  int col = 0;
  for (const auto& fk : F_) {
    const auto& f = fk.second;
    for (int pid : f.cand_path_ids) {
      x_index_.push_back({f.id, pid});
      x_col_[{f.id, pid}] = col++;
    }
  }
  for (const auto& e : links_) {
    if (!G_.sdn(e)) continue;
    be_index_.push_back(e);
    be_col_[e] = col++;
  }
}

} // namespace te

