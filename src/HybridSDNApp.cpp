// src/HybridSDNApp.cpp
#include "HybridSDNApp.hpp"

using namespace std::chrono;

// ---- 建構子定義（要跟 hpp 簽名完全一致）----
HybridSDNApp::HybridSDNApp(uint16_t of_port, const Paths& paths)
  : of_port_(of_port),
    paths_(paths),
    // TopoViewer(ctl 指標, swid->node 映射, lldp 週期, edge 過期秒數)
    topo_(&ctl_, [](int sw){ return sw; }, milliseconds(1000), seconds(10)),
    // Monitor(ctl 指標, 容量查詢, 取樣週期)
    mon_(&ctl_, [this](const ::LinkId& e){ return this->cap_lookup_(e); }, milliseconds(2000))
{
  // 載入拓樸與流
  runtime_graph_ = load_graph_json_(paths_.graph_json);

  Forecast::Config fcfg;
  fcfg.alpha = 0.6;
  fcfg.adaptive_alpha = true;
  fcfg.adapt_window = 6;
  fcfg.alpha_min = 0.3;
  fcfg.alpha_max  = 0.9;
  forecast_ = std::make_unique<Forecast>(fcfg);

  flows_ = load_flows_csv_or_default_(paths_.flows_csv, runtime_graph_.nodes);
}

// ---- run() 最小可運作版本（之後你可替換為完整控制邏輯）----
void HybridSDNApp::run() {
  // 啟動控制器與背景模組
  if (!ctl_.start(of_port_)) {
    throw std::runtime_error("Failed to start OpenFlow controller");
  }
  topo_.start();
  mon_.start();

  // 先跑個簡單的示範循環（避免主程式立刻結束）
  // 你可以把這段替換成先前的 MILP 迴圈
  for (int i = 0; i < 3 && running_; ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  stop();
}

// ---- stop() ----
void HybridSDNApp::stop() {
  if (!running_) return;
  running_ = false;
  topo_.stop();
  mon_.stop();
  ctl_.stop();
}

// ---- 輔助：讀 flows（或產生 Demo）----
std::vector<te::Flow>
HybridSDNApp::load_flows_csv_or_default_(const std::string& path,
                                         const std::vector<int>& /*nodes*/) {
  std::vector<te::Flow> flows;
  std::ifstream ifs(path);
  if (!ifs) {
    // Demo flows
    std::vector<std::tuple<int,int,double>> demo = {
      {1, 9, 200.0}, {3, 7, 150.0}, {4,12, 180.0}, {6,11, 120.0}, {8,10, 160.0}
    };
    int id = 1;
    for (auto& t : demo) {
      flows.push_back(te::Flow{id, std::get<0>(t), std::get<1>(t), std::get<2>(t), {}});
      ++id;
    }
    return flows;
  }
  // CSV header: flow_id,s,d,demand_mbps
  std::string line; std::getline(ifs, line);
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line); std::string tok;
    std::vector<std::string> cols;
    while (std::getline(ss, tok, ',')) cols.push_back(tok);
    if (cols.size() < 4) continue;
    int id = std::stoi(cols[0]);
    int s  = std::stoi(cols[1]);
    int d  = std::stoi(cols[2]);
    double dem = std::stod(cols[3]);
    flows.push_back(te::Flow{id, s, d, dem, {}});
  }
  return flows;
}

