#include "topo_viewer.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

TopoViewer::TopoViewer(OFController* ctl,
                       std::function<int(int)> swid_to_node,
                       std::chrono::milliseconds lldp_period,
                       std::chrono::seconds expiry)
  : ctl_(ctl),
    swid_to_node_(std::move(swid_to_node)),
    lldp_period_(lldp_period),
    expiry_(expiry) {
  if (!swid_to_node_) {
    // Default to identity mapping: node_id == swid
    swid_to_node_ = [](int sw){ return sw; };
  }
  // Subscribe to controller LLDP events
  ctl_->on_lldp([this](const LLDPEvent& e){ this->handle_lldp(e); });
}

TopoViewer::~TopoViewer() { stop(); }

void TopoViewer::start() {
  if (running_.exchange(true)) return;
  // Align controller's LLDP period to our setting
  ctl_->set_lldp_period(lldp_period_);

  bg_ = std::thread([this](){
    auto next = Clock::now();
    while (running_.load()) {
      auto now = Clock::now();
      if (now >= next) {
        tick_send_lldp();  // 1) broadcast LLDP
        prune_expired();   // 2) prune expired edges
        next = now + lldp_period_;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
}

void TopoViewer::stop() {
  if (!running_.exchange(false)) return;
  if (bg_.joinable()) bg_.join();
}

void TopoViewer::tick_send_lldp() {
  auto swids = ctl_->switch_ids();
  for (int sw : swids) {
    auto ports = ctl_->ports_of(sw);
    for (int p : ports) {
      ctl_->send_lldp(sw, p); // controller constructs LLDP frames internally.
    }
  }
}

void TopoViewer::prune_expired() {
  const auto now = Clock::now();
  std::lock_guard<std::mutex> lk(mtx_);
  for (auto it = edges_.begin(); it != edges_.end(); ) {
    if (now - it->second > expiry_) it = edges_.erase(it);
    else ++it;
  }
}

void TopoViewer::handle_lldp(const LLDPEvent& e) {
  // Map switch IDs to graph node IDs
  const int nu = swid_to_node_(e.src_swid);
  const int nv = swid_to_node_(e.dst_swid);
  if (nu == nv) return; // ignore self-loops

  // Canonicalize (u < v); swap corresponding ports if needed
  EdgeKey k{};
  if (nu < nv) {
    k = EdgeKey{nu, nv, e.src_port, e.dst_port};
  } else {
    k = EdgeKey{nv, nu, e.dst_port, e.src_port};
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    edges_[k] = Clock::now();
  }

  // Optional debug:
  // std::cerr << "[topo] LLDP: " << nu << ":" << e.src_port
  //           << " -- " << nv << ":" << e.dst_port << "\n";
}

std::vector<TopoViewer::Edge> TopoViewer::snapshot_edges() const {
  std::vector<Edge> out;
  std::lock_guard<std::mutex> lk(mtx_);
  out.reserve(edges_.size());
  for (const auto& kv : edges_) {
    const auto& k = kv.first;
    out.push_back(Edge{k.u, k.v, k.u_port, k.v_port, kv.second});
  }
  return out;
}

std::string TopoViewer::export_dot() const {
  std::ostringstream os;
  os << "graph SDN {\n";
  os << "  graph [overlap=false, splines=true];\n";
  os << "  node  [shape=circle, fontsize=10];\n";
  // Nodes (collect from edges)
  std::set<int> nodes;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto& kv : edges_) {
      nodes.insert(kv.first.u);
      nodes.insert(kv.first.v);
    }
  }
  for (int n : nodes) os << "  " << n << ";\n";

  // Edges with port labels
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto& kv : edges_) {
      const auto& e = kv.first;
      os << "  " << e.u << " -- " << e.v
         << " [label=\"(" << e.u_port << "," << e.v_port << ")\"];\n";
    }
  }
  os << "}\n";
  return os.str();
}

void TopoViewer::set_swid_to_node_mapper(std::function<int(int)> f) {
  if (!f) return;
  std::lock_guard<std::mutex> lk(mtx_);
  swid_to_node_ = std::move(f);
}

