// src/actuator.cpp
#include <map>
#include <utility>
#include "of_controller.hpp"
#include "topo_viewer.cpp" // or include "topo_viewer.hpp" if you split headers
#include "milp_te.cpp"     // for TE_Output / LinkId; swap to header if you have one

static inline LinkId mk_edge(int a,int b){ if(a>b) std::swap(a,b); return LinkId{a,b}; }

struct Actuator {
  OFController* ctl;
  void apply_beta(const TE_Output& plan, const std::vector<TopoViewer::Edge>& alive_edges) {
    // (u,v) -> (u_port, v_port) and assume swid == node id
    std::map<LinkId, std::pair<int,int>> ports;
    std::map<LinkId, std::pair<int,int>> sw_of_node;
    for (const auto& e : alive_edges) {
      auto id = mk_edge(e.u, e.v);
      int up=e.u_port, vp=e.v_port;
      if (e.u > e.v) std::swap(up, vp);
      ports[id] = {up, vp};
      sw_of_node[id] = {e.u, e.v};
    }

    for (const auto& kv : plan.beta) {
      const auto& id = kv.first; int beta = kv.second;
      auto pit = ports.find(id);
      auto sit = sw_of_node.find(id);
      if (pit==ports.end() || sit==sw_of_node.end()) continue;

      int u = sit->second.first, v = sit->second.second;
      int u_port = pit->second.first, v_port = pit->second.second;

      bool up = (beta==1);
      int speed = up ? 10000 : 0; // up->10G, down->admin down (adjust as needed)
      ctl->port_mod(u, u_port, up, speed);
      ctl->port_mod(v, v_port, up, speed);
      ctl->barrier(u);
      ctl->barrier(v);
    }
  }
};

// If you need this class from other translation units, provide a header `actuator.hpp`:
// class Actuator { public: OFController* ctl; void apply_beta(...); };

