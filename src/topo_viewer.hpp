#pragma once
#ifndef HYBRID_TOPO_VIEWER_HPP
#define HYBRID_TOPO_VIEWER_HPP

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>
#include<atomic>
#include "of_controller.hpp"

class TopoViewer {
public:
  using Clock = std::chrono::steady_clock;

  // swid_to_node: optional mapping from switch ID to graph node ID (default: identity).
  explicit TopoViewer(OFController* ctl,
                      std::function<int(int)> swid_to_node = {},
                      std::chrono::milliseconds lldp_period = std::chrono::milliseconds(1000),
                      std::chrono::seconds expiry = std::chrono::seconds(10));
  ~TopoViewer();

  // Start/stop background thread that sends periodic LLDP and prunes expired edges.
  void start();
  void stop();

  // Send one round of LLDP on all known switches/ports.
  void tick_send_lldp();

  // Remove edges whose last_seen is older than 'expiry_'.
  void prune_expired();

  // Edge snapshot item
  struct Edge {
    int u, v;            // node IDs (canonical: u < v)
    int u_port, v_port;  // port numbers on each side
    Clock::time_point last_seen;
  };
  std::vector<Edge> snapshot_edges() const;

  // Export current topology to Graphviz DOT string.
  std::string export_dot() const;

  // Change mapping from swid -> node id
  void set_swid_to_node_mapper(std::function<int(int)> f);

private:
  // Controller will call this via on_lldp() when an LLDP frame is observed.
  void handle_lldp(const LLDPEvent& e);

  // Canonical edge key (undirected, sorted by node ID)
  struct EdgeKey {
    int u, v;
    int u_port, v_port; // ports as seen on u and v respectively
    bool operator<(const EdgeKey& o) const {
      return std::tie(u,v,u_port,v_port) < std::tie(o.u,o.v,o.u_port,o.v_port);
    }
  };

  OFController* ctl_;
  std::function<int(int)> swid_to_node_;
  std::chrono::milliseconds lldp_period_;
  std::chrono::seconds expiry_;

  mutable std::mutex mtx_;
  std::map<EdgeKey, Clock::time_point> edges_; // last_seen timestamp

  std::atomic<bool> running_{false};
  std::thread bg_;
};

#endif // HYBRID_TOPO_VIEWER_HPP

