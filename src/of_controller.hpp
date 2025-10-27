#ifndef HYBRID_OF_CONTROLLER_HPP
#define HYBRID_OF_CONTROLLER_HPP
#pragma once
#include "models.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <functional>
#include <mutex>
#include <chrono>
#include <optional>
#include <tuple>

//
//  OpenFlow Controller Header
//  -----------------------------------------------
//  A generic controller interface for hybrid SDN experiments.
//  Designed for modularity: can be implemented using libfluid, raw sockets, or any OF1.0/1.3 stack.
//  Provides interfaces for LLDP, monitoring, topology discovery, and dynamic flow/port control.
//

// ---------------------------
// Basic statistics and IDs
// ---------------------------

// Per-port statistics structure (collected via OFPMP_PORT_STATS)
struct PortStats {
  uint64_t rx_bytes{0};
  uint64_t tx_bytes{0};
  uint32_t speedMbps{0};  // current link speed (administrative or measured)
};

// Logical link identifier used in aggregated network views
/*struct LinkId {
  int u{-1}, v{-1}; // graph node indices
  bool operator<(const LinkId& o) const { return std::tie(u,v) < std::tie(o.u,o.v); }
  bool operator==(const LinkId& o) const { return u==o.u && v==o.v; }
};*/

// Per-port information on each switch
struct PortInfo {
  int port_no{0};
  bool up{true};
  uint32_t curr_speedMbps{0};
  PortStats last; // last sampled stats
};

// Switch information container
struct SwitchInfo {
  int swid{0};                 // internal software ID
  uint64_t dpid{0};            // datapath ID reported by switch
  bool connected{false};
  std::map<int, PortInfo> ports; // key = port number
};

// Raw packet-in event
struct PacketIn {
  int swid{0};
  int in_port{0};
  const uint8_t* data{nullptr};
  size_t len{0};
};

// LLDP event (used by the Global Topology Viewer module)
struct LLDPEvent {
  int src_swid{0}, src_port{0};
  int dst_swid{0}, dst_port{0};
};

// ---------------------------
// Callback function types
// ---------------------------

using OnSwitchState   = std::function<void(int swid, bool up)>; // switch connect/disconnect
using OnPacketIn      = std::function<void(const PacketIn& pki)>; // any packet-in
using OnLLDP          = std::function<void(const LLDPEvent& e)>;  // LLDP discovery event
using OnError         = std::function<void(int swid, uint16_t type, uint16_t code, const std::string& msg)>; // error messages
using OnStatsReply    = std::function<void(int swid)>;             // asynchronous stats callback
using Clock           = std::chrono::steady_clock;

// ---------------------------
// Controller class interface
// ---------------------------

class OFController {
public:
  OFController();
  ~OFController();


  // ---- Controller lifecycle ----

  // Start the OpenFlow controller server (listening for switch connections)
  // Typically port = 6633 or 6653.
  bool start(uint16_t of_port);

  // Run blocking event loop (if using your own IO thread, this is optional)
  void run();

  // Stop and clean up sockets, threads, and all session state.
  void stop();

  // ---- Switch and port inventory ----

  // Get all currently connected switch IDs.
  std::vector<int> switch_ids() const;

  // Retrieve information for a specific switch.
  std::optional<SwitchInfo> switch_info(int swid) const;

  // Get a list of port numbers for a switch.
  std::vector<int> ports_of(int swid) const;

  // ---- Packet-out / LLDP ----

  // Send a custom Ethernet frame to a specific port.
  // The data must be a full L2 frame (dst/src/type/payload).
  void packet_out(int swid, int out_port, const uint8_t* eth, size_t len);

  // Build and send a standard LLDP packet for topology discovery.
  void send_lldp(int swid, int out_port);

  // ---- Monitoring ----

  // Poll port statistics for all connected SDN switches and aggregate into LinkId form.
  std::map<LinkId, PortStats> poll_port_stats();

  // Poll raw port statistics for one specific switch.
  std::map<int, PortStats> poll_port_stats(int swid);

  // ---- Control: Flows and Ports ----

  // Install or remove flow rules.
  // match/actions follow OpenFlow textual syntax, e.g.:
  //   match = "in_port=1,dl_type=0x0800,nw_src=10.0.0.1/32,nw_dst=10.0.0.9/32,tp_dst=5001"
  //   actions = "set_vlan_vid:100,output=2"
  void flow_mod(int swid,
                const std::string& match,
                const std::string& actions,
                int priority = 100,
                bool add = true,                     // false = delete rule
                std::optional<uint16_t> idle_timeout = {},
                std::optional<uint16_t> hard_timeout = {},
                std::optional<uint64_t> cookie = {});

  // Modify a port's administrative state or speed.
  // - up = false disables the port (used for energy saving)
  // - speedMbps can set rate adaptation (e.g., 1000, 10000, etc.)
  void port_mod(int swid, int port_no, bool up, int speedMbps);

  // Send a barrier request to ensure previous messages are processed.
  void barrier(int swid);

  // ---- Periodic timers ----

  // Set LLDP broadcast interval (default 1000 ms)
  void set_lldp_period(std::chrono::milliseconds p);

  // Set port-stats polling interval (default 2000 ms)
  void set_stats_period(std::chrono::milliseconds p);

  // ---- Callbacks ----

  void on_switch_state(OnSwitchState cb);
  void on_packet_in(OnPacketIn cb);
  void on_lldp(OnLLDP cb);
  void on_error(OnError cb);
  void on_stats_reply(OnStatsReply cb);

  // ---- Thread-safe inventory access ----

  // Return a snapshot of current switch inventory.
  std::map<int, SwitchInfo> inventory_snapshot() const;

  // ---- Utility ----

  // Helper to build a basic IPv4 5-tuple match string.
  static std::string ip_match(int in_port,
                              const std::string& src, const std::string& dst,
                              int ip_proto /* 6=tcp,17=udp,1=icmp */,
                              std::optional<int> tp_src = {},
                              std::optional<int> tp_dst = {});

private:
  // Non-copyable
  OFController(const OFController&) = delete;
  OFController& operator=(const OFController&) = delete;

  // Implementation hidden (PIMPL)
  struct Impl;
  Impl* impl_; // internal socket/event manager

  // Periods
  std::chrono::milliseconds lldp_period_{1000};
  std::chrono::milliseconds stats_period_{2000};

  // Callback functions
  OnSwitchState cb_switch_state_;
  OnPacketIn    cb_packet_in_;
  OnLLDP        cb_lldp_;
  OnError       cb_error_;
  OnStatsReply  cb_stats_;

  // Shared state (protected by mutex)
  mutable std::mutex mtx_;
  std::map<int, SwitchInfo> sws_;
};

#endif // HYBRID_OF_CONTROLLER_HPP

