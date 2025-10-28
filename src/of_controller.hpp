#ifndef HYBRID_OF_CONTROLLER_HPP
#define HYBRID_OF_CONTROLLER_HPP
#pragma once
#include "models.hpp"
#include <cstdint>
#include <cstddef>
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
// (aligned with te_controller.cpp: {swid, port})
/*struct LinkId {
  int swid{-1};   // switch index (1..N)
  int port{-1};   // port number on that switch
  bool operator<(const LinkId& o) const { return std::tie(swid, port) < std::tie(o.swid, o.port); }
  bool operator==(const LinkId& o) const { return swid == o.swid && port == o.port; }
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
  bool start(uint16_t of_port);  // Start the OpenFlow controller server (e.g., 6633/6653)
  void run();                    // Optional blocking loop (if not using internal thread)
  void stop();                   // Stop and cleanup

  // ---- Switch and port inventory ----
  std::vector<int> switch_ids() const;
  std::optional<SwitchInfo> switch_info(int swid) const;
  std::vector<int> ports_of(int swid) const;

  // ---- Packet-out / LLDP ----
  void packet_out(int swid, int out_port, const uint8_t* eth, size_t len);
  void send_lldp(int swid, int out_port);

  // ---- Monitoring ----
  std::map<LinkId, PortStats> poll_port_stats();  // aggregated (swid,port) view
  std::map<int, PortStats>    poll_port_stats(int swid); // per-switch raw

  // ---- Control: Flows and Ports ----
  void flow_mod(int swid,
                const std::string& match,
                const std::string& actions,
                int priority = 100,
                bool add = true,
                std::optional<uint16_t> idle_timeout = {},
                std::optional<uint16_t> hard_timeout = {},
                std::optional<uint64_t> cookie = {});

  void port_mod(int swid, int port_no, bool up, int speedMbps);
  void barrier(int swid);

  // ---- Periodic timers ----
  void set_lldp_period(std::chrono::milliseconds p);   // default 1000 ms
  void set_stats_period(std::chrono::milliseconds p);  // default 2000 ms

  // ---- Callbacks ----
  void on_switch_state(OnSwitchState cb);
  void on_packet_in(OnPacketIn cb);
  void on_lldp(OnLLDP cb);
  void on_error(OnError cb);
  void on_stats_reply(OnStatsReply cb);

  // ---- Thread-safe inventory access ----
  std::map<int, SwitchInfo> inventory_snapshot() const;

  // ---- Utility ----
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

