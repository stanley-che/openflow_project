#pragma once
#ifndef HYBRID_MONITOR_HPP
#define HYBRID_MONITOR_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "of_controller.hpp"  // needs PortStats + OFController + LinkId

class Monitor {
public:
  using Clock = std::chrono::steady_clock;

  // Per-link instantaneous rate/utilization
  struct LinkRate {
    double rx_mbps{0.0};
    double tx_mbps{0.0};
    double util{0.0};   // (rx+tx)/capacity, clamped to [0, 1] if capacity>0
  };

  // One sample datapoint
  struct Sample {
    LinkId id;
    Clock::time_point t;
    LinkRate rate;
  };

  // capacity_mbps: callback providing capacity (in Mbps) for a given LinkId.
  // period: sampling interval for background mode and window_average().
  explicit Monitor(OFController* ctl,
                   std::function<double(const LinkId&)> capacity_mbps,
                   std::chrono::milliseconds period = std::chrono::milliseconds(2000));

  ~Monitor();

  // ---- synchronous / background control ----
  // Take one synchronous sample now; returns per-link datapoints.
  std::vector<Sample> sample_once();

  // Start/stop background periodic sampling into internal time series.
  void start();
  void stop();

  // Compute window-average rates over a real-time duration (blocking).
  // It samples at 'period_' boundaries until 'dur' elapses, then returns per-link averages.
  std::map<LinkId, LinkRate> window_average(std::chrono::seconds dur);

  // ---- snapshots / queries ----
  // Get the latest per-link instantaneous rates (thread-safe snapshot).
  std::map<LinkId, LinkRate> last_rates_snapshot() const;

  // Return the in-memory time series for a specific link (most recent first = false).
  std::vector<Sample> timeseries(const LinkId& id) const;

  // ---- export ----
  // Export recent time series to CSV. Columns: time_iso,u,v,rx_mbps,tx_mbps,util
  // If max_points_per_link>0, truncate to the most recent K points per link.
  bool export_csv(const std::string& path, size_t max_points_per_link = 0) const;

  // Adjust sampling period for background thread and window-average cadence.
  void set_period(std::chrono::milliseconds p);

private:
  // Internal state kept per link for delta computation
  struct LastCounter {
    uint64_t rx_bytes{0};
    uint64_t tx_bytes{0};
    Clock::time_point t{};
    bool valid{false};
  };

  // Helpers
  static double mbps_from_bytes_delta(uint64_t dbytes, double dt_sec);
  static std::string to_iso8601(Clock::time_point tp);

  // Compute instantaneous rates from current counters, updating last_counters_
  std::vector<Sample> compute_rates_and_update(const std::map<LinkId, PortStats>& now);

private:
  OFController* ctl_;
  std::function<double(const LinkId&)> cap_mbps_;
  std::chrono::milliseconds period_;

  // Mutable state
  mutable std::mutex mtx_;
  std::map<LinkId, LastCounter> last_counters_;
  std::map<LinkId, LinkRate> last_rates_;
  std::map<LinkId, std::vector<Sample>> series_; // append-only time series per link

  // Background thread
  std::atomic<bool> running_{false};
  std::thread bg_;
};

#endif // HYBRID_MONITOR_HPP

