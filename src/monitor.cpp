#include "monitor.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

// -------------------------
// Helpers (static)
// -------------------------
double Monitor::mbps_from_bytes_delta(uint64_t dbytes, double dt_sec) {
  if (dt_sec <= 0.0) return 0.0;
  // bits per second / 1e6
  return (8.0 * static_cast<double>(dbytes)) / dt_sec / 1e6;
}

std::string Monitor::to_iso8601(Clock::time_point tp) {
  using namespace std::chrono;
  auto now_c = system_clock::now() + duration_cast<system_clock::duration>(tp - Clock::now());
  std::time_t t = system_clock::to_time_t(now_c);
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

// -------------------------
// Implementation
// -------------------------

Monitor::Monitor(OFController* ctl,
                 std::function<double(const LinkId&)> capacity_mbps,
                 std::chrono::milliseconds period)
  : ctl_(ctl),
    cap_mbps_(std::move(capacity_mbps)),
    period_(period) {}

Monitor::~Monitor() { stop(); }

std::vector<Monitor::Sample>
Monitor::compute_rates_and_update(const std::map<LinkId, PortStats>& now) {
  const auto tnow = Clock::now();
  std::vector<Sample> out;
  out.reserve(now.size());

  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto& kv : now) {
    const LinkId& id = kv.first;
    const PortStats& ps = kv.second;

    auto& last = last_counters_[id];
    double dt = 0.0;
    uint64_t drx = 0, dtx = 0;

    if (last.valid) {
      dt  = std::chrono::duration<double>(tnow - last.t).count();
      drx = (ps.rx_bytes >= last.rx_bytes) ? (ps.rx_bytes - last.rx_bytes) : 0;
      dtx = (ps.tx_bytes >= last.tx_bytes) ? (ps.tx_bytes - last.tx_bytes) : 0;
    }

    // Update last counters
    last.rx_bytes = ps.rx_bytes;
    last.tx_bytes = ps.tx_bytes;
    last.t = tnow;
    last.valid = true;

    // Compute Mbps
    LinkRate r;
    if (dt > 0.0) {
      r.rx_mbps = mbps_from_bytes_delta(drx, dt);
      r.tx_mbps = mbps_from_bytes_delta(dtx, dt);
    } else {
      r.rx_mbps = r.tx_mbps = 0.0;
    }

    // Utilization with capacity guard
    double cap = 0.0;
    try { cap = cap_mbps_(id); } catch (...) { cap = 0.0; }
    if (cap > 0.0) {
      r.util = std::min(1.0, std::max(0.0, (r.rx_mbps + r.tx_mbps) / cap));
    } else {
      r.util = 0.0;
    }

    // Save instantaneous
    last_rates_[id] = r;
    // Append to time series
    series_[id].push_back(Sample{id, tnow, r});
    out.push_back(Sample{id, tnow, r});
  }

  return out;
}

std::vector<Monitor::Sample> Monitor::sample_once() {
  // Ask controller for aggregated per-LinkId counters
  auto counters = ctl_->poll_port_stats();
  return compute_rates_and_update(counters);
}

void Monitor::start() {
  if (running_.exchange(true)) return;
  // Align controller's stats period to ours (optional; your OFController may ignore)
  ctl_->set_stats_period(period_);

  bg_ = std::thread([this](){
    auto next = Clock::now();
    while (running_.load()) {
      auto now = Clock::now();
      if (now >= next) {
        try {
          (void)sample_once();
        } catch (const std::exception& e) {
          std::cerr << "[monitor] sample_once error: " << e.what() << "\n";
        }
        next = now + period_;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
}

void Monitor::stop() {
  if (!running_.exchange(false)) return;
  if (bg_.joinable()) bg_.join();
}

std::map<LinkId, Monitor::LinkRate>
Monitor::window_average(std::chrono::seconds dur) {
  // Collect samples until duration elapsed
  const auto t_start = Clock::now();
  std::map<LinkId, double> sum_rx, sum_tx;
  std::map<LinkId, int> cnt;

  do {
    auto v = sample_once();
    for (auto& s : v) {
      sum_rx[s.id] += s.rate.rx_mbps;
      sum_tx[s.id] += s.rate.tx_mbps;
      cnt[s.id]    += 1;
    }
    std::this_thread::sleep_for(period_);
  } while (Clock::now() - t_start < dur);

  // Build average map
  std::map<LinkId, LinkRate> avg;
  for (auto& kv : cnt) {
    const LinkId& id = kv.first;
    int c = kv.second;
    if (c <= 0) continue;
    double cap = 0.0;
    try { cap = cap_mbps_(id); } catch (...) { cap = 0.0; }

    LinkRate r;
    r.rx_mbps = sum_rx[id] / c;
    r.tx_mbps = sum_tx[id] / c;
    r.util    = (cap > 0.0) ? std::min(1.0, std::max(0.0, (r.rx_mbps + r.tx_mbps) / cap)) : 0.0;
    avg[id] = r;
  }
  return avg;
}

std::map<LinkId, Monitor::LinkRate> Monitor::last_rates_snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return last_rates_;
}

std::vector<Monitor::Sample> Monitor::timeseries(const LinkId& id) const {
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = series_.find(id);
  if (it == series_.end()) return {};
  return it->second;
}

bool Monitor::export_csv(const std::string& path, size_t max_points_per_link) const {
  std::ofstream ofs(path);
  if (!ofs) return false;

  ofs << "time_iso,u,v,rx_mbps,tx_mbps,util\n";
  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto& kv : series_) {
    const LinkId& id = kv.first;
    const auto& vec = kv.second;
    size_t start = 0;
    if (max_points_per_link > 0 && vec.size() > max_points_per_link) {
      start = vec.size() - max_points_per_link;
    }
    for (size_t i = start; i < vec.size(); ++i) {
      const auto& s = vec[i];
      ofs << to_iso8601(s.t) << ","
          << id.u << "," << id.v << ","
          << std::fixed << std::setprecision(6)
          << s.rate.rx_mbps << ","
          << s.rate.tx_mbps << ","
          << s.rate.util << "\n";
    }
  }
  return true;
}

void Monitor::set_period(std::chrono::milliseconds p) { period_ = p; }

