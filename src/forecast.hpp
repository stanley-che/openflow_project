#pragma once
#ifndef HYBRID_FORECAST_HPP
#define HYBRID_FORECAST_HPP

#include <map>
#include <vector>
#include <optional>
#include <chrono>
#include <tuple>
#include <utility>
#include <cstdint>

#include "models.hpp"  // 需要 LinkId { int u,v; <,== 比較 } 的宣告

class Forecast {
public:
  // Weights returned to TE/MILP: Energy weight (EWr) and Load weight (LWr).
  struct Weights { double ewr{0.5}; double lwr{0.5}; };

  // Summary of predictions for a time slice.
  struct PredSummary {
    std::map<LinkId, double> next; // next-step prediction per link (e.g., Mbps)
    double peak{0.0};              // max over links
    double mean{0.0};              // mean over links
  };

  // Configuration for EWMA / adaptive alpha
  struct Config {
    double alpha{0.6};                  // base EWMA alpha in [0,1]
    bool   adaptive_alpha{true};        // enable adaptive alpha
    int    adapt_window{6};             // look-back for volatility-based adapt
    double alpha_min{0.3}, alpha_max{0.9};
  };

  Forecast() = default;
  explicit Forecast(Config cfg): cfg_(cfg) {}

  // ---- Core: single-series EWMA ----
  // Returns the next-step prediction from a non-empty series 'hist'.
  // If hist.size()==1, returns hist[0]. Empty series returns 0.0.
  static double ewma_next(const std::vector<double>& hist, double alpha);

  // Adaptive alpha based on recent coefficient of variation (CoV).
  // Higher volatility -> higher alpha (more reactive).
  static double adapt_alpha(const std::vector<double>& hist,
                            double alpha_min, double alpha_max, int win);

  // ---- Batch prediction over multiple links ----
  // hist_map: link -> chronological samples (e.g., Mbps).
  // If a link has empty history, it is predicted as 0.0.
  PredSummary predict_next(const std::map<LinkId, std::vector<double>>& hist_map) const;

  // Compute (EWr, LWr) weights from predicted peak and a capacity threshold (Mbps).
  // Intuition: if peak is high vs. threshold -> prioritize congestion (LWr up).
  static Weights weights_from_peak(double predicted_peak_mbps, double threshold_mbps);

  // Convenience overload using ratio and a shaping exponent gamma.
  static Weights weights_from_ratio(double peak_over_thresh, double gamma = 1.0);

  // Utility: mean, peak for a vector
  static double mean(const std::vector<double>& xs);
  static double peak(const std::vector<double>& xs);

  // Optional: set/get config
  void set_config(const Config& c) { cfg_ = c; }
  Config config() const { return cfg_; }

private:
  Config cfg_{};
};

#endif // HYBRID_FORECAST_HPP

