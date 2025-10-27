#include "forecast.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

double Forecast::ewma_next(const std::vector<double>& hist, double alpha) {
  if (hist.empty()) return 0.0;
  double s = hist[0];
  for (size_t i = 1; i < hist.size(); ++i) {
    s = alpha * hist[i] + (1.0 - alpha) * s;
  }
  return s; // next-step forecast equals smoothed state
}

double Forecast::adapt_alpha(const std::vector<double>& hist,
                             double alpha_min, double alpha_max, int win) {
  if (!std::isfinite(alpha_min) || !std::isfinite(alpha_max) || alpha_min > alpha_max) {
    return std::clamp(0.6, 0.0, 1.0);
  }
  if ((int)hist.size() < std::max(2, win)) {
    return std::clamp((alpha_min + alpha_max) * 0.5, 0.0, 1.0);
  }
  // Take last 'win' samples
  const int n = std::min<int>(win, (int)hist.size());
  const double m = std::accumulate(hist.end()-n, hist.end(), 0.0) / n;
  if (m <= 0.0) return alpha_min;

  double var = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = (hist.end()-n)[i] - m;
    var += d * d;
  }
  var /= std::max(1, n-1);
  const double sd = std::sqrt(var);
  const double cov = sd / std::max(1e-9, m); // coefficient of variation

  // Map CoV to alpha in [alpha_min, alpha_max] with soft clipping
  // cov_ref ~ 0.3 means moderate volatility
  const double cov_ref = 0.3;
  double x = cov / cov_ref;                 // normalized
  double w = x / (1.0 + x);                 // in (0,1)
  double a = alpha_min + (alpha_max - alpha_min) * w;
  return std::clamp(a, alpha_min, alpha_max);
}

Forecast::PredSummary
Forecast::predict_next(const std::map<LinkId, std::vector<double>>& hist_map) const {
  PredSummary out;
  out.next.clear();
  if (hist_map.empty()) {
    out.peak = 0.0; out.mean = 0.0;
    return out;
  }

  double sum = 0.0;
  double pk = 0.0;
  size_t cnt = 0;

  for (const auto& kv : hist_map) {
    const LinkId& id = kv.first;
    const std::vector<double>& h = kv.second;
    double a = cfg_.alpha;
    if (cfg_.adaptive_alpha) {
      a = adapt_alpha(h, cfg_.alpha_min, cfg_.alpha_max, cfg_.adapt_window);
    }
    const double pred = ewma_next(h, a);
    out.next[id] = pred;
    pk = std::max(pk, pred);
    sum += pred;
    cnt += 1;
  }

  out.peak = pk;
  out.mean = (cnt ? (sum / cnt) : 0.0);
  return out;
}

Forecast::Weights
Forecast::weights_from_peak(double predicted_peak_mbps, double threshold_mbps) {
  if (!(threshold_mbps > 0.0)) {
    return Weights{1.0, 0.0}; // degenerate: no threshold -> prioritize energy
  }
  const double r = predicted_peak_mbps / threshold_mbps;
  return weights_from_ratio(r, /*gamma=*/1.25);
}

Forecast::Weights
Forecast::weights_from_ratio(double peak_over_thresh, double gamma) {
  // Smooth S-shaped mapping from ratio to LWr in [0,1]
  // LWr = clamp( r^gamma / (1 + r^gamma) )
  const double r = std::max(0.0, peak_over_thresh);
  const double rg = std::pow(r, std::max(0.5, gamma));
  const double lwr = rg / (1.0 + rg);
  const double ewr = 1.0 - lwr;
  return Weights{ewr, lwr};
}

double Forecast::mean(const std::vector<double>& xs) {
  if (xs.empty()) return 0.0;
  double s = std::accumulate(xs.begin(), xs.end(), 0.0);
  return s / xs.size();
}

double Forecast::peak(const std::vector<double>& xs) {
  if (xs.empty()) return 0.0;
  return *std::max_element(xs.begin(), xs.end());
}

