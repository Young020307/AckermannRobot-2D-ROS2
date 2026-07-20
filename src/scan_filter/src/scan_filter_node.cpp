// scan_filter_node.cpp
//
// Bayesian dynamic obstacle filter for 2D LaserScan.
//
// Key fix (v2): frozen reference
// ──────────────────────────────
// The original implementation used the immediately-preceding scan as the
// reference.  This caused a silent failure: after the FIRST frame in which
// a person appeared in a beam, that person's range became the new reference.
// In the SECOND frame the comparison r_new ≈ r_ref yielded zero dynamic
// evidence, and the probability never rose above the removal threshold.
//
// Fix: maintain a per-beam "frozen reference" (frozen_ref_) that is only
// updated for beams whose P(dynamic) < freeze_threshold.  As soon as a beam
// shows the first sign of a dynamic obstacle (P_dyn rises above the prior),
// its reference is frozen at the pre-obstacle range (the static background).
// Subsequent frames with the obstacle present keep firing dynamic evidence
// against the frozen background range, so P_dyn accumulates correctly.
// When the obstacle leaves and the range returns to the background, the beam
// is reclassified as static and the frozen reference starts updating again.
//
// This mirrors the behaviour of the original 3D mapping algorithm, where the
// persistent map (reference) is only updated for static points.
//
// Algorithm summary
// -----------------
//  1. A rolling buffer of the last `buffer_size` raw scans is kept.
//  2. A "current reference" is derived from that buffer (last frame or median).
//     This is used only for the motion guard.
//  3. A "frozen reference" is maintained per beam.  It equals the current
//     reference for static beams and is frozen (not updated) for dynamic ones.
//  4. Each beam's r_new is compared to frozen_ref[i]:
//       w_d2 > 0  when r_new is significantly SHORTER (obstacle appeared)
//       w_p2 > 0  when r_new matches the reference (no change)
//  5. Bayesian update propagates P(static) / P(dynamic) per beam.
//  6. Beams with P(dynamic) > dyn_threshold are set to infinity in output.
//
// Parameter tuning cheat-sheet
// ----------------------------
//   eps_d          must be larger than the range change caused by robot motion
//                  per frame.  At 0.5 m/s, 10 Hz → 5 cm/frame; set 0.30–0.50 m.
//   eps_a          proportional component (3–5 % of range)
//   freeze_threshold  P(dynamic) above which frozen_ref stops updating.
//                  Default 0.50 (freeze as soon as any dynamic evidence appears).
//   dyn_threshold  removal threshold; lower → filter more aggressively
//   temporal_smooth  (c1) inertia of probability estimate; 0.5–0.65 is typical
//   alpha          P(static→static); high → walls resist being flipped
//   beta           P(dynamic→dynamic); low → dynamic label decays fast

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// ---------------------------------------------------------------------------
class ScanFilterNode : public rclcpp::Node
{
public:
  explicit ScanFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("scan_filter_node", options)
  {
    declare_parameter("input_scan_topic",  std::string("/scan"));
    declare_parameter("output_scan_topic", std::string("/scan_filtered"));

    declare_parameter("prior_static",   0.5);
    declare_parameter("prior_dynamic",  0.5);

    // Range-comparison error model
    // Must absorb robot-motion-induced range change between frames.
    // At 0.5 m/s, 10 Hz → 5 cm/frame; set eps_d well above that.
    declare_parameter("eps_a", 0.05);   // proportional component  [fraction]
    declare_parameter("eps_d", 0.40);   // fixed floor             [m]

    // Bayesian transition probabilities
    declare_parameter("alpha", 0.90);   // P(static  → static)
    declare_parameter("beta",  0.55);   // P(dynamic → dynamic)  – low → fast decay

    declare_parameter("max_dyn",          0.90);
    declare_parameter("dyn_threshold",    0.65);

    // Temporal smoothing (c1): weight of previous estimate in each update
    declare_parameter("temporal_smooth",  0.60);

    // Frozen-reference threshold:
    // frozen_ref[i] is NOT updated when P(dynamic)[i] > freeze_threshold.
    // This prevents the reference from adapting to a persistent obstacle.
    declare_parameter("freeze_threshold", 0.50);

    declare_parameter("eps", 1e-4);

    declare_parameter("buffer_size",     3);
    declare_parameter("reference_mode",  std::string("last"));

    // Motion guard: skip update if > this fraction of beams change at once
    declare_parameter("motion_guard_frac", 0.55);

    // ML hook – reserved for future methods
    declare_parameter("filter_method", std::string("bayesian"));

    loadParameters();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(std::move(msg)); });

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "ScanFilterNode ready  '%s' → '%s'  "
      "eps_d=%.2f  dyn_thr=%.2f  freeze_thr=%.2f  buf=%d  mode=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      eps_d_, dyn_threshold_, freeze_threshold_,
      buffer_size_, reference_mode_.c_str());
  }

private:
  // ── Parameter loading ──────────────────────────────────────────────────
  void loadParameters()
  {
    input_topic_       = get_parameter("input_scan_topic").as_string();
    output_topic_      = get_parameter("output_scan_topic").as_string();
    prior_static_      = get_parameter("prior_static").as_double();
    prior_dynamic_     = get_parameter("prior_dynamic").as_double();
    eps_a_             = get_parameter("eps_a").as_double();
    eps_d_             = get_parameter("eps_d").as_double();
    alpha_             = get_parameter("alpha").as_double();
    beta_              = get_parameter("beta").as_double();
    max_dyn_           = get_parameter("max_dyn").as_double();
    dyn_threshold_     = get_parameter("dyn_threshold").as_double();
    c1_                = get_parameter("temporal_smooth").as_double();
    freeze_threshold_  = get_parameter("freeze_threshold").as_double();
    eps_               = get_parameter("eps").as_double();
    buffer_size_       = get_parameter("buffer_size").as_int();
    reference_mode_    = get_parameter("reference_mode").as_string();
    motion_guard_frac_ = get_parameter("motion_guard_frac").as_double();
    filter_method_     = get_parameter("filter_method").as_string();
  }

  // ── Main scan callback ─────────────────────────────────────────────────
  void onScan(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const std::size_t n = msg->ranges.size();

    // First message: initialise per-beam state
    if (prob_static_.size() != n) {
      prob_static_.assign(n, static_cast<float>(prior_static_));
      prob_dynamic_.assign(n, static_cast<float>(prior_dynamic_));
      frozen_ref_.assign(n, std::numeric_limits<float>::infinity());
      scan_buffer_.clear();
      RCLCPP_INFO(get_logger(), "Initialised state for %zu beams.", n);
    }

    // ── 1. Update rolling buffer ─────────────────────────────────────────
    scan_buffer_.emplace_back(msg->ranges.begin(), msg->ranges.end());
    if (static_cast<int>(scan_buffer_.size()) > buffer_size_) {
      scan_buffer_.erase(scan_buffer_.begin());
    }

    if (scan_buffer_.size() < 2u) {
      scan_pub_->publish(*msg);
      return;
    }

    // ── 2. Build current reference (used for motion guard) ───────────────
    const std::vector<float> cur_ref = buildReference(n);

    // ── 3. Initialise frozen_ref on first valid reference ────────────────
    bool frozen_ref_valid = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (std::isfinite(frozen_ref_[i])) { frozen_ref_valid = true; break; }
    }
    if (!frozen_ref_valid) {
      frozen_ref_ = cur_ref;
    }

    // ── 4. Motion guard ──────────────────────────────────────────────────
    {
      float changed_frac = 0.0f;
      if (isRobotMotionFrame(*msg, cur_ref, n, changed_frac)) {
        ++diag_guard_triggered_;
        diag_guard_frac_accum_ += changed_frac;
        ++diag_guard_total_frames_;
        printGuardDiag();
        scan_pub_->publish(*msg);
        return;
      }
      ++diag_guard_total_frames_;
    }

    // ── 5. Bayesian update using frozen_ref ──────────────────────────────
    const float f_c1      = static_cast<float>(c1_);
    const float f_c2      = 1.0f - f_c1;
    const float f_eps     = static_cast<float>(eps_);
    const float f_a       = static_cast<float>(alpha_);
    const float f_b       = static_cast<float>(beta_);
    const float f_maxd    = static_cast<float>(max_dyn_);
    const float f_epsa    = static_cast<float>(eps_a_);
    const float f_epsd    = static_cast<float>(eps_d_);
    const float f_frzthr  = static_cast<float>(freeze_threshold_);

    for (std::size_t i = 0; i < n; ++i) {
      const float r_new = msg->ranges[i];

      // Invalid current reading: decay toward prior
      if (!std::isfinite(r_new) || r_new < msg->range_min || r_new > msg->range_max) {
        prob_dynamic_[i] = (1.0f - f_eps) * prob_dynamic_[i] +
                           f_eps * static_cast<float>(prior_dynamic_);
        prob_static_[i]  = (1.0f - f_eps) * prob_static_[i]  +
                           f_eps * static_cast<float>(prior_static_);
        // Unfreeze reference if beam is invalid (treat as gone)
        if (std::isfinite(cur_ref[i])) { frozen_ref_[i] = cur_ref[i]; }
        continue;
      }

      const float r_ref = frozen_ref_[i];

      // No frozen reference yet for this beam: initialise and skip
      if (!std::isfinite(r_ref) || r_ref < msg->range_min || r_ref > msg->range_max) {
        frozen_ref_[i] = r_new;
        continue;
      }

      // ── Evidence weights ──────────────────────────────────────────────
      // Only flag when r_new is significantly SHORTER than the frozen reference.
      // A longer reading means the view "opened up" – that is benign.
      const float range_decrease = r_ref - r_new;           // >0 when r_new < r_ref
      const float d_max          = f_epsa * r_new;
      const float offset         = range_decrease - f_epsd;

      float w_d2, w_p2;
      if (range_decrease < f_epsd) {
        w_d2 = f_eps;
        w_p2 = 1.0f;
      } else if (d_max < f_eps || offset >= d_max) {
        w_d2 = 1.0f;
        w_p2 = f_eps;
      } else {
        const float frac = offset / d_max;
        w_d2 = f_eps + (1.0f - f_eps) * frac;
        w_p2 = f_eps + (1.0f - f_eps) * (1.0f - frac);
      }

      // ── Bayesian update ───────────────────────────────────────────────
      const float p_d = prob_dynamic_[i];
      const float p_s = prob_static_[i];

      float new_pd, new_ps;
      if (p_d < f_maxd) {
        new_pd = f_c1 * p_d + f_c2 * w_d2 * ((1.0f - f_a) * p_s + f_b * p_d);
        new_ps = f_c1 * p_s + f_c2 * w_p2 * (f_a * p_s + (1.0f - f_b) * p_d);
      } else {
        new_pd = 1.0f - f_eps;
        new_ps = f_eps;
      }

      const float sum = new_pd + new_ps;
      if (sum > f_eps) {
        prob_dynamic_[i] = new_pd / sum;
        prob_static_[i]  = new_ps / sum;
      }

      // ── Update frozen reference only for static beams ─────────────────
      // If P(dynamic) is below the freeze threshold, this beam is considered
      // static: allow its reference to follow the current scan (tracking robot
      // motion).  Otherwise freeze it so the background range is preserved.
      if (prob_dynamic_[i] < f_frzthr) {
        // Static beam: update frozen_ref toward current reading (smooth)
        // Use cur_ref (not r_new) to keep the reference lag-free w.r.t. motion
        if (std::isfinite(cur_ref[i])) {
          frozen_ref_[i] = cur_ref[i];
        }
      }
      // Dynamic beam: frozen_ref_[i] unchanged
    }

    // ── 6. Build and publish filtered scan ───────────────────────────────
    auto out = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    const float f_thr = static_cast<float>(dyn_threshold_);
    int removed = 0, valid_total = 0;

    for (std::size_t i = 0; i < n; ++i) {
      if (std::isfinite(msg->ranges[i]) &&
          msg->ranges[i] >= msg->range_min &&
          msg->ranges[i] <= msg->range_max) {
        ++valid_total;
      }
      if (prob_dynamic_[i] > f_thr) {
        out->ranges[i] = std::numeric_limits<float>::infinity();
        if (!out->intensities.empty()) { out->intensities[i] = 0.0f; }
        ++removed;
      }
    }

    ++diag_frame_count_;
    diag_removed_accum_ += removed;
    diag_valid_accum_   += valid_total;
    if (diag_frame_count_ >= 50) {
      const float avg_rm  = static_cast<float>(diag_removed_accum_) /
                            static_cast<float>(diag_frame_count_);
      const float avg_vld = static_cast<float>(diag_valid_accum_) /
                            static_cast<float>(diag_frame_count_);
      const float pct = (avg_vld > 0.0f) ? (avg_rm / avg_vld * 100.0f) : 0.0f;
      RCLCPP_INFO(get_logger(),
        "[diag] removed %.1f/%.0f beams (%.1f%%) over last %d frames",
        avg_rm, avg_vld, pct, diag_frame_count_);
      diag_frame_count_ = 0; diag_removed_accum_ = 0; diag_valid_accum_ = 0;
    }

    scan_pub_->publish(*out);
  }

  // ── Build current reference from buffer ───────────────────────────────
  std::vector<float> buildReference(std::size_t n) const
  {
    std::vector<float> ref(n, std::numeric_limits<float>::infinity());

    if (reference_mode_ == "last") {
      const auto & prev = scan_buffer_[scan_buffer_.size() - 2];
      for (std::size_t i = 0; i < n; ++i) { ref[i] = prev[i]; }
      return ref;
    }

    // "median" over all buffer entries except the most recent
    const int buf_end = static_cast<int>(scan_buffer_.size()) - 1;
    if (buf_end <= 0) { return ref; }

    std::vector<float> vals;
    vals.reserve(static_cast<std::size_t>(buf_end));
    for (std::size_t i = 0; i < n; ++i) {
      vals.clear();
      for (int k = 0; k < buf_end; ++k) {
        const float v = scan_buffer_[static_cast<std::size_t>(k)][i];
        if (std::isfinite(v)) { vals.push_back(v); }
      }
      if (!vals.empty()) {
        const std::size_t mid = vals.size() / 2;
        std::nth_element(vals.begin(),
          vals.begin() + static_cast<std::ptrdiff_t>(mid), vals.end());
        ref[i] = vals[mid];
      }
    }
    return ref;
  }

  // ── Motion guard ──────────────────────────────────────────────────────
  bool isRobotMotionFrame(
    const sensor_msgs::msg::LaserScan & msg,
    const std::vector<float> & ref,
    std::size_t n,
    float & changed_frac_out) const
  {
    int changed = 0, valid = 0;
    const float f_epsd = static_cast<float>(eps_d_);
    const float f_epsa = static_cast<float>(eps_a_);
    for (std::size_t i = 0; i < n; ++i) {
      const float r = msg.ranges[i];
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) { continue; }
      if (!std::isfinite(ref[i])) { continue; }
      ++valid;
      if (std::abs(r - ref[i]) > f_epsd + f_epsa * r) { ++changed; }
    }
    if (valid == 0) { changed_frac_out = 0.0f; return false; }
    changed_frac_out = static_cast<float>(changed) / static_cast<float>(valid);
    return changed_frac_out > static_cast<float>(motion_guard_frac_);
  }

  // ── Guard diagnostics (print every 50 guard-evaluated frames) ─────────
  void printGuardDiag()
  {
    if (diag_guard_total_frames_ % 50 != 0) { return; }
    const float rate = static_cast<float>(diag_guard_triggered_) /
                       static_cast<float>(std::max(diag_guard_total_frames_, 1));
    const float avg_frac = (diag_guard_triggered_ > 0)
      ? diag_guard_frac_accum_ / static_cast<float>(diag_guard_triggered_) : 0.0f;
    RCLCPP_INFO(get_logger(),
      "[guard-diag] triggered %.0f%% of frames  avg_changed=%.1f%%  "
      "(threshold=%.0f%%)",
      rate * 100.0f, avg_frac * 100.0f, motion_guard_frac_ * 100.0f);
  }

  // ── Parameters ────────────────────────────────────────────────────────
  std::string input_topic_, output_topic_, reference_mode_, filter_method_;
  double prior_static_, prior_dynamic_;
  double eps_a_, eps_d_;
  double alpha_, beta_;
  double max_dyn_, dyn_threshold_;
  double c1_;
  double freeze_threshold_;
  double eps_;
  double motion_guard_frac_;
  int    buffer_size_;

  // ── Per-beam state ────────────────────────────────────────────────────
  std::vector<float> prob_static_;
  std::vector<float> prob_dynamic_;
  // Frozen reference: only updated for beams classified as static.
  // This prevents the reference from drifting toward a persistent obstacle.
  std::vector<float> frozen_ref_;

  // ── Rolling raw-scan buffer ───────────────────────────────────────────
  std::vector<std::vector<float>> scan_buffer_;

  // ── Diagnostics ───────────────────────────────────────────────────────
  int   diag_frame_count_        = 0;
  int   diag_removed_accum_      = 0;
  int   diag_valid_accum_        = 0;
  int   diag_guard_triggered_    = 0;
  int   diag_guard_total_frames_ = 0;
  float diag_guard_frac_accum_   = 0.0f;

  // ── ROS interfaces ────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
