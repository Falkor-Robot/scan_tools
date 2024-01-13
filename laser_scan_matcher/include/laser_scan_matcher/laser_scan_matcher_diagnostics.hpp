#ifndef SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_HPP_
#define SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_HPP_

#include <laser_scan_matcher/laser_scan_matcher_diagnostics_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>

namespace scan_tools
{
  class LaserScanMatcher;

class LaserScanMatcherDiagnostics
{
public:
  typedef LaserScanMatcherDiagnosticsStatus status_type;

  static constexpr double MAIN_LOOP_TIME_MIN = 0.2;   // [s]
  static constexpr double READING_AGE_MIN = 3.0;     // [s]
  static constexpr double MATCH_DURATION = 30;   // [ms]

  explicit LaserScanMatcherDiagnostics(LaserScanMatcher * scanner);
  virtual ~LaserScanMatcherDiagnostics() = default;

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void update();

  void updateStatus(const status_type::ConstPtr & status);

private:
  /**
   * @brief Levels
   */
  enum
  {
    OK = diagnostic_msgs::msg::DiagnosticStatus::OK,
    WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN,
    ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR
  };

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_;
  std::shared_ptr<status_type> status_;
};
}  // scan_tools
#endif  //  SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_HPP_