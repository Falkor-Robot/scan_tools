#include <laser_scan_matcher/laser_scan_matcher_diagnostics.hpp>
#include <laser_scan_matcher/laser_scan_matcher.hpp>

namespace scan_tools{
LaserScanMatcherDiagnostics::LaserScanMatcherDiagnostics(LaserScanMatcher * scanner)
{
  diagnostic_ = std::make_shared<diagnostic_updater::Updater>(scanner);
  status_ = std::make_shared<status_type>();

  diagnostic_->add("LaserScanMatcher mux status", this, &LaserScanMatcherDiagnostics::diagnostics);
  diagnostic_->setHardwareID("none");
}

void LaserScanMatcherDiagnostics::update()
{
  diagnostic_->force_update();
}

void LaserScanMatcherDiagnostics::updateStatus(const status_type::ConstPtr & status)
{
  status_->match_duration = status->match_duration;
  status_->scan_valid = status->scan_valid;
  //status_->priority = status->priority;

  status_->main_loop_time = status->main_loop_time;
  status_->reading_age = status->reading_age;

  update();
}

void LaserScanMatcherDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  /// Check if the loop period is quick enough
  if (status_->main_loop_time > MAIN_LOOP_TIME_MIN) {
    stat.summary(ERROR, "loop time too long");
  } else if (status_->reading_age > READING_AGE_MIN) {
    stat.summary(ERROR, "data received is too old");
  } else if (status_->match_duration > MATCH_DURATION) {
    stat.summary(WARN, "match duration is slow");
  } else {
    stat.summary(OK, "ok");
  }
/*
  for (auto & velocity_h : *status_->velocity_hs) {
    stat.addf(
      "velocity " + velocity_h.getName(), " %s (listening to %s @ %fs with priority #%d)",
      (velocity_h.isMasked(status_->priority) ? "masked" : "unmasked"),
      velocity_h.getTopic().c_str(),
      velocity_h.getTimeout().seconds(), static_cast<int>(velocity_h.getPriority()));
  }

  for (const auto & lock_h : *status_->lock_hs) {
    stat.addf(
      "lock " + lock_h.getName(), " %s (listening to %s @ %fs with priority #%d)",
      (lock_h.isLocked() ? "locked" : "free"), lock_h.getTopic().c_str(),
      lock_h.getTimeout().seconds(),
      static_cast<int>(lock_h.getPriority()));
  }
*/
  stat.add("current scan status", static_cast<bool>(status_->scan_valid));

  stat.add("loop time in [msec]", status_->main_loop_time);
  stat.add("match duration [msec]", status_->match_duration);
  stat.add("data age in [sec]", status_->reading_age);
}

}  // scan_tools