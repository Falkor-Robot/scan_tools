#ifndef SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_STATUS_HPP_
#define SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace scan_tools
{

struct LaserScanMatcherDiagnosticsStatus {
    typedef std::shared_ptr<LaserScanMatcherDiagnosticsStatus> Ptr;
    typedef std::shared_ptr<const LaserScanMatcherDiagnosticsStatus> ConstPtr;

    double match_duration;
    rclcpp::Time last_loop_update;
    double main_loop_time;
    bool scan_valid;
    double reading_age;

    LaserScanMatcherDiagnosticsStatus()
    : match_duration(0),
        last_loop_update(rclcpp::Clock().now()),
        main_loop_time(0),
        scan_valid(false),
        reading_age(0)
    {}
};

typedef LaserScanMatcherDiagnosticsStatus::Ptr LaserScanMatcherDiagnosticsStatusPtr;
typedef LaserScanMatcherDiagnosticsStatus::ConstPtr LaserScanMatcherDiagnosticsStatusConstPtr;

}  // scan_tools
#endif  //  SCAN_TOOLS__LASER_SCAN_MATCHER_DIAGNOSTICS_STATUS_HPP_