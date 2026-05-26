#pragma once

#include "yolov8-pose.h"
#include <string>
#include <vector>

struct BehaviorReport {
    int people_count;
    bool crouching;
    float crouching_conf;
    bool violence_alert;
    float violence_conf;
    bool crowd_alert;
    int crowd_threshold;
    std::vector<std::string> messages;
};

BehaviorReport analyze_behavior(const object_detect_result_list &results,
                                int crowd_threshold = 4);
