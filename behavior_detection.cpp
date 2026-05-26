#include "behavior_detection.h"
#include <cmath>
#include <cstring>

// ── 辅助：两个框的重叠率 (IOU) ──
static float box_iou(const image_rect_t &a, const image_rect_t &b)
{
    int inter_left   = std::max(a.left,   b.left);
    int inter_top    = std::max(a.top,    b.top);
    int inter_right  = std::min(a.right,  b.right);
    int inter_bottom = std::min(a.bottom, b.bottom);
    int inter_w = std::max(0, inter_right - inter_left);
    int inter_h = std::max(0, inter_bottom - inter_top);
    float inter = (float)inter_w * inter_h;
    float area_a = (float)(a.right - a.left) * (a.bottom - a.top);
    float area_b = (float)(b.right - b.left) * (b.bottom - b.top);
    float union_area = area_a + area_b - inter;
    return (union_area > 0) ? inter / union_area : 0;
}

// ── 检测蹲伏 ──
//  关键点: 5=左肩,6=右肩, 11=左髋,12=右髋, 13=左膝,14=右膝, 15=左踝,16=右踝
static float detect_crouching(const float keypoints[17][3])
{
    float conf[8] = {
        keypoints[5][2],  keypoints[6][2],
        keypoints[11][2], keypoints[12][2],
        keypoints[13][2], keypoints[14][2],
        keypoints[15][2], keypoints[16][2],
    };
    float avg_conf = 0;
    for (int i = 0; i < 8; i++) avg_conf += conf[i];
    avg_conf /= 8;
    if (avg_conf < 0.3f) return 0;   // 关键点置信度太低，跳过

    float shoulder_y = (keypoints[5][1] + keypoints[6][1]) / 2;
    float hip_y      = (keypoints[11][1] + keypoints[12][1]) / 2;
    float knee_y     = (keypoints[13][1] + keypoints[14][1]) / 2;
    float ankle_y    = (keypoints[15][1] + keypoints[16][1]) / 2;

    float body_h  = ankle_y - shoulder_y;   // 总身高
    float hip2ank = ankle_y - hip_y;         // 髋到踝
    float knee2ank = ankle_y - knee_y;       // 膝到踝

    if (body_h < 1) return 0;

    // 蹲伏特征：
    //   1. 髋到踝 / 总身高 偏小（腿没伸直）
    //   2. 膝到踝 / 髋到踝 也偏小
    float r1 = hip2ank / body_h;
    float r2 = knee2ank / hip2ank;

    // 站立: r1≈0.5~0.6, r2≈0.5~0.6
    // 蹲伏: r1<0.4,       r2<0.4
    float score = 0;
    if (r1 < 0.45f) score += 0.6f;
    if (r2 < 0.40f) score += 0.4f;

    return std::min(score, 1.0f);
}

// ── 检测暴力倾向（单帧启发式）──
//  1. 两个人框高度重叠 → 冲突嫌疑
//  2. 一个人双臂举过头顶或大张开 → 异常姿势
static float detect_violence_frame(const object_detect_result_list &results)
{
    float score = 0;

    // 1. 框重叠检查
    for (int i = 0; i < results.count; i++) {
        for (int j = i + 1; j < results.count; j++) {
            float iou = box_iou(results.results[i].box, results.results[j].box);
            if (iou > 0.3f) {
                score = std::max(score, 0.5f + iou * 0.3f);
            }
        }
    }

    // 2. 异常姿势（手臂高举或伸展）
    for (int i = 0; i < results.count; i++) {
        const auto &kp = results.results[i].keypoints;

        // 左右肩 (5,6) 和 左右腕 (9,10)
        float sh_y = (kp[5][1] + kp[6][1]) / 2;
        float sh_x = (kp[5][0] + kp[6][0]) / 2;

        float wrist_y = (kp[9][1] + kp[10][1]) / 2;
        float wrist_x = (kp[9][0] + kp[10][0]) / 2;

        float sh_wrist_dy = sh_y - wrist_y;  // 正 = 手腕在肩膀上方
        float sh_wrist_dx = std::abs(wrist_x - sh_x);

        float bio_w = (float)(results.results[i].box.right - results.results[i].box.left);
        float bio_h = (float)(results.results[i].box.bottom - results.results[i].box.top);
        float diag = std::sqrt(bio_w * bio_w + bio_h * bio_h);

        if (diag < 1) continue;

        // 手腕明显高于肩膀 → 举手
        if (sh_wrist_dy > bio_h * 0.2f) {
            score = std::max(score, 0.6f);
        }
        // 手腕横向距离很大（手臂张开）
        if (sh_wrist_dx > bio_w * 1.2f) {
            score = std::max(score, 0.5f);
        }
    }

    return std::min(score, 1.0f);
}

// ── 主入口 ──
BehaviorReport analyze_behavior(const object_detect_result_list &results,
                                int crowd_threshold)
{
    BehaviorReport r;
    memset(&r, 0, sizeof(r));
    r.crowd_threshold = crowd_threshold;
    r.people_count = results.count;

    // 聚集
    if (results.count >= crowd_threshold) {
        r.crowd_alert = true;
        r.messages.push_back("CROWD (" + std::to_string(results.count) + " people)");
    }

    for (int i = 0; i < results.count; i++) {
        // 蹲伏
        float c = detect_crouching(results.results[i].keypoints);
        if (c > r.crouching_conf) {
            r.crouching_conf = c;
            r.crouching = c > 0.5f;
        }
    }

    // 暴力
    float v = detect_violence_frame(results);
    r.violence_conf = v;
    r.violence_alert = v > 0.4f;

    if (r.crouching)
        r.messages.push_back("CROUCH");
    if (r.violence_alert)
        r.messages.push_back("VIOLENCE");

    return r;
}
