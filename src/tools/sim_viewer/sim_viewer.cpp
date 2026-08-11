// Standalone viewer for the simulator link.
//
// The detector's own debug windows are created from its pop thread and never
// appeared on screen here, which made it impossible to see what it was
// actually working with.  This subscribes to the same two topics and draws
// them itself from the main thread, so the window is under our control and
// tells us both what the camera is publishing and what the detector made of
// it.
//
// Layout: the camera frame, with each published armour drawn as its two light
// bars plus a connecting quad, and a HUD line counting frames and armours.

#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"

#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>
#include <iceoryx_posh/capro/service_description.hpp>
#include <thread>
#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdio>
#include <deque>
#include <string>
#include <algorithm>

namespace {
cv::Point toPoint(const msgs::Point2d &p) {
  return cv::Point(static_cast<int>(p.x), static_cast<int>(p.y));
}
}  // namespace

int main(int argc, char **argv) {
  const std::string camera = argc > 1 ? argv[1] : "camera0";
  const bool save_only = argc > 2 && std::string(argv[2]) == "--save";
  const std::string save_path =
      argc > 3 ? argv[3] : "/out/sim_viewer.png";

  // iceoryx runtime names must be unique, so a --save grab can run alongside
  // an already-open viewer window.
  iox::runtime::PoshRuntime::initRuntime(
      save_only ? iox::RuntimeName_t{"sim_viewer_grab"}
                : iox::RuntimeName_t{"sim_viewer"});
  iox::popo::Subscriber<msgs::Image1440x1080_8UC3, msgs::Header> image_sub(
      {"image_raw", {iox::TruncateToCapacity, camera.c_str()}, "data"});
  iox::popo::Subscriber<msgs::Armor, msgs::Header> armor_sub(
      {"armors", "detector", "data"});

  if (!save_only) {
    cv::namedWindow("sim_viewer", cv::WINDOW_NORMAL);
    cv::resizeWindow("sim_viewer", 1280, 960);
  }

  cv::Mat frame(1080, 1440, CV_8UC3, cv::Scalar::all(0));
  std::deque<msgs::Armor> armours;
  long frames = 0, armour_msgs = 0, heartbeats = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(save_only ? 25 : 3600);

  while (!iox::hasTerminationRequested() &&
         std::chrono::steady_clock::now() < deadline) {
    bool fresh = false;
    while (image_sub.take().and_then([&](auto &sample) {
      std::memcpy(frame.data, sample->data, sizeof(sample->data));
      ++frames;
      fresh = true;
    })) {
    }
    // Only what arrived since the last frame gets drawn.  Keeping a backlog
    // and redrawing it every frame smears stale boxes across the image as soon
    // as the robot or gimbal moves, which reads as the detector being wrong
    // when it is not.
    bool got_armour = false;
    while (armor_sub.take().and_then([&](auto &sample) {
      if (sample->heart_beat) {
        ++heartbeats;
        return;
      }
      ++armour_msgs;
      if (!got_armour) {
        armours.clear();
        got_armour = true;
      }
      armours.push_back(*sample);
    })) {
    }

    if (fresh) {
      cv::Mat canvas = frame.clone();
      for (const auto &armour : armours) {
        const cv::Scalar colour = armour.armor_color == 0
                                      ? cv::Scalar(0, 0, 255)
                                      : cv::Scalar(255, 128, 0);
        const cv::Point lt = toPoint(armour.left_light.top);
        const cv::Point lb = toPoint(armour.left_light.bottom);
        const cv::Point rt = toPoint(armour.right_light.top);
        const cv::Point rb = toPoint(armour.right_light.bottom);
        // Same marks the detector draws itself (DetectorNode::drawArmor):
        // each bar as a line with its endpoints ringed, and the armour as the
        // two diagonals.  A closed quad through the four endpoints looks
        // skewed whenever the bars differ in height or tilt, which reads as a
        // bad detection when the detection is fine.
        cv::circle(canvas, lt, 3, cv::Scalar(0, 255, 0), 1);
        cv::circle(canvas, lb, 3, cv::Scalar(0, 255, 0), 1);
        cv::circle(canvas, rt, 3, cv::Scalar(0, 255, 0), 1);
        cv::circle(canvas, rb, 3, cv::Scalar(0, 255, 0), 1);
        cv::line(canvas, lt, lb, colour, 2);
        cv::line(canvas, rt, rb, colour, 2);
        cv::line(canvas, lt, rb, cv::Scalar(0, 255, 0), 2);
        cv::line(canvas, lb, rt, cv::Scalar(0, 255, 0), 2);
        char label[96];
        std::snprintf(label, sizeof(label), "type=%d colour=%d conf=%.2f d=%.2fm",
                      armour.armor_type, armour.armor_color, armour.confidence,
                      armour.position.z);
        cv::putText(canvas, label, cv::Point(std::min(lt.x, rt.x),
                                            std::min(lt.y, rt.y) - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      }
      char hud[160];
      std::snprintf(hud, sizeof(hud),
                    "frames=%ld  armours=%ld  heartbeats=%ld  drawn=%zu",
                    frames, armour_msgs, heartbeats, armours.size());
      cv::putText(canvas, hud, cv::Point(12, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 255, 255), 2);

      if (save_only) {
        if (frames > 12) {
          cv::imwrite(save_path, canvas);
          std::printf("saved %s  %s\n", save_path.c_str(), hud);
          return 0;
        }
      } else {
        cv::imshow("sim_viewer", canvas);
        if (cv::waitKey(1) == 27)
          break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::printf("frames=%ld armours=%ld heartbeats=%ld\n", frames, armour_msgs,
              heartbeats);
  return 0;
}
