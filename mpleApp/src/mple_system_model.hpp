#pragma once
#include <mutex>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

struct MpleSystemModel {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex base_frame_id;
    pinocchio::FrameIndex ee_frame_id;
    std::size_t state_dim;
    std::mutex mutex;
};

