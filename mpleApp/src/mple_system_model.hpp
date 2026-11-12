#pragma once
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/geometry.hpp>

struct MpleSystemModel {
    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    pinocchio::FrameIndex base_frame_id;
    pinocchio::FrameIndex ee_frame_id;
    std::size_t state_dim;
};

