#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <epicsUnitTest.h>
#include <testMain.h>

#include "planning_scene.hpp"

namespace {

struct ShapeSpec {
    std::string type;
    std::vector<double> position;
    std::vector<double> orientation;
    std::vector<double> dimensions;
};

struct ObjectSpec {
    std::string name;
    std::vector<double> position;
    std::vector<double> orientation;
    std::vector<ShapeSpec> shapes;
};

pvxs::shared_array<const double> to_array(const std::vector<double>& v) {
    pvxs::shared_array<double> arr(v.begin(), v.end());
    return arr.freeze();
}

pvxs::Value build(const std::vector<ObjectSpec>& specs) {
    auto top = scene_objects_pva_type().create();
    pvxs::shared_array<pvxs::Value> objects(specs.size());
    for (size_t i = 0; i < specs.size(); ++i) {
        const auto& spec = specs[i];
        auto& obj = objects[i];
        obj = top["objects"].allocMember();
        obj["name"] = spec.name;
        obj["position"] = to_array(spec.position);
        obj["orientation"] = to_array(spec.orientation);

        pvxs::shared_array<pvxs::Value> shapes(spec.shapes.size());
        for (size_t j = 0; j < spec.shapes.size(); ++j) {
            const auto& shape_spec = spec.shapes[j];
            auto& shape = shapes[j];
            shape = obj["shapes"].allocMember();
            shape["type"] = shape_spec.type;
            shape["position"] = to_array(shape_spec.position);
            shape["orientation"] = to_array(shape_spec.orientation);
            shape["dimensions"] = to_array(shape_spec.dimensions);
        }
        obj["shapes"] = shapes.freeze();
    }
    top["objects"] = objects.freeze();
    return top;
}

std::vector<ObjectSpec> valid_objects() {
    return {
        {"obj1",
         {1, 2, 3},
         {0, 0, 0},
         {
             {"box", {0, 0, 0}, {0, 0, 0}, {1, 2, 3}},
             {"sphere", {1, 0, 0}, {0, 0, 0}, {0.5}},
         }},
        {"obj2",
         {-1, 0, 0},
         {0, 0, 1},
         {
             {"cylinder", {0, 0, 0}, {0, 0, 0}, {0.25, 1.0}},
         }},
    };
}

template <typename F>
void expect_invalid(const char* desc, F modify) {
    auto specs = valid_objects();
    modify(specs);
    try {
        PlanningScene::from_pvxs_value(build(specs));
        testFail("%s: no exception thrown", desc);
    } catch (const std::invalid_argument& e) {
        testPass("%s: %s", desc, e.what());
    } catch (const std::exception& e) {
        testFail("%s: wrong exception type: %s", desc, e.what());
    }
}

bool same_pose(const Pose& a, const Pose& b) {
    return a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z &&
           a.orientation.rx == b.orientation.rx && a.orientation.ry == b.orientation.ry &&
           a.orientation.rz == b.orientation.rz;
}

bool same_primitive(const ScenePrimitive& a, const ScenePrimitive& b) {
    if (const auto* pa = std::get_if<Box>(&a)) {
        const auto* pb = std::get_if<Box>(&b);
        return pb && pa->width == pb->width && pa->height == pb->height && pa->depth == pb->depth;
    }
    if (const auto* pa = std::get_if<Sphere>(&a)) {
        const auto* pb = std::get_if<Sphere>(&b);
        return pb && pa->radius == pb->radius;
    }
    if (const auto* pa = std::get_if<Cylinder>(&a)) {
        const auto* pb = std::get_if<Cylinder>(&b);
        return pb && pa->radius == pb->radius && pa->height == pb->height;
    }
    return false;
}

void test_round_trip() {
    PlanningScene scene;
    scene.add_object(SceneObject{
        "multi",
        Pose{{1.5, -2.5, 0.25}, {0.1, 0.2, 0.3}},
        {
            SceneShape{Box{1, 2, 3}, Pose{{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}}},
            SceneShape{Sphere{0.5}, Pose{{-1, 0, 1}, {0, 0, M_PI / 3.0}}},
            SceneShape{Cylinder{0.25, 1.5}, Pose{{0, 2, 0}, {M_PI / 2.0, 0, 0}}},
        },
    });

    const auto value = scene.to_pvxs_value(7);
    testOk(value["revision"].as<uint64_t>() == 7, "revision serialized");

    const auto result = PlanningScene::from_pvxs_value(value);
    testOk(result.objects.size() == 1, "round trip object count");

    const auto& in = scene.objects.at("multi");
    const auto it = result.objects.find("multi");
    if (it == result.objects.end()) {
        testFail("round trip object found");
        testSkip(5, "object missing");
        return;
    }
    const auto& out = it->second;
    testOk(same_pose(in.pose, out.pose), "round trip object pose");
    testOk(out.shapes.size() == 3, "round trip shape count");
    if (out.shapes.size() != 3) {
        testSkip(3, "shape count mismatch");
        return;
    }
    for (size_t i = 0; i < 3; ++i) {
        testOk(same_primitive(in.shapes[i].primitive, out.shapes[i].primitive) &&
                   same_pose(in.shapes[i].pose, out.shapes[i].pose),
               "round trip shape %zu", i);
    }
}

void test_empty_scene() {
    PlanningScene scene;
    const auto value = scene.to_pvxs_value(0);
    testOk(value["objects"].as<pvxs::shared_array<const pvxs::Value>>().empty(), "empty scene serializes");
    try {
        const auto result = PlanningScene::from_pvxs_value(value);
        testOk(result.objects.empty(), "empty scene parses");
    } catch (const std::exception& e) {
        testFail("empty scene parses: %s", e.what());
    }
}

void test_valid_scene() {
    try {
        const auto result = PlanningScene::from_pvxs_value(build(valid_objects()));
        testOk(result.objects.size() == 2, "valid scene parses");
    } catch (const std::exception& e) {
        testFail("valid scene parses: %s", e.what());
    }
}

void test_invalid_scenes() {
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();

    expect_invalid("empty name", [](auto& s) { s[0].name = ""; });
    expect_invalid("duplicate name", [](auto& s) { s[1].name = s[0].name; });
    expect_invalid("no shapes", [](auto& s) { s[0].shapes.clear(); });
    expect_invalid("object position length", [](auto& s) { s[0].position = {1, 2}; });
    expect_invalid("object orientation length", [](auto& s) { s[0].orientation = {0, 0, 0, 0}; });
    expect_invalid("shape position length", [](auto& s) { s[0].shapes[0].position = {0, 0}; });
    expect_invalid("shape orientation empty", [](auto& s) { s[0].shapes[0].orientation = {}; });
    expect_invalid("sphere dimension count", [](auto& s) { s[0].shapes[1].dimensions = {0.5, 0.5}; });
    expect_invalid("box dimension count", [](auto& s) { s[0].shapes[0].dimensions = {1, 2}; });
    expect_invalid("cylinder dimension count", [](auto& s) { s[1].shapes[0].dimensions = {1, 2, 3}; });
    expect_invalid("unknown shape type", [](auto& s) { s[0].shapes[0].type = "cone"; });
    expect_invalid("zero dimension", [](auto& s) { s[0].shapes[0].dimensions = {1, 0, 3}; });
    expect_invalid("negative dimension", [](auto& s) { s[0].shapes[1].dimensions = {-0.5}; });
    expect_invalid("NaN object position", [nan](auto& s) { s[0].position = {nan, 0, 0}; });
    expect_invalid("inf shape dimension", [inf](auto& s) { s[1].shapes[0].dimensions = {inf, 1}; });
    expect_invalid("inf shape orientation", [inf](auto& s) { s[0].shapes[0].orientation = {0, -inf, 0}; });
}

} // namespace

MAIN(testScenePva) {
    testPlan(26);
    test_round_trip();
    test_empty_scene();
    test_valid_scene();
    test_invalid_scenes();
    return testDone();
}
