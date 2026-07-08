//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Obstacles data = nlohmann::json::parse(jsonString);
#pragma once

#include <optional>
#include <nlohmann/json.hpp>

#ifndef NLOHMANN_OPT_HELPER
#define NLOHMANN_OPT_HELPER
namespace nlohmann {
    template <typename T>
    struct adl_serializer<std::shared_ptr<T>> {
        static void to_json(json & j, const std::shared_ptr<T> & opt) {
            if (!opt) j = nullptr; else j = *opt;
        }

        static std::shared_ptr<T> from_json(const json & j) {
            if (j.is_null()) return std::make_shared<T>(); else return std::make_shared<T>(j.get<T>());
        }
    };
    template <typename T>
    struct adl_serializer<std::optional<T>> {
        static void to_json(json & j, const std::optional<T> & opt) {
            if (!opt) j = nullptr; else j = *opt;
        }

        static std::optional<T> from_json(const json & j) {
            if (j.is_null()) return std::make_optional<T>(); else return std::make_optional<T>(j.get<T>());
        }
    };
}
#endif

namespace quicktype {
    using nlohmann::json;

    #ifndef NLOHMANN_UNTYPED_quicktype_HELPER
    #define NLOHMANN_UNTYPED_quicktype_HELPER
    inline json get_untyped(const json & j, const char * property) {
        if (j.find(property) != j.end()) {
            return j.at(property).get<json>();
        }
        return json();
    }

    inline json get_untyped(const json & j, std::string property) {
        return get_untyped(j, property.data());
    }
    #endif

    #ifndef NLOHMANN_OPTIONAL_quicktype_HELPER
    #define NLOHMANN_OPTIONAL_quicktype_HELPER
    template <typename T>
    inline std::shared_ptr<T> get_heap_optional(const json & j, const char * property) {
        auto it = j.find(property);
        if (it != j.end() && !it->is_null()) {
            return j.at(property).get<std::shared_ptr<T>>();
        }
        return std::shared_ptr<T>();
    }

    template <typename T>
    inline std::shared_ptr<T> get_heap_optional(const json & j, std::string property) {
        return get_heap_optional<T>(j, property.data());
    }
    template <typename T>
    inline std::optional<T> get_stack_optional(const json & j, const char * property) {
        auto it = j.find(property);
        if (it != j.end() && !it->is_null()) {
            return j.at(property).get<std::optional<T>>();
        }
        return std::optional<T>();
    }

    template <typename T>
    inline std::optional<T> get_stack_optional(const json & j, std::string property) {
        return get_stack_optional<T>(j, property.data());
    }
    #endif

    struct ObstaclePose {
        std::vector<double> orientation;
        std::vector<double> position;
    };

    struct ShapePose {
        std::optional<std::vector<double>> orientation;
        std::optional<std::vector<double>> position;
    };

    enum class Type : int { BOX, CYLINDER, SPHERE };

    struct ShapeElement {
        std::vector<double> dimensions;
        std::optional<ShapePose> pose;
        Type type;
    };

    struct ObstacleValue {
        ObstaclePose pose;
        std::vector<ShapeElement> shapes;
    };

    using Obstacles = std::map<std::string, ObstacleValue>;
}

namespace quicktype {
    void from_json(const json & j, ObstaclePose & x);
    void to_json(json & j, const ObstaclePose & x);

    void from_json(const json & j, ShapePose & x);
    void to_json(json & j, const ShapePose & x);

    void from_json(const json & j, ShapeElement & x);
    void to_json(json & j, const ShapeElement & x);

    void from_json(const json & j, ObstacleValue & x);
    void to_json(json & j, const ObstacleValue & x);

    void from_json(const json & j, Type & x);
    void to_json(json & j, const Type & x);

    inline void from_json(const json & j, ObstaclePose& x) {
        x.orientation = j.at("orientation").get<std::vector<double>>();
        x.position = j.at("position").get<std::vector<double>>();
    }

    inline void to_json(json & j, const ObstaclePose & x) {
        j = json::object();
        j["orientation"] = x.orientation;
        j["position"] = x.position;
    }

    inline void from_json(const json & j, ShapePose& x) {
        x.orientation = get_stack_optional<std::vector<double>>(j, "orientation");
        x.position = get_stack_optional<std::vector<double>>(j, "position");
    }

    inline void to_json(json & j, const ShapePose & x) {
        j = json::object();
        j["orientation"] = x.orientation;
        j["position"] = x.position;
    }

    inline void from_json(const json & j, ShapeElement& x) {
        x.dimensions = j.at("dimensions").get<std::vector<double>>();
        x.pose = get_stack_optional<ShapePose>(j, "pose");
        x.type = j.at("type").get<Type>();
    }

    inline void to_json(json & j, const ShapeElement & x) {
        j = json::object();
        j["dimensions"] = x.dimensions;
        j["pose"] = x.pose;
        j["type"] = x.type;
    }

    inline void from_json(const json & j, ObstacleValue& x) {
        x.pose = j.at("pose").get<ObstaclePose>();
        x.shapes = j.at("shapes").get<std::vector<ShapeElement>>();
    }

    inline void to_json(json & j, const ObstacleValue & x) {
        j = json::object();
        j["pose"] = x.pose;
        j["shapes"] = x.shapes;
    }

    inline void from_json(const json & j, Type & x) {
        if (j == "box") x = Type::BOX;
        else if (j == "cylinder") x = Type::CYLINDER;
        else if (j == "sphere") x = Type::SPHERE;
        else { throw std::runtime_error("Cannot deserialize to enumeration \"Type\""); }
    }

    inline void to_json(json & j, const Type & x) {
        switch (x) {
            case Type::BOX: j = "box"; break;
            case Type::CYLINDER: j = "cylinder"; break;
            case Type::SPHERE: j = "sphere"; break;
            default: throw std::runtime_error("Unexpected value in enumeration \"Type\": " + std::to_string(static_cast<int>(x)));
        }
    }
}


// JSON Schema
/**
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "ObstacleConfigSchema",
  "type": "object",
  "additionalProperties": {
    "type": "object",
    "required": ["pose", "shapes"],
    "properties": {
      "pose": {
        "$ref": "#/$defs/rigidPose"
      },
      "shapes": {
        "type": "array",
        "minItems": 1,
        "items": {
          "$ref": "#/$defs/shapeItem"
        }
      }
    }
  },
  "$defs": {
    "vector3": {
      "type": "array",
      "minItems": 3,
      "maxItems": 3,
      "items": {
        "type": "number"
      }
    },
    "rigidPose": {
      "type": "object",
      "required": ["position", "orientation"],
      "properties": {
        "position": {
          "$ref": "#/$defs/vector3"
        },
        "orientation": {
          "$ref": "#/$defs/vector3"
        }
      }
    },
    "shapePose": {
      "type": "object",
      "properties": {
        "position": {
          "$ref": "#/$defs/vector3"
        },
        "orientation": {
          "$ref": "#/$defs/vector3"
        }
      }
    },
    "shapeItem": {
      "type": "object",
      "required": ["type", "dimensions"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["sphere", "box", "cylinder"]
        },
        "dimensions": {
          "type": "array",
          "items": {
            "type": "number"
          }
        },
        "pose": {
          "$ref": "#/$defs/shapePose"
        }
      },
      "allOf": [
        {
          "if": {
            "properties": {
              "type": {
                "const": "sphere"
              }
            }
          },
          "then": {
            "properties": {
              "dimensions": {
                "minItems": 1,
                "maxItems": 1
              }
            }
          }
        },
        {
          "if": {
            "properties": {
              "type": {
                "const": "cylinder"
              }
            }
          },
          "then": {
            "properties": {
              "dimensions": {
                "minItems": 2,
                "maxItems": 2
              }
            }
          }
        },
        {
          "if": {
            "properties": {
              "type": {
                "const": "box"
              }
            }
          },
          "then": {
            "properties": {
              "dimensions": {
                "minItems": 3,
                "maxItems": 3
              }
            }
          }
        }
      ]
    }
  }
}
**/
