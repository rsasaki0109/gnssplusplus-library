#pragma once

#include "types.hpp"
#include <string>

namespace libgnss {
namespace kml {

/**
 * @brief Wrap KML content in a gx:Tour tag.
 */
KmlString wrapTour(const KmlString& content, const std::string& name);

/**
 * @brief Wrap KML content in a gx:FlyTo tag.
 */
KmlString wrapFlyTo(const KmlString& content,
                    double duration = 0.0,
                    FlyToMode mode = FlyToMode::Smooth);

/**
 * @brief Wrap KML content in a Folder tag.
 */
KmlString wrapFolder(const KmlString& content, const std::string& name);

/**
 * @brief Create gx:Wait element.
 */
KmlString createWait(double duration);

}  // namespace kml
}  // namespace libgnss
