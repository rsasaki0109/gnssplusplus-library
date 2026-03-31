#pragma once

#include "types.hpp"
#include <string>
#include <vector>

namespace libgnss {
namespace kml {

/**
 * @brief Write a complete KML file.
 * @param filepath Output file path (.kml)
 * @param content KML content (elements, tours, etc.)
 * @param name Document name (defaults to filename)
 * @return true on success
 */
bool writeKml(const std::string& filepath,
              const KmlString& content,
              const std::string& name = "");

/**
 * @brief Write a KMZ (compressed KML) file.
 *
 * Creates a temporary .kml, compresses it to .kmz using system zip,
 * and removes the temporary file.
 *
 * @param filepath Output file path (.kmz)
 * @param content KML content
 * @param name Document name
 * @param extra_files Additional files to include (e.g., DAE models, images)
 * @return true on success
 */
bool writeKmz(const std::string& filepath,
              const KmlString& content,
              const std::string& name = "",
              const std::vector<std::string>& extra_files = {});

}  // namespace kml
}  // namespace libgnss
