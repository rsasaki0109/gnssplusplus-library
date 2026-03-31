#include "libgnss++/visibility/fisheye_camera.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <regex>
#include <algorithm>

namespace libgnss {
namespace visibility {

namespace {

// Simple XML tag value extractor (no full XML parser dependency)
std::string extractTag(const std::string& xml, const std::string& tag) {
    std::string open = "<" + tag + ">";
    std::string close = "</" + tag + ">";
    auto p1 = xml.find(open);
    if (p1 == std::string::npos) return "";
    p1 += open.size();
    auto p2 = xml.find(close, p1);
    if (p2 == std::string::npos) return "";
    std::string val = xml.substr(p1, p2 - p1);
    // Strip comments <!-- ... -->
    std::regex comment_re("<!--.*?-->");
    val = std::regex_replace(val, comment_re, "");
    // Trim whitespace
    auto start = val.find_first_not_of(" \t\n\r");
    auto end = val.find_last_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    return val.substr(start, end - start + 1);
}

std::vector<double> parseDoubleList(const std::string& s) {
    std::vector<double> result;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        auto start = token.find_first_not_of(" \t");
        if (start != std::string::npos) {
            result.push_back(std::stod(token.substr(start)));
        }
    }
    return result;
}

}  // namespace

bool FisheyeCamera::loadFromXml(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) return false;

    std::string xml((std::istreambuf_iterator<char>(ifs)),
                    std::istreambuf_iterator<char>());

    auto w = extractTag(xml, "width");
    auto h = extractTag(xml, "hight");  // sic: original XML uses "hight"
    auto x = extractTag(xml, "xc");
    auto y = extractTag(xml, "yc");
    auto r_str = extractTag(xml, "r");
    auto el_str = extractTag(xml, "el");
    auto deg = extractTag(xml, "degree");

    if (w.empty() || h.empty() || x.empty() || y.empty() ||
        r_str.empty() || el_str.empty() || deg.empty()) return false;

    width = std::stoi(w);
    height = std::stoi(h);
    xc = std::stod(x);
    yc = std::stod(y);
    r_calib = parseDoubleList(r_str);
    el_calib = parseDoubleList(el_str);
    poly_degree = std::stoi(deg);
    poly_dirty_ = true;

    return true;
}

std::vector<double> FisheyeCamera::fitPolynomial() const {
    if (!poly_dirty_) return poly_cache_;

    // For degree 1 (linear): r = a * el + b
    // Fits using least squares via normal equations.
    // General case supports arbitrary degree but the original uses degree=1.
    int n = static_cast<int>(el_calib.size());
    int d = poly_degree;

    if (n < d + 1) {
        return {};
    }

    // Build Vandermonde matrix and solve normal equations
    // For degree 1: [el 1] * [a; b] = r
    // Using simple approach with Eigen-free implementation
    if (d == 1 && n == 2) {
        // Direct solution for 2 points, degree 1
        double el0 = el_calib[0], el1 = el_calib[1];
        double r0 = r_calib[0], r1 = r_calib[1];
        if (std::abs(el1 - el0) < 1e-15) return {};
        double a = (r1 - r0) / (el1 - el0);
        double b = r0 - a * el0;
        poly_cache_ = {a, b};
        poly_dirty_ = false;
        return poly_cache_;
    }

    // General least-squares polyfit (up to degree ~5, no Eigen needed)
    // Build (d+1)x(d+1) system: (V^T V) c = V^T r
    int m = d + 1;
    std::vector<std::vector<double>> A(m, std::vector<double>(m, 0.0));
    std::vector<double> b(m, 0.0);

    for (int i = 0; i < n; ++i) {
        double x = el_calib[i];
        double y = r_calib[i];
        for (int j = 0; j < m; ++j) {
            for (int k = 0; k < m; ++k) {
                // power: (d-j) + (d-k) = 2d - j - k
                A[j][k] += std::pow(x, 2 * d - j - k);
            }
            b[j] += y * std::pow(x, d - j);
        }
    }

    // Gaussian elimination with partial pivoting
    std::vector<double> c(m, 0.0);
    for (int col = 0; col < m; ++col) {
        // Pivot
        int max_row = col;
        for (int row = col + 1; row < m; ++row) {
            if (std::abs(A[row][col]) > std::abs(A[max_row][col])) {
                max_row = row;
            }
        }
        std::swap(A[col], A[max_row]);
        std::swap(b[col], b[max_row]);

        if (std::abs(A[col][col]) < 1e-15) return {};

        for (int row = col + 1; row < m; ++row) {
            double factor = A[row][col] / A[col][col];
            for (int k = col; k < m; ++k) {
                A[row][k] -= factor * A[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    for (int row = m - 1; row >= 0; --row) {
        c[row] = b[row];
        for (int k = row + 1; k < m; ++k) {
            c[row] -= A[row][k] * c[k];
        }
        c[row] /= A[row][row];
    }

    poly_cache_ = c;
    poly_dirty_ = false;
    return poly_cache_;
}

}  // namespace visibility
}  // namespace libgnss
