#pragma once
#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <stdexcept>

namespace libgnss {
// Read-only paired residual statistics. No inference weights estimated.
// Only link rows with identical stream keys and exactly shared endpoints.
template<class Key> class AdjacentResidualMoments {
    struct Last { std::size_t current; double residual; };
    std::map<Key,Last> last_;
    std::size_t n_=0;
    double mx_=0.,my_=0.,xx_=0.,yy_=0.,xy_=0.;
public:
    void breakStream(const Key& key) { last_.erase(key); }
    void add(const Key& key,std::size_t previous,std::size_t current,double residual) {
        if(previous>=current) throw std::invalid_argument("invalid residual endpoints");
        if(!std::isfinite(residual)) { breakStream(key); return; }
        const auto found=last_.find(key);
        if(found!=last_.end() && found->second.current==previous) {
            const double x=found->second.residual,y=residual;
            ++n_;
            const double dx=x-mx_,dy=y-my_;
            mx_+=dx/double(n_); my_+=dy/double(n_);
            xx_+=dx*(x-mx_); yy_+=dy*(y-my_); xy_+=dx*(y-my_);
        }
        last_[key]={current,residual};
    }
    std::size_t pairs() const { return n_; }
    std::optional<double> correlation() const {
        if(n_<2 || xx_<=0. || yy_<=0.) return std::nullopt;
        const double value=xy_/std::sqrt(xx_)/std::sqrt(yy_);
        return std::isfinite(value)?std::optional<double>(value):std::nullopt;
    }
};
}
