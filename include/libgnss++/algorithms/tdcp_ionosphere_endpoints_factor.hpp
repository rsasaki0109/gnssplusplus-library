#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace libgnss::code_ionosphere {
// For original TDCP residual prediction-minus-measurement, add
// +a_previous*I_previous - a_current*I_current. I is POST-model vertical
// residual, not total ionosphere. Builder owns epoch identity and priors.
class TdcpEndpointsFactor final : public gtsam::NoiseModelFactor {
    std::shared_ptr<const gtsam::NoiseModelFactor> original_;
    gtsam::Key previous_, current_;
    double a_previous_, a_current_;
    static gtsam::KeyVector keysFor(const std::shared_ptr<const gtsam::NoiseModelFactor>& f,
                                   gtsam::Key p, gtsam::Key c, double ap, double ac) {
        if (!f || !f->noiseModel() || f->dim()!=1 || p==c ||
            !std::isfinite(ap) || !std::isfinite(ac) || ap<=0 || ac<=0)
            throw std::invalid_argument("Invalid TDCP ionosphere endpoints");
        auto keys=f->keys();
        for (const auto k:{p,c}) {
            if (std::find(keys.begin(),keys.end(),k)!=keys.end())
                throw std::invalid_argument("TDCP ionosphere key collision");
            keys.push_back(k);
        }
        return keys;
    }
public:
    TdcpEndpointsFactor(std::shared_ptr<const gtsam::NoiseModelFactor> f,
                        gtsam::Key p,gtsam::Key c,double ap,double ac)
        : gtsam::NoiseModelFactor(f?f->noiseModel():nullptr,keysFor(f,p,c,ap,ac)),
          original_(std::move(f)),previous_(p),current_(c),a_previous_(ap),a_current_(ac) {}
    bool active(const gtsam::Values& x) const override { return original_->active(x); }
    using gtsam::NoiseModelFactor::unwhitenedError;
    gtsam::Vector unwhitenedError(const gtsam::Values& x,
                                  gtsam::OptionalMatrixVecType h=nullptr) const override {
        std::vector<gtsam::Matrix> base_h(original_->size());
        auto r=original_->unwhitenedError(x,h?&base_h:nullptr);
        const double p=x.at<double>(previous_),c=x.at<double>(current_);
        if (r.size()!=1 || !r.allFinite() || !std::isfinite(p) || !std::isfinite(c))
            throw std::invalid_argument("Invalid TDCP ionosphere state");
        r[0]+=a_previous_*p-a_current_*c;
        if (!r.allFinite()) throw std::invalid_argument("TDCP ionosphere overflow");
        if(h) {
            *h=std::move(base_h);
            h->push_back(gtsam::Matrix::Constant(1,1,a_previous_));
            h->push_back(gtsam::Matrix::Constant(1,1,-a_current_));
        }
        return r;
    }
    bool equals(const gtsam::NonlinearFactor& other,double tol=1e-9) const override {
        const auto* f=dynamic_cast<const TdcpEndpointsFactor*>(&other);
        return f && previous_==f->previous_ && current_==f->current_ &&
            std::abs(a_previous_-f->a_previous_)<=tol && std::abs(a_current_-f->a_current_)<=tol &&
            original_->equals(*f->original_,tol) && gtsam::NoiseModelFactor::equals(other,tol);
    }
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::make_shared<TdcpEndpointsFactor>(*this);
    }
};
}
