// Aggregate raw-base support diagnostic; no positioning/truth outputs.
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <set>
#include <iostream>
#include <iomanip>
#include <limits>
extern "C" {
#include "rtklib.h"
}
int main(int argc, char** argv) {
    using namespace libgnss;
    if (argc != 3 && argc != 4) return 2;
    io::RINEXReader base, nr;
    base.setSourceHeaderTrackingFilter(true);
    base.setPreserveAdditionalFrequencyBands(true);
    io::RINEXReader::RINEXHeader header;
    NavigationData nav;
    ObservationSeries series;
    if (!base.open(argv[1]) || !base.readHeader(header) ||
        header.version < 3 || header.version >= 4 ||
        !header.has_approximate_position || !header.approximate_position.allFinite() ||
        header.approximate_position.norm() < 6e6 || header.approximate_position.norm() > 7e6 ||
        !header.has_antenna_delta || !header.antenna_delta.isZero() ||
        !nr.open(argv[2]) || !nr.readNavigationData(nav) ||
        !base.readAllObservations(series)) return 3;
    std::size_t rows=0, below5=0, nonpositive=0, gps_rows=0, gps_below5=0;
    double minimum=std::numeric_limits<double>::infinity();
    using Key=std::pair<SatelliteId,source_transmission_clock::Slot>;
    std::map<Key,std::vector<double>> deltas, elevations;
    std::map<std::pair<SatelliteId,SignalType>,Key> signal_keys;
    double position[3];
    ecef2geodetic(header.approximate_position,position[0],position[1],position[2]);
    std::size_t epoch_index=0;
    for (const auto& epoch : series.epochs) {
        const auto states=source_transmission_clock::buildEpochStates(epoch.time,epoch.observations,nav);
        for (const auto& obs : epoch.observations) {
            if (!obs.valid || !obs.has_pseudorange || !std::isfinite(obs.pseudorange) || obs.pseudorange<=0) continue;
            const auto& code=obs.pseudorange_observation_type;
            const auto slot=code.size()==3 ? source_transmission_clock::slotForRinexBand(obs.satellite.system,code[1]-'0') : std::nullopt;
            if (!slot) return 4;
            if (*slot!=source_transmission_clock::Slot::L1 && *slot!=source_transmission_clock::Slot::L5) continue;
            const auto& state=states.at(obs.satellite);
            // Count all health values: conservative support superset.
            const auto geometry=phase126_raw_base::geodistWithSagnac(header.approximate_position,state.state.position_ecef);
            const double degrees=geometry.elevation_rad*180/std::acos(-1.0);
            if (!std::isfinite(degrees)) return 5;
            ++rows; minimum=std::min(minimum,degrees);
            below5+=degrees<5; nonpositive+=degrees<=0;
            if (obs.satellite.system==GNSSSystem::GPS) {++gps_rows; gps_below5+=degrees<5;}
            const Key key{obs.satellite,*slot};
            const auto signal_key=std::make_pair(obs.satellite,obs.signal);
            if (signal_keys.count(signal_key) && signal_keys.at(signal_key)!=key) return 9;
            signal_keys.insert_or_assign(signal_key,key);
            if (!deltas.count(key)) {
                deltas[key].assign(series.epochs.size(),std::numeric_limits<double>::quiet_NaN());
                elevations[key]=deltas[key];
            }
            if (std::isfinite(deltas[key][epoch_index])) return 7;
            const double azel[2]={geometry.azimuth_rad,geometry.elevation_rad};
            // New-minus-old base residual = old-minus-new modeled delay.
            deltas[key][epoch_index]=models::tropDelaySaastamoinen(header.approximate_position,azel[1])-
                tropmodel(gpst2time(epoch.time.week,epoch.time.tow),position,azel,0.7);
            if (!std::isfinite(deltas[key][epoch_index])) return 8;
            elevations[key][epoch_index]=degrees;
        }
        ++epoch_index;
    }
    if (!rows) return 6;
    std::cout<<std::setprecision(17)<<"{\"epochs\":"<<series.epochs.size()
             <<",\"rows\":"<<rows<<",\"below_5deg\":"<<below5
             <<",\"nonpositive\":"<<nonpositive<<",\"minimum_elevation_deg\":"<<minimum
             <<",\"gps_rows\":"<<gps_rows<<",\"gps_below_5deg\":"<<gps_below5<<"}\n";
    std::size_t changed_raw=0, changed_smoothed=0, changed_above5=0, changed_above15=0;
    double max_raw=0,max_smoothed=0,max_above5=0,max_above15=0;
    std::map<Key,std::vector<double>> smooth_streams;
    for (const auto& [key,values] : deltas) {
        const auto smoothed=base_pseudorange_compensation::centeredMovingMean(values,151);
        smooth_streams.emplace(key,smoothed);
        for (std::size_t i=0;i<values.size();++i) {
            if (std::isfinite(values[i])) {
                max_raw=std::max(max_raw,std::abs(values[i]));
                changed_raw+=std::abs(values[i])>1e-6;
            }
            if (!std::isfinite(smoothed[i])) continue;
            const double magnitude=std::abs(smoothed[i]);
            max_smoothed=std::max(max_smoothed,magnitude);
            changed_smoothed+=magnitude>1e-6;
            const double el=elevations.at(key)[i];
            if (el>=5) {max_above5=std::max(max_above5,magnitude);changed_above5+=magnitude>1e-6;}
            if (el>=15) {max_above15=std::max(max_above15,magnitude);changed_above15+=magnitude>1e-6;}
        }
    }
    std::cout<<"{\"streams\":"<<deltas.size()<<",\"changed_raw_rows\":"<<changed_raw
             <<",\"changed_smoothed_grid_points\":"<<changed_smoothed
             <<",\"changed_observed_above5\":"<<changed_above5
             <<",\"changed_observed_above15\":"<<changed_above15
             <<",\"max_raw_delta_m\":"<<max_raw<<",\"max_smoothed_delta_m\":"<<max_smoothed
             <<",\"max_observed_above5_delta_m\":"<<max_above5
             <<",\"max_observed_above15_delta_m\":"<<max_above15<<"}\n";
    if (argc==4) {
        io::AndroidRawGnssConfig config;
        config.verify_enriched_pseudorange=false;
        io::AndroidRawGnssResult phone;
        std::string error;
        if (!io::loadAndroidRawGnssCsv(argv[3],config,phone,error)) return 10;
        FGOProcessor::FGOConfig fgo_config;
        fgo_config.use_source_rover_epoch_states=true;
        fgo_config.use_multi_constellation=true;
        fgo_config.use_multi_frequency_double_difference=true;
        fgo_config.use_upstream_observable_quality=true;
        fgo_config.use_native_phase126_raw_base_source_complete=true;
        const auto problem=FGOProcessor(fgo_config).buildPseudorangeProblem(phone.observations.epochs,nav);
        using Admission=std::tuple<int,double,SatelliteId,SignalType>;
        std::set<Admission> admitted;
        for (const auto& factor : problem.pseudorange_factors) {
            const auto& time=problem.epochs.at(factor.epoch_index).time;
            admitted.emplace(time.week,time.tow,factor.satellite,factor.signal);
        }
        std::size_t admitted_finite=0,admitted_changed=0;
        double admitted_maximum=0;
        std::size_t candidates=0,finite=0,changed=0,missing=0,unavailable=0;
        double maximum=0;
        for (const auto& epoch : phone.observations.epochs) {
            const auto upper=std::lower_bound(series.epochs.begin(),series.epochs.end(),epoch.time,
                [](const auto& base_epoch,const auto& time){return base_epoch.time<time;});
            for (const auto& obs : epoch.observations) {
                if (!obs.valid || !obs.has_pseudorange) continue;
                ++candidates;
                const auto key=signal_keys.find({obs.satellite,obs.signal});
                if (key==signal_keys.end()) {++missing;continue;}
                if (upper==series.epochs.end() || epoch.time<series.epochs.front().time) {++unavailable;continue;}
                const auto index=static_cast<std::size_t>(upper-series.epochs.begin());
                const auto& values=smooth_streams.at(key->second);
                double delta=values[index];
                if (upper->time-epoch.time!=0) {
                    if (index==0) {++unavailable;continue;}
                    const double span=upper->time-series.epochs[index-1].time;
                    if (!(span>0)) return 11;
                    const double weight=(epoch.time-series.epochs[index-1].time)/span;
                    delta=values[index-1]*(1-weight)+values[index]*weight;
                }
                if (!std::isfinite(delta)) {++unavailable;continue;}
                ++finite;changed+=std::abs(delta)>1e-6;
                maximum=std::max(maximum,std::abs(delta));
                if (admitted.count({epoch.time.week,epoch.time.tow,obs.satellite,obs.signal})) {
                    ++admitted_finite;admitted_changed+=std::abs(delta)>1e-6;
                    admitted_maximum=std::max(admitted_maximum,std::abs(delta));
                }
            }
        }
        std::cout<<"{\"raw_phone_candidates\":"<<candidates<<",\"finite_delta_queries\":"<<finite
                 <<",\"changed_queries\":"<<changed<<",\"missing_stream\":"<<missing
                 <<",\"unavailable\":"<<unavailable<<",\"max_query_delta_m\":"<<maximum<<"}\n";
        std::cout<<"{\"diagnostic_fgo_factors\":"<<problem.pseudorange_factors.size()
                 <<",\"admitted_finite_queries\":"<<admitted_finite
                 <<",\"admitted_changed_queries\":"<<admitted_changed
                 <<",\"max_admitted_delta_m\":"<<admitted_maximum<<"}\n";
    }
}
