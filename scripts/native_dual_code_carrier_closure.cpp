// Raw-only diagnostic, not a factor gate or atmospheric estimate.
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/core/signals.hpp>
#include <algorithm>
#include <array>
#include <iostream>
#include <map>

int main(int argc,char** argv) {
    if(argc!=2) return 2;
    using namespace libgnss;
    io::AndroidRawGnssConfig config;
    config.device_model="pixel5";
    config.verify_enriched_pseudorange=false;
    config.require_frequency_pair_timing=true;
    io::AndroidRawGnssResult raw; std::string error;
    if(!io::loadAndroidRawGnssCsv(argv[1],config,raw,error)) {
        std::cerr<<error<<'\n'; return 3;
    }
    struct Pair { double code,phase; };
    std::map<SatelliteId,Pair> previous;
    std::vector<double> absolute;
    for(std::size_t i=0;i<raw.observations.epochs.size();++i) {
        std::map<SatelliteId,std::array<const Observation*,2>> pairs;
        for(const auto& o:raw.observations.epochs[i].observations) {
            int band=-1;
            if(o.signal==SignalType::GPS_L1CA || o.signal==SignalType::GAL_E1) band=0;
            if(o.signal==SignalType::GPS_L5 || o.signal==SignalType::GAL_E5A) band=1;
            if(band>=0) pairs[o.satellite][band]=&o;
        }
        std::map<SatelliteId,Pair> current;
        for(const auto& [sat,p]:pairs) {
            bool valid=true;
            for(const auto* o:p) valid=valid && o && o->valid &&
                o->has_pseudorange && o->has_carrier_phase && !o->loss_of_lock &&
                o->lli==0 && std::isfinite(o->pseudorange) && std::isfinite(o->carrier_phase);
            if(!valid) continue;
            Pair value{p[0]->pseudorange-p[1]->pseudorange,
                p[0]->carrier_phase*signalWavelengthMeters(*p[0])-
                p[1]->carrier_phase*signalWavelengthMeters(*p[1])};
            current[sat]=value;
            const auto old=previous.find(sat);
            if(!i || old==previous.end()) continue;
            const double dt=raw.observations.epochs[i].time-raw.observations.epochs[i-1].time;
            if(!(dt>0 && dt<=1.5) || raw.epoch_hardware_clock_discontinuity_count[i]!=
                raw.epoch_hardware_clock_discontinuity_count[i-1]) continue;
            const double v=(value.code-old->second.code)+(value.phase-old->second.phase);
            if(std::isfinite(v)) absolute.push_back(std::abs(v));
        }
        previous=std::move(current);
    }
    if(absolute.empty()) return 4;
    std::sort(absolute.begin(),absolute.end());
    const auto q=[&](double f){double x=f*(absolute.size()-1); auto j=static_cast<std::size_t>(x);
        return absolute[j]+(x-j)*(absolute[std::min(j+1,absolute.size()-1)]-absolute[j]);};
    std::cout<<"{\"pairs\":"<<absolute.size()<<",\"abs_p50_m\":"<<q(.5)
        <<",\"abs_p95_m\":"<<q(.95)<<",\"abs_max_m\":"<<absolute.back()<<"}\n";
}
