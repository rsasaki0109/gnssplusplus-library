#include <libgnss++/io/android_raw_gnss.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc,char** argv) {
    if(argc!=2) return 2;
    libgnss::io::AndroidRawGnssConfig config;
    config.device_model="pixel5";
    config.verify_enriched_pseudorange=false;
    libgnss::io::AndroidRawGnssResult result;
    std::string error;
    if(!libgnss::io::loadAndroidRawGnssCsv(argv[1],config,result,error)) {
        std::cerr<<error<<'\n'; return 3;
    }
    std::size_t rows=0,metadata=0,carrier=0,missing=0,masked_with_metadata=0;
    std::vector<double> values;
    for(const auto& epoch:result.observations.epochs) for(const auto& obs:epoch.observations) {
        ++rows;
        if(obs.has_source_adr_uncertainty_m) {
            if(!std::isfinite(obs.source_adr_uncertainty_m) || obs.source_adr_uncertainty_m<=0.) return 4;
            ++metadata;
            if(!obs.has_carrier_phase) ++masked_with_metadata;
        } else if(obs.source_adr_uncertainty_m!=0.) return 5;
        if(obs.has_carrier_phase) {
            ++carrier;
            if(obs.has_source_adr_uncertainty_m) values.push_back(obs.source_adr_uncertainty_m);
            else ++missing;
        }
    }
    if(values.empty()) return 6;
    std::sort(values.begin(),values.end());
    const auto n=values.size();
    const double median=n%2?values[n/2]:(values[n/2-1]+values[n/2])/2.;
    std::cout<<std::setprecision(17)<<"epochs="<<result.observations.epochs.size()
        <<" rows="<<rows<<" metadata="<<metadata<<" carrier="<<carrier
        <<" carrier_missing_uncertainty="<<missing<<" masked_with_metadata="<<masked_with_metadata
        <<" carrier_uncertainty_median_m="<<median<<'\n';
}
