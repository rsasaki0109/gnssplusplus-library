#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/gps_triple_code_closure.hpp>
#include <libgnss++/algorithms/source_ephemeris_selection.hpp>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());
    const auto n=values.size();
    return n%2 ? values[n/2] : (values[n/2-1]+values[n/2])/2.;
}
int main(int argc,char** argv) {
    using namespace libgnss;
    if(argc!=2 && argc!=3) return 2;
    NavigationData nav;
    if(argc==3) {
        io::RINEXReader navigation;
        if(!navigation.open(argv[2]) || !navigation.readNavigationData(nav)) return 7;
    }
    io::RINEXReader reader;
    reader.setSourceHeaderTrackingFilter(true);
    reader.setPreserveAdditionalFrequencyBands(true);
    io::RINEXReader::RINEXHeader header;
    ObservationSeries series;
    if(!reader.open(argv[1]) || !reader.readHeader(header) ||
       header.version<3 || header.version>=4 || !reader.readAllObservations(series)) return 3;
    std::map<std::string,std::map<int,std::vector<double>>> streams;
    std::size_t missing_tgd=0;
    for(const auto& epoch:series.epochs) {
        std::map<int,std::map<std::string,double>> rows;
        for(const auto& obs:epoch.observations) {
            if(obs.satellite.system!=GNSSSystem::GPS || !obs.valid || !obs.has_pseudorange ||
               !std::isfinite(obs.pseudorange) || obs.pseudorange<=0.) continue;
            auto& row=rows[obs.satellite.prn];
            if(!row.emplace(obs.pseudorange_observation_type,obs.pseudorange).second) return 4;
        }
        for(const auto& [sat,row]:rows) for(const std::string middle:{"C2W","C2X"}) {
            if(!row.count("C1C") || !row.count(middle) || !row.count("C5X")) continue;
            const auto c=gps_triple_code::closure(row.at("C1C"),row.at(middle),row.at("C5X"));
            if(!c) return 5;
            streams[middle][sat].push_back(*c);
            if(argc==3 && middle=="C2W") {
                SatelliteId satellite;
                satellite.system=GNSSSystem::GPS; satellite.prn=sat;
                const auto records=nav.ephemeris_data.find(satellite);
                const Ephemeris* eph=records==nav.ephemeris_data.end() ? nullptr :
                    source_transmission_clock::selectBroadcastMessage(records->second,satellite,epoch.time);
                if(!eph || eph->health!=0 || !std::isfinite(eph->tgd)) { ++missing_tgd; continue; }
                const auto tgd=gps_triple_code::tgdComponent(eph->tgd);
                if(!tgd) return 8;
                streams["C2W_tgd_supported_raw"][sat].push_back(*c);
                streams["C2W_tgd_component"][sat].push_back(*tgd);
                streams["C2W_minus_tgd_only"][sat].push_back(*c-*tgd);
            }
        }
    }
    std::cout<<std::setprecision(17)<<"epochs="<<series.epochs.size()<<'\n';
    if(argc==3) std::cout<<"missing_or_unhealthy_tgd_rows="<<missing_tgd<<'\n';
    for(const auto& [code,by_sat]:streams) {
        std::vector<double> centers,mads;
        std::size_t count=0;
        for(const auto& [sat,values]:by_sat) {
            count+=values.size();
            const double center=median(values);
            centers.push_back(std::abs(center));
            std::vector<double> deviations;
            for(double value:values) deviations.push_back(std::abs(value-center));
            mads.push_back(median(deviations));
        }
        std::cout<<code<<" rows="<<count<<" satellites="<<centers.size()
                 <<" median_abs_stream_center_m="<<median(centers)
                 <<" median_stream_mad_m="<<median(mads)<<'\n';
    }
    return streams.empty()?6:0;
}
