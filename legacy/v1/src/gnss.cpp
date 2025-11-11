#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/io/rinex.hpp>

namespace libgnss {

class GNSSProcessor::Impl {
public:
    Mode mode_ = Mode::SPP;
    std::unique_ptr<ProcessorBase> processor_;
    ProcessorConfig config_;
    
    Impl() {
        // Initialize with default SPP processor
        processor_ = std::make_unique<SPPProcessor>();
        processor_->initialize(config_);
    }
    
    void setMode(Mode mode) {
        mode_ = mode;
        switch (mode) {
            case Mode::SPP:
                processor_ = std::make_unique<SPPProcessor>();
                break;
            case Mode::RTK:
                processor_ = std::make_unique<RTKProcessor>();
                break;
            case Mode::PPP:
                processor_ = std::make_unique<PPPProcessor>();
                break;
        }
        processor_->initialize(config_);
    }
};

GNSSProcessor::GNSSProcessor() : pImpl(std::make_unique<Impl>()) {}

GNSSProcessor::~GNSSProcessor() = default;

bool GNSSProcessor::loadConfig(const std::string& config_file) {
    // Simple implementation - in real version would parse YAML/JSON config
    (void)config_file; // Suppress unused parameter warning
    return true;
}

void GNSSProcessor::setMode(Mode mode) {
    pImpl->setMode(mode);
}

Solution GNSSProcessor::processFile(const std::string& obs_file, const std::string& nav_file) {
    Solution solution;
    
    try {
        // Read RINEX files
        io::RINEXReader obs_reader, nav_reader;
        
        if (!obs_reader.open(obs_file)) {
            return solution; // Empty solution
        }
        
        if (!nav_reader.open(nav_file)) {
            return solution; // Empty solution
        }
        
        // Read navigation data
        NavigationData nav_data;
        nav_reader.readNavigationData(nav_data);
        
        // Process observation epochs
        ObservationData obs_data;
        while (obs_reader.readObservationEpoch(obs_data)) {
            auto pos_solution = pImpl->processor_->processEpoch(obs_data, nav_data);
            if (pos_solution.isValid()) {
                solution.addSolution(pos_solution);
            }
        }
        
    } catch (const std::exception&) {
        // Return empty solution on error
    }
    
    return solution;
}

PositionSolution GNSSProcessor::processEpoch(const ObservationData& obs, const NavigationData& nav) {
    return pImpl->processor_->processEpoch(obs, nav);
}

ProcessorStats GNSSProcessor::getStats() const {
    return pImpl->processor_->getStats();
}

} // namespace libgnss
