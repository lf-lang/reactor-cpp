#pragma once

#include "reactor-cpp/reactor-cpp.hh"
#include "ConfigParameters.hh"

namespace sdk
{

class Environment: public reactor::Environment {
private:
    ConfigParameterBase *config_parameters;
    bool visualize = false;
public:
    Environment(ConfigParameterBase *sys_param = nullptr, unsigned int num_workers = 1, bool fast_fwd_execution = true,
                       const reactor::Duration& timeout = reactor::Duration::max(), bool visualize = false);

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;
    void run();

    ConfigParameterBase *get_config_params() { return config_parameters; }

    friend class SystemParameterBase;
};
    
} // namespace sdk