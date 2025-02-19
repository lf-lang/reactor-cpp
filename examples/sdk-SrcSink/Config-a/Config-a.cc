#include "Config-a.hh"

UserParameters cfg_parameters;

ConfigParameter<int, uint32_t>::ParametersMap UserParameters::homogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 0 } }
    };
}

ConfigParameter<int, uint32_t>::ParametersMap UserParameters::heterogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 20 } },
            {"Main.Source.n_ports", ConfigParameterMetadata<int> { 2 } },
            {"Main.n_sinks", ConfigParameterMetadata<int> { 2 } },

    };
}

// UserParameters::filter_out () {
//     if (cfg_map["T0.P0.L1.n_ervers"] != cfg_map["T0.P0.L2.n_ervers"]) {

//     }
// }
