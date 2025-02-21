#include "Config-a.hh"

UserParameters cfg_parameters;

ConfigParameter<int, uint32_t, string>::ParametersMap UserParameters::homogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 5 } },
            {"Main.Sink.name", ConfigParameterMetadata<string> { "Homogeneous" } }
    };
}

ConfigParameter<int, uint32_t, string>::ParametersMap UserParameters::heterogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 20 } },
            {"Main.Source.n_ports", ConfigParameterMetadata<int> { 4 } },
            {"Main.n_sinks", ConfigParameterMetadata<int> { 4 } },
            {"Main.Sink_0.name", ConfigParameterMetadata<string> { "Heterogeneous_0" } },
            {"Main.Sink_2.name", ConfigParameterMetadata<string> { "Heterogeneous_2" } }

    };
}

// UserParameters::filter_out () {
//     if (cfg_map["T0.P0.L1.n_ervers"] != cfg_map["T0.P0.L2.n_ervers"]) {

//     }
// }
