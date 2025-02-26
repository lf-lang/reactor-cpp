#include "Config-a.hh"

UserParameters cfg_parameters;

ConfigParameter<int, uint32_t>::ParametersMap UserParameters::homogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 5 } }
    };
}

ConfigParameter<int, uint32_t>::ParametersMap UserParameters::heterogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 20 } },
            {"Main.Sink.n_ports", ConfigParameterMetadata<int> { 2 } }

    };
}