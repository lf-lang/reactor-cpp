#include "Config-a.hh"

UserParameters cfg_parameters;

ConfigParameter<int, Duration>::ParametersMap UserParameters::homogeneous_config() {
    return {
    };
}

ConfigParameter<int, Duration>::ParametersMap UserParameters::heterogeneous_config() {
    return {
            {"Main.slow.period", ConfigParameterMetadata<Duration> { 1s } },
            {"Main.slow.duration", ConfigParameterMetadata<Duration> { 5s } },
            {"Main.n_fast", ConfigParameterMetadata<int> { 3 } },
            {"Main.fast_0.period", ConfigParameterMetadata<Duration> { 500ms } },
            {"Main.fast_0.duration", ConfigParameterMetadata<Duration> { 10ms } }
    };
}