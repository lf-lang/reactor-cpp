#include "Config-a.hh"

UserParameters cfg_parameters;

ConfigParameter<int, uint32_t, string>::ParametersMap UserParameters::homogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 5 } },
            // {"Main.Sink.name", ConfigParameterMetadata<string> { "Homog Name" } },
    };
}

ConfigParameter<int, uint32_t, string>::ParametersMap UserParameters::heterogeneous_config() {
    return {
            {"Main.Source.iterations", ConfigParameterMetadata<int> { 20 } },
            {"Main.Source.n_ports", ConfigParameterMetadata<int> { 4 } },
            {"Main.n_sinks", ConfigParameterMetadata<int> { 4 } },
            // {"Main.Sink_0.name", ConfigParameterMetadata<string> { "Hetero Name 0" } },
            // {"Main.Sink_1.name", ConfigParameterMetadata<string> { "Hetero Name 1" } },
            // {"Main.Sink_2.name", ConfigParameterMetadata<string> { "Hetero Name 2" } },
            // {"Main.Sink_3.name", ConfigParameterMetadata<string> { "Hetero Name 3" } },

    };
}
