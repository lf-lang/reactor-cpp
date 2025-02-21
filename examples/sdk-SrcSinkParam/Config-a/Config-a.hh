#pragma once

#include <reactor-sdk/reactor-sdk.hh>
#include <map>
#include <variant>
#include <string>

using namespace sdk;

struct UserParameters : public ConfigParameter<int, uint32_t, string> {
    ConfigParameter<int, uint32_t, string>::ParametersMap homogeneous_config();
    ConfigParameter<int, uint32_t, string>::ParametersMap heterogeneous_config();
};

extern UserParameters cfg_parameters;