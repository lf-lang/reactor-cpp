#ifndef USER_PARAMETERS_H
#define USER_PARAMETERS_H

#include <reactor-sdk/reactor-sdk.hh>
#include <map>
#include <variant>
#include <string>

using namespace sdk;

struct UserParameters : public ConfigParameter<int, uint32_t> {
    ConfigParameter<int, uint32_t>::ParametersMap homogeneous_config();
    ConfigParameter<int, uint32_t>::ParametersMap heterogeneous_config();
};

// using ParametersMap = std::map<std::string, SystemParameterMetadata<std::variant<SystemParameterMetadata<ParameterValueType>...>>>;

extern UserParameters cfg_parameters;

#endif // USER_PARAMETERS_H
