#ifndef USER_PARAMETERS_H
#define USER_PARAMETERS_H

#include <reactor-sdk/reactor-sdk.hh>
#include <map>
#include <variant>
#include <string>

using namespace sdk;

struct UserParameters : public ConfigParameter<int, Duration> {
    ConfigParameter<int, Duration>::ParametersMap homogeneous_config();
    ConfigParameter<int, Duration>::ParametersMap heterogeneous_config();
};

// using ParametersMap = std::map<std::string, SystemParameterMetadata<std::variant<SystemParameterMetadata<ParameterValueType>...>>>;

extern UserParameters cfg_parameters;

#endif // USER_PARAMETERS_H
