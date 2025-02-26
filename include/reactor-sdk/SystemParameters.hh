#pragma once

#include <string>
#include <cxxabi.h>
#include "SystemParameterBase.hh"
#include "Environment.hh"
#include "time_parser.hh"
namespace sdk
{

inline std::string convertToString(const std::string& s) {
    std::string annotated = "\"" + s + "\"";
    return annotated;
}

template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value, std::string>::type
convertToString(const T& value) {
    return std::to_string(value);
}

inline std::string convertToString(const reactor::Duration& dur) {
  return time_to_string (dur);
}

template <typename T>
struct ParameterMetadata {
    std::string name;
    std::string description;
    T min_value;
    T max_value;
    T value;
    std::string type_name;
};

#define REACTOR_PARAMETER(Type, variable_name, description, min_value, max_value, default_value) \
    ParameterMetadata<Type> variable_name = ParameterMetadata<Type>{ #variable_name, description, min_value, max_value, default_value, #Type }

template <typename... ParameterValueType>
class SystemParameter : public SystemParameterBase {
public:
    using ParameterValue = std::variant<ParameterMetadata<ParameterValueType>*...>;

    struct MapEntry {
        std::string alt_name;
        ParameterValue param;
    };

    SystemParameter(Reactor *owner)
    : reactor(owner), env(owner->get_env()) {
        reactor->set_param (this);
    }

    void fetch_config() override {
        if (env->get_config_params()) {
            for (auto& entry : param_map) {
                std::visit([&](auto* paramMetadataPtr) {
                    env->get_config_params()->PullConfigParameter(false, entry.second.alt_name, paramMetadataPtr);
                    env->get_config_params()->PullConfigParameter(true, entry.first, paramMetadataPtr);
                }, entry.second.param);
            }
        }
    }

    void print() override {
        for (const auto& entry : param_map) {
            std::cout << "Parameter: " << entry.first << ", alt_name:" << entry.second.alt_name << ", ";

            std::visit([](auto&& param) {
                std::cout << "Description: " << param->description 
                          << ", Value: " << param->value 
                          << ", Value Type Key: " << typeid(param->value).name()
                          << ", Value Type: " << param->type_name
                          << std::endl;
            }, entry.second.param);
        }
    }

    template <typename... Args>
    void register_parameters(Args&... args) {
        register_parameters_(reactor->fqn(), reactor->homog_fqn(), args...);
        print();
    }

    void populate_params(std::set<std::string> &types, std::map<std::string, std::string> &homog_map_entries, std::map<std::string, std::string> &hetero_map_entries) {
        for (const auto& entry : param_map) {
            std::cout << "POPULATING: " << entry.first << ", alt_name:" << entry.second.alt_name << std::endl;            

            std::visit([&](auto&& param) {
                bool result = types.insert(param->type_name).second;
                std::cout << "type:" <<  param->type_name << (result ? " PUSHED" : " SKIPPED") << std::endl;
                if (homog_map_entries.find (entry.second.alt_name) == homog_map_entries.end()) {
                    std::string homog_entry_str = "{ \"" + entry.second.alt_name + "\", ConfigParameterMetadata<" + param->type_name + "> {" + convertToString(param->value) + "} }";
                    homog_map_entries[entry.second.alt_name] = homog_entry_str;
                    std::cout << "homog-entry:" <<  homog_entry_str << " PUSHED" << std::endl;
                } else {
                    std::cout << "homog-entry:" <<  entry.second.alt_name << " SKIPPED" << std::endl;
                }

                if (hetero_map_entries.find (entry.first) == hetero_map_entries.end()) {
                    std::string hetero_entry_str = "{ \"" + entry.first + "\", ConfigParameterMetadata<" + param->type_name + "> {" + convertToString(param->value) + "} }";
                    hetero_map_entries[entry.first] = hetero_entry_str;
                    std::cout << "hetero-entry:" <<  hetero_entry_str << " PUSHED" << std::endl;
                } else {
                    std::cout << "hetero-entry:" <<  entry.first << " SKIPPED" << std::endl;
                }
            }, entry.second.param);
        }
    }

private:
    std::map<std::string, MapEntry> param_map;
    Reactor *reactor;
    Environment *env;

    template <typename T>
    void register_parameter(const std::string& hetero_name, const std::string& homog_name, ParameterMetadata<T>& param) {
        MapEntry entry = MapEntry {
            .alt_name = homog_name,
            .param = &param
        };
        param_map[hetero_name] = std::move(entry);
    }

    template <typename T, typename... Args>
    void register_parameters_(const std::string& hetero_name, const std::string& homog_name, ParameterMetadata<T>& first, Args&... args) {
        register_parameter(hetero_name + "." + first.name, homog_name + "." + first.name, first);
        if constexpr (sizeof...(args) > 0) {
            register_parameters_(hetero_name, homog_name, args...);
        }
    }
};

} // namespace sdk