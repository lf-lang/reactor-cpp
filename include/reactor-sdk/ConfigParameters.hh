#pragma once

#include <map>
#include <string>
#include <variant>

namespace sdk
{

template <typename T, typename = void>
struct is_comparable : std::false_type {};

template <typename T>
struct is_comparable<
    T,
    std::void_t<decltype(std::declval<T>() < std::declval<T>())>
> : std::true_type {};

extern std::map<std::string, std::string> type_convert;
template <typename T>
struct ParameterMetadata;

class ConfigParameterBase {
protected:
    virtual void pull_config() = 0;
    virtual void display() = 0;
    virtual int validate() = 0;

public:
    virtual ~ConfigParameterBase() = default;
    virtual int pull_config_parameter(const bool &is_heterogeneous, const std::string &key, void *user_param, const std::type_info& ti) = 0;

    template<typename T>
    int PullConfigParameter(const bool &is_heterogeneous, const std::string &key, ParameterMetadata<T>* user_param) {
        return pull_config_parameter(is_heterogeneous, key, static_cast<void*>(user_param), typeid(T));
    }
    friend class Environment;
};

template <typename T>
struct ConfigParameterMetadata {
    std::vector<T> values;

    ConfigParameterMetadata(std::initializer_list<T> val) : values(val) {}
};

template <typename... ParameterValueType>
class ConfigParameter : public ConfigParameterBase {
public:
    using ParameterValue = std::variant<ConfigParameterMetadata<ParameterValueType>...>;
    using ParametersMap = std::map<std::string, ParameterValue>;

    virtual ParametersMap homogeneous_config() = 0;
    virtual ParametersMap heterogeneous_config() = 0;
    int pull_config_parameter(const bool &is_heterogeneous, const std::string &key, void *user_param, const std::type_info& ti) override {
        std::map<std::string, ParameterValue> *param_map = is_heterogeneous ? &hetero_param_map : &homoge_param_map;
        std::set<std::string> *invalid_keys = is_heterogeneous ? &hetero_invalid_keys : &homoge_invalid_keys;
        auto itr_system = param_map->find(key);
        if (itr_system != param_map->end()) {
            auto v_it = invalid_keys->find(key);
            if (v_it != invalid_keys->end()) {
                invalid_keys->erase(v_it);
            }
            std::visit([is_heterogeneous, user_param, &ti, key](auto&& system_param) {
                using ContainerType = std::decay_t<decltype(system_param.values)>;
                using U = typename ContainerType::value_type;

                if (ti == typeid(U)) {
                    ParameterMetadata<U>* param = static_cast<ParameterMetadata<U>*>(user_param);
                    if constexpr (is_comparable<U>::value && !std::is_same<U, std::string>::value) {
                        if ((system_param.values[0] < param->min_value) ||
                            (system_param.values[0] > param->max_value)) {
                            reactor::log::Error() << "Error: " << ((is_heterogeneous) ? "Heterogeneous Map" : "Homogeneous Map") << " -- Range mismatch for parameter name: " << key << " value:" << system_param.values[0] <<
                                " min_value:" << param->min_value << " max_value:" << param->max_value;
                            std::exit(EXIT_FAILURE);
                        }
                    }
                    param->value = system_param.values[0];

                } else {
                    reactor::log::Error() << "Error: Type mismatch for parameter name: " << key << "\n"
                          << "Expected type: " << type_convert[ti.name()]
                          << ", Provided type: " << type_convert[typeid(U).name()];
                    std::exit(EXIT_FAILURE);
                }
            }, itr_system->second);
            return 0;
        }
        return -1;
    }

protected:
    std::map<std::string, ParameterValue> homoge_param_map;
    std::set<std::string> homoge_invalid_keys;
    std::map<std::string, ParameterValue> hetero_param_map;
    std::set<std::string> hetero_invalid_keys;
    void pull_config() override {
        homoge_param_map = homogeneous_config();
        for (const auto& entry : homoge_param_map) {
            bool result = homoge_invalid_keys.insert(entry.first).second;
            assert(result);
        }

        hetero_param_map = heterogeneous_config();
        for (const auto& entry : hetero_param_map) {
            bool result = hetero_invalid_keys.insert(entry.first).second;
            assert(result);
        }
    }

    int validate() override {
        for (const auto &key : hetero_invalid_keys) {
            reactor::log::Error() << "Heterogeneous Invalid key:" << key << "\n";
        }

        for (const auto &key : homoge_invalid_keys) {
            reactor::log::Error() << "Homogeneous Invalid key:" << key << "\n";
        }
        return (hetero_invalid_keys.size() + homoge_invalid_keys.size());
    }

    void display() override {
        for (const auto& entry : hetero_param_map) {
            reactor::log::Debug() << "Parameter: " << entry.first;

            std::visit([](auto&& param) {
                for (auto val : param.values) {
                    reactor::log::Debug() << "Value: " << val;
                }
            }, entry.second);
        }
    }
};

} // namespace sdk