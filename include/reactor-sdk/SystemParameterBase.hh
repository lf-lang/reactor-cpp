#pragma once

#include <set>
namespace sdk
{

class SystemParameterBase {
public:
    virtual ~SystemParameterBase() = default;
    virtual void fetch_config() = 0;
    virtual void print() = 0;
    virtual void populate_params(std::set<std::string> &types, std::set<std::string> &homog_map_entries, std::set<std::string> &hetero_map_entries) = 0;
};
    
} // namespace sdk