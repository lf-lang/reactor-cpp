#pragma once

#include <set>
#include <map>
namespace sdk
{

class SystemParameterBase {
public:
    virtual ~SystemParameterBase() = default;
    virtual void fetch_config() = 0;
    virtual void print() = 0;
    virtual void populate_params(std::set<std::string> &types, std::map<std::string, std::string> &homog_map_entries, std::map<std::string, std::string> &hetero_map_entries) = 0;
};
    
} // namespace sdk