#pragma once

namespace sdk
{

class SystemParameterBase {
public:
    virtual ~SystemParameterBase() = default;
    virtual void fetch_config() = 0;
    virtual void print() = 0;
};
    
} // namespace sdk