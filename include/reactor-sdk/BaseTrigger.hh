#pragma once

namespace sdk
{
class Reactor;

class BaseTrigger : public std::enable_shared_from_this<BaseTrigger>
{
public:
    Reactor *reactor;
    std::shared_ptr<BaseTrigger> next;
    std::string name;

public:
    BaseTrigger(std::string name, Reactor *parent)
    : reactor(parent), next(nullptr), name (name) {}
    virtual ~BaseTrigger() = default;
};
    
} // namespace sdk