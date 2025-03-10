#pragma once

namespace sdk
{
class Reactor;

class ReactionBase : public std::enable_shared_from_this<ReactionBase>
{
public:
    Reactor *reactor;
    std::shared_ptr<ReactionBase> next;
    std::string name;

public:
    ReactionBase(std::string name, Reactor *parent)
    : reactor(parent), next(nullptr), name(name) {}
    virtual ~ReactionBase() = default;

    virtual void assemble() {}
};
    
} // namespace sdk