#include <fstream>
#include <unordered_map>
#include <map>
#include "reactor-sdk/Reactor.hh"
#include "reactor-sdk/Environment.hh"

using namespace std;

namespace sdk
{

Reactor::Reactor(const std::string &name, Environment *env)
    : reactor::Reactor(name, (reactor::Environment*)env), env(env) {
}

Reactor::Reactor(const std::string &name, Reactor *container)
    : reactor::Reactor(name, container), env(container->env) {
    container->add_child (this);
}

void Reactor::add_child(Reactor* reactor) {
    [[maybe_unused]] bool result = child_reactors.insert(reactor).second;
    reactor_assert(result);
}

void Reactor::add_to_reaction_map (std::string &name, std::shared_ptr<BaseTrigger> reaction) {
    reaction_map[name] = reaction;
}

Reactor &Reactor::reaction (const std::string name) {
    current_reaction_name = name;
    return *this;
}

void Reactor::construct() {
    if (p_param) {
        p_param->fetch_config();
    }
    construction();
}
void Reactor::assemble() {
    assembling();
}

} // namespace sdk