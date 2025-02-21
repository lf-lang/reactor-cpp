#include <fstream>
#include <unordered_map>
#include <map>
#include "reactor-sdk/Reactor.hh"
#include "reactor-sdk/Environment.hh"

using namespace std;

namespace sdk
{

std::string Reactor::BankName(const std::string& name) {
    std::string bank_name = name;
    size_t index = bank_name.rfind("\r\n");
    if (index != std::string::npos) {
        bank_name.replace(index, strlen("\r\n"), "_");
    }
    return bank_name;
}

std::string Reactor::HomogName(const std::string& name) {
    std::string h_name = name;
    size_t index = h_name.rfind("\r\n");
    if (index != std::string::npos) {
        return h_name.substr(0, index);
    }
    return name;
}

Reactor::Reactor(const std::string &name, Environment *env)
    : reactor::Reactor(BankName(name), (reactor::Environment*)env), env(env) {
    env->add_reactor(this);
    homog_name = HomogName(name);
}

Reactor::Reactor(const std::string &name, Reactor *container)
    : reactor::Reactor(BankName(name), container), env(container->env), parent(container) {
    container->add_child (this);
    homog_name = container->homog_name + "." + HomogName(name);
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

void Reactor::populate_params(std::set<std::string> &types, std::set<std::string> &homog_map_entries, std::set<std::string> &hetero_map_entries) {
    if (p_param) {
        p_param->populate_params (types, homog_map_entries, hetero_map_entries);
    }

    for (auto *reactor : child_reactors) {
        reactor->populate_params(types, homog_map_entries, hetero_map_entries);
    }

}

} // namespace sdk