#include <fstream>
#include <unordered_map>
#include <map>
#include "reactor-sdk/Environment.hh"

using namespace std;

namespace sdk
{

std::map<std::string, std::string> type_convert = {
    {"i", "int"},
    {"j", "uint32_t"},
    {"b", "bool"},
    {"f", "float"},
    {"d", "double"},
    {"c", "char"},
    {"s", "std::string"},
    {"ll", "long long"},
    {"ul", "unsigned long"},
    {"uc", "unsigned char"},
    {"ld", "long double"},
    {"uint", "unsigned int"}
};

Environment::Environment( ConfigParameterBase *cfg_param, unsigned int num_workers, bool fast_fwd_execution,
                                        const reactor::Duration& timeout, bool visualize)
    : reactor::Environment (num_workers, fast_fwd_execution, timeout), config_parameters(cfg_param), visualize(visualize) {
}


void Environment::run()
{
    if (this->config_parameters) {
        this->config_parameters->pull_config();
        // instance->config_parameters->display();
    }

    this->construct();
    this->assemble();

    if (this->config_parameters) {
        if (this->config_parameters->validate() != 0) {
            reactor::log::Error() << "INVALID CONFIGURATION!";
            return;
        }
    }

    if (visualize) {
        this->export_dependency_graph("graph.dot");
    }
    auto thread = this->startup();
    thread.join();
}

} // namespace sdk