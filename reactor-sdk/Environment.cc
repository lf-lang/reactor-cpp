#include <fstream>
#include <unordered_map>
#include <map>
#include <cstdlib>

#include "reactor-sdk/Environment.hh"
#include "reactor-sdk/Reactor.hh"

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
                                        const reactor::Duration& timeout, bool _cfg_gen)
    : reactor::Environment (num_workers, fast_fwd_execution, timeout), config_parameters(cfg_param), cfg_gen(_cfg_gen) {
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

    this->export_dependency_graph("graph.dot");
    int ret = std::system("dot -Tpng graph.dot -o graph.png");
    if (ret != 0) {
        reactor::log::Error() << "Error: Failed to generate graph diagram from dot file";
    }
    
    if (cfg_gen) {
        std::set<std::string> types;
        std::map<std::string, std::string> homog_map_entries;
        std::map<std::string, std::string> hetero_map_entries;
        for (auto *reactor : top_tier_reactors) {
            reactor->populate_params (types, homog_map_entries, hetero_map_entries);
        }
        std::string type_str = "";
        bool first = true;
        for (auto &type : types) {
            type_str += first ? type : (", " + type);
            first = false;
        }
        std::cout << "TYPE_STR:" << type_str << std::endl;

        std::string homog_entry_str = "";
        first = true;
        for (auto &entry : homog_map_entries) {
            homog_entry_str += first ? ("\t\t" + entry.second) : (",\n\t\t" + entry.second);
            first = false;
        }
        std::cout << "HOMOG_ENTRY_STR:" << homog_entry_str << std::endl;

        std::string hetero_entry_str = "";
        first = true;
        for (auto &entry : hetero_map_entries) {
            hetero_entry_str += first ? ("\t\t" + entry.second) : (",\n\t\t" + entry.second);
            first = false;
        }
        std::cout << "HETERO_ENTRY_STR:" << hetero_entry_str << std::endl;

        std::string header_file_str =    std::string("#pragma once\n") +
                                        "#include <reactor-sdk/reactor-sdk.hh>\n" +
                                        "#include <map>\n" +
                                        "#include <variant>\n" +
                                        "#include <string>\n\n" +
                                        "using namespace sdk;\n\n" +
                                        "struct UserParameters : public ConfigParameter<" + type_str + "> {\n" +
                                        "\tConfigParameter<" + type_str + ">::ParametersMap homogeneous_config();\n" +
                                        "\tConfigParameter<" + type_str + ">::ParametersMap heterogeneous_config();\n" +
                                        "};\n" +
                                        "extern UserParameters cfg_parameters;";

        std::ofstream header("GeneratedConfig.hh");
        if (!header) {
            cout << "ERROR: Failed to open header file\n";
        } else {
            header << header_file_str;
        }

        std::string source_file_str =   std::string("#include \"GeneratedConfig.hh\"\n\n") +
                                        "UserParameters cfg_parameters;\n\n" +
                                        "ConfigParameter<" + type_str + ">::ParametersMap UserParameters::homogeneous_config() {\n" +
                                        "\treturn {\n" + homog_entry_str + "\n\t};\n}\n\n" +
                                        "ConfigParameter<" + type_str + ">::ParametersMap UserParameters::heterogeneous_config() {\n" +
                                        "\treturn {\n" + hetero_entry_str + "\n\t};\n}";

        std::ofstream source("GeneratedConfig.cc");
        if (!source) {
            cout << "ERROR: Failed to open source file\n";
        } else {
            source << source_file_str;
        }
        return;
    }
    auto thread = this->startup();
    thread.join();
}

} // namespace sdk