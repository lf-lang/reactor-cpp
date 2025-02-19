#include "MainReactor.hh"

void MainReactor::construction() {

    std::cout << "Construction Main n_fast:" << parameters.n_fast.value << "\n";

    slow = std::make_unique<NodeReactor>("slow", this);

    for (int __lf_idx = 0; __lf_idx < parameters.n_fast.value; __lf_idx++) {
        std::string __lf_inst_name = "fast_" + std::to_string(__lf_idx);
        fast.emplace_back(std::make_unique<NodeReactor>(__lf_inst_name, this));
    }
}

void MainReactor::assembling() {
    std::cout << "Assembling Main n_sinks:" << parameters.n_fast.value << "\n";
}