#include "MainReactor.hh"

void MainReactor::construction() {

    std::cout << "Construction Main n_fast:" << parameters.n_fast.value << "\n";

    slow = std::make_unique<NodeReactor>("slow", this);

    for (int i = 0; i < parameters.n_fast.value; i++) {
        fast.create_reactor();
    }
}

void MainReactor::assembling() {
    std::cout << "Assembling Main n_sinks:" << parameters.n_fast.value << "\n";
}