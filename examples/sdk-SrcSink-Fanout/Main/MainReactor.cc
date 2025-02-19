#include "MainReactor.hh"

void MainReactor::construction() {

    cout << "Construction Main\n";

    src = std::make_unique<SourceReactor>("Source", this);
    snk = std::make_unique<SinkReactor>("Sink", this);
}

void MainReactor::assembling() {
    cout << "Assembling Main\n";

    src->req -->> snk->req;
    snk->rsp --> src->rsp;

    reaction("reaction_1").
        triggers(&startup).
        effects().
        function(
            [&](Startup& startup) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Starting up reaction\n" << "Bank:" << bank_index << " name:" << parameters.alias.value << " fqn:" << fqn() << endl;
            }
    );
}