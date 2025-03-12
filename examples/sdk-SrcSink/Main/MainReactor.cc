#include "MainReactor.hh"

void MainReactor::construction() {

    cout << "Construction Main n_sinks:" << parameters.n_sinks.value << " default n_sinks:" << parameters.defaults.n_sinks << "\n";

    src = std::make_unique<SourceReactor>("Source", this);

    for (int i = 0; i < parameters.n_sinks.value; i++) {
        snk.create_reactor(SinkReactor::Parameters{.name = "Default Sink Name"});
    }
}

void MainReactor::wiring() {
    cout << "Wiring Main n_sinks:" << parameters.n_sinks.value << " default n_sinks:" << parameters.defaults.n_sinks << "\n";

    src->req --> snk.for_each(select_default(snk).req);
    // src->req --> snk.for_each(&SinkReactor::req);         // alternative
    // src->req --> snk.for_each(&snk[0].req);              // alternative
    // src->req --> snk->*(select_default(snk).req);        // alternative
    // src->req --> snk->*(&SinkReactor::req);              //  alternative

    snk.for_each(select_default(snk).rsp) --> src->rsp;
    // snk.for_each(&SinkReactor::rsp) --> src->rsp;        // alternative
    // snk.for_each(&snk[0].rsp) --> src->rsp;              // alternative
    // (snk->*(select_default(snk).rsp)) --> src->rsp;      // alternative
    // (snk->*(&SinkReactor::rsp)) --> src->rsp;            // alternative
}

void REACTION_SCOPE(MainReactor)::add_reactions(MainReactor *reactor) {
    reaction("reaction_1").
        triggers(&reactor->startup).
        dependencies().
        effects().
        function(
            [this](Startup& startup) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Starting up reaction\n" << "Bank:" << bank_index << " name:" << parameters.alias.value << " fqn:" << fqn() << " n_sinks:" << parameters.n_sinks.value << endl;
            }
    );
}