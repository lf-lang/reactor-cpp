#include "SinkReactor.hh"
using namespace std;
    
void SinkReactor::construction() {
    cout << "Construction Sink n_ports:" << parameters.n_ports.value << "\n";
    req.set_width (parameters.n_ports.value);
    rsp.set_width (parameters.n_ports.value);
}

void SinkReactor::wiring() {
    cout << "Wiring Sink\n";
}

void REACTION_SCOPE(SinkReactor)::add_reactions (SinkReactor *reactor) {
    reaction("startup_reaction").
        triggers(&reactor->startup).
        dependencies().
        effects().
        function(pass_function(startup_reaction)
    );

    reaction("process_request").
        triggers(&reactor->req).
        dependencies().
        effects(&reactor->rsp).
        function(pass_function(process_request)
    );
}



void REACTION_SCOPE(SinkReactor)::startup_reaction (Startup& startup) {
    cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
    "Starting up reaction\n" << "Bank:" << bank_index << " name:" << parameters.name.value << " fqn:" << fqn() << endl;
}

void REACTION_SCOPE(SinkReactor)::process_request (MultiportInput<int>& req, MultiportOutput<int>& rsp) {
    for (int i = 0; i < parameters.n_ports.value; ++i) {
        if (req[i].is_present()) {
            cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
            "Received input:" << *req[i].get() << " port:" << i << endl;
            rsp[i].set (*req[i].get());
        }
    }
}