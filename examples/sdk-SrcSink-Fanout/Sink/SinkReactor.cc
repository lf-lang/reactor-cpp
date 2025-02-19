#include "SinkReactor.hh"
using namespace std;
    
void SinkReactor::construction() {
    cout << "Construction Sink n_ports:" << parameters.n_ports.value << "\n";
    req.set_width (parameters.n_ports.value);
    rsp.set_width (parameters.n_ports.value);
}

void SinkReactor::assembling() {

    cout << "Assembling Sink\n";

    reaction("startup_reaction").
        triggers(&startup).
        effects().
        function(pass_function(startup_reaction)
    );

    reaction("process_request").
        triggers(&req).
        effects(&rsp).
        function(pass_function(process_request)
    );
}



void SinkReactor::startup_reaction (Startup& startup) {
    cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
    "Starting up reaction\n" << "Bank:" << bank_index << " name:" << parameters.name.value << " fqn:" << fqn() << endl;
}

void SinkReactor::process_request (MultiportInput<int>& req, MultiportOutput<int>& rsp) {
    for (int i = 0; i < parameters.n_ports.value; ++i) {
        if (req[i].is_present()) {
            cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
            "Received input:" << *req[i].get() << " port:" << i << endl;
            rsp[i].set (*req[i].get());
        }
    }
}