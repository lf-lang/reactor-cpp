
#include "SourceReactor.hh"
using namespace std;

void SourceReactor::construction() {

    cout << "Construction Source n_ports:" << parameters.n_ports.value << "\n";

    req.set_width (parameters.n_ports.value);
    rsp.set_width (parameters.n_ports.value);
}

void SourceReactor::wiring() {
    cout << "Wiring Source n_ports:" << parameters.n_ports.value << "\n";
}

void REACTION_SCOPE(SourceReactor)::add_reactions(SourceReactor *reactor) {
    reaction("reaction_1").
        triggers(&reactor->startup).
        dependencies().
        effects(&reactor->sch).
        function(
            [this](Startup& startup, LogicalAction<int>& sched) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Starting up reaction\n" << "Bank:" << bank_index << " name:" << name << " fqn:" << fqn() << " iterations:" << parameters.iterations.value << endl;
                if (itr < parameters.iterations.value) {
                    sched.schedule (itr, 0ms);
                    ++itr;
                }
            }
        );

    reaction("reaction_2").
        triggers(&reactor->sch).
        dependencies().
        effects(&reactor->req).
        function(
            [this](LogicalAction<int>& sch, MultiportOutput<int>& req) {
                for (int i = 0; i < parameters.n_ports.value; ++i) {
                    cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                    "Scheduling iteration:" << *sch.get() << " out_port:" << i << endl;
                    req[i].set (*sch.get());
                }
            }
        );

    reaction("reaction_3").
        triggers(&reactor->rsp).
        dependencies().
        effects(&reactor->sch).
        function(
                [this](MultiportInput<int>& rsp, LogicalAction<int>& sch) {
                    for (int i = 0; i < parameters.n_ports.value; ++i) {
                        if (rsp[i].is_present()) {
                            cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                            "Recevied response:" << *rsp[i].get() << " in_port:" << i << endl;
                            ++rsp_itr;
                        }
                    }

                    if (rsp_itr < parameters.n_ports.value) {
                        return;
                    }

                    rsp_itr = 0;
                    
                    if (itr < parameters.iterations.value) {
                        sch.schedule (itr, 0ms);
                        ++itr;
                    } else {
                        request_stop();
                    }
                }
        );
}