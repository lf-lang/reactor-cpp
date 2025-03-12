
#include "SourceReactor.hh"
using namespace std;

void SourceReactor::construction() {

    cout << "Construction Source iterations:" << parameters.iterations.value << "\n";
}

void SourceReactor::wiring() {
    cout << "Wiring Source iterations:" << parameters.iterations.value << "\n";
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
            [this](LogicalAction<int>& sch, Output<int>& req) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Scheduling iteration:" << *sch.get() << endl;
                req.set (*sch.get());
            }
        );

    reaction("reaction_3").
        triggers(&reactor->rsp).
        dependencies().
        effects(&reactor->sch).
        function(
            [this](Input<int>& rsp, LogicalAction<int>& sch) {
                if (rsp.is_present()) {
                    cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                    "Recevied response:" << *rsp.get() << endl;
                }
                
                if (itr < parameters.iterations.value) {
                    sch.schedule (itr, 0ms);
                    ++itr;
                } else {
                    request_stop();
                }
            }
        );
}