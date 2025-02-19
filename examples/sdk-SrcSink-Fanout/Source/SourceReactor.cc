
#include "SourceReactor.hh"
using namespace std;

void SourceReactor::construction() {

    cout << "Construction Source iterations:" << parameters.iterations.value << "\n";
}

void SourceReactor::assembling() {
    cout << "Assembling Source iterations:" << parameters.iterations.value << "\n";
    reaction("reaction_1").
        triggers(&startup).
        effects(&sch).
        function(
            [&](Startup& startup, LogicalAction<int>& sched) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Starting up reaction\n" << "Bank:" << bank_index << " name:" << name << " fqn:" << fqn() << " iterations:" << parameters.iterations.value << endl;
                if (itr < parameters.iterations.value) {
                    sched.schedule (itr, 0ms);
                    ++itr;
                }
            }
        );

    reaction("reaction_2").
        triggers(&sch).
        effects(&req).
        function(
            [&](LogicalAction<int>& sch, Output<int>& req) {
                cout << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " <<
                "Scheduling iteration:" << *sch.get() << endl;
                req.set (*sch.get());
            }
        );

    reaction("reaction_3").
        triggers(&rsp).
        effects(&sch).
        function(
            [&](Input<int>& rsp, LogicalAction<int>& sch) {
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