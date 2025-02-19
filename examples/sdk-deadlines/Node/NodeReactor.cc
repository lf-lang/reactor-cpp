#include "NodeReactor.hh"

void NodeReactor::construction() {
    std::cout << "Construction:" << fqn() << " period:" << parameters.period.value << " duration:" << parameters.duration.value << "\n";
}

void NodeReactor::assembling() {
    std::cout << "Assembling Node\n";

    reaction("reaction_1").
        triggers(&startup, &a).
        effects().
        function(
            [&](Startup& startup, LogicalAction<void> &a) {
                reactor::log::Info() << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " << fqn() << " reaction executes.";
                std::this_thread::sleep_for(parameters.duration.value);
                reactor::log::Info() << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " << fqn() << " reaction done.";
                a.schedule(parameters.period.value);
            }
        ).deadline (parameters.period.value, 
            [&](Startup& startup, LogicalAction<void> &a) {
                reactor::log::Error() << "(" << get_elapsed_logical_time() << ", " << get_microstep() << "), physical_time: " << get_elapsed_physical_time() << " " << fqn() << " deadline was violated!";
                exit(1);
            }
        );
}