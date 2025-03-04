
#include <reactor-sdk/reactor-sdk.hh>

using namespace sdk;

class Relay : public Reactor {
public:
    struct Parameters {
        int n_outputs;
    };

private:
    int index = 0;
    int *busy = 0;

    Parameters parameters;
    const int &n_outputs = parameters.n_outputs;

public:
    Input<int> in_req{"in_req", this};
    Output<int> out_rsp{"out_rsp", this};
    MultiportOutput<int> out_req{"out_req", this};
    MultiportInput<int> in_rsp{"in_rsp", this};

    Output<bool> all_workers_busy{"all_workers_busy", this};

    Relay(const std::string &name, Environment *env, Parameters &&param)
    : Reactor(name, env), parameters{std::forward<Parameters>(param)} {}
    Relay(const std::string &name, Reactor *container, Parameters &&param)
    : Reactor(name, container), parameters{std::forward<Parameters>(param)} {}

    void construction() override {
        out_req.set_width(n_outputs);
        in_rsp.set_width(n_outputs);
    }

    void assembling() override {
        reaction("reaction_1").
            triggers(&startup).
            dependencies().
            effects().
            function(
                [&](Startup& startup) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                    << fqn() << " Startup\n";
                    busy = (int*) calloc (n_outputs, sizeof(int));
                }
            );

        reaction("reaction_2").
            triggers(&in_req).
            dependencies().
            effects(&all_workers_busy, &out_req).
            function(
                [&](Input<int> &in_req, Output<bool> &all_workers_busy, MultiportOutput<int> &out_req) {
                    for (int i = 0; i < n_outputs; ++i, index = (index + 1) % n_outputs) {
                        if (busy[index] == 0) {
                            out_req[index].set(*in_req.get());
                            std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                        << fqn() << " Sending task_id:" << *in_req.get() << " to worker:" << index << std::endl;
                            busy[index] = 1;
                            index = (index + 1) % n_outputs;
                            break;
                        }
                    }
                    int busy_count = 0;
                    for (int i = 0; i < n_outputs; ++i) {
                        busy_count = busy[i] ? (busy_count + 1) : busy_count;
                    }

                    if (busy_count == n_outputs) {
                        all_workers_busy.set(true);
                    }
                }
            );

        reaction("reaction_3").
            triggers(&in_rsp).
            dependencies().
            effects(&out_rsp).
            function(
                [&](MultiportInput<int> &in_rsp, Output<int> &out_rsp) {
                    for (int i = 0; i < n_outputs; ++i) {
                        if (in_rsp[i].is_present()) {
                            std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                        << fqn() << " Receiving task_id:" << *in_rsp[i].get() << " from worker:" << i << std::endl;
                            busy[i] = 0;
                            out_rsp.set(*in_rsp[i].get());
                        }
                    }
                }
            );

        reaction("reaction_4").
            triggers(&shutdown).
            dependencies().
            effects().
            function(
                [&](Shutdown &shutdown) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Shutdown\n";
                }
            );
    }
};

class Worker : public Reactor {
public:
    struct Parameters {
        Duration processing_delay = 2s;
    };

private:
    LogicalAction<int> sch_rsp{"sch_rsp", this};

    Parameters parameters;
    const Duration &processing_delay = parameters.processing_delay;

public:
    Worker(const std::string &name, Environment *env, Parameters &&param)
    : Reactor(name, env), parameters{std::forward<Parameters>(param)} {}
    Worker(const std::string &name, Reactor *container, Parameters &&param)
    : Reactor(name, container), parameters{std::forward<Parameters>(param)} {}

    Input<int> req{"req", this};
    Output<int> rsp{"rsp", this};

    void construction() override {}

    void assembling() override {
        reaction("reaction_1").
            triggers(&startup).
            dependencies().
            effects().
            function(
                [&](Startup& startup) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                    << fqn() << " Startup\n";
                }
            );

        reaction("reaction_2").
            triggers(&req).
            dependencies().
            effects(&sch_rsp).
            function(
                [&](Input<int> &req, LogicalAction<int> &sch_rsp) {
                    auto req_ref = *req.get();
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Receiving task_id:" << req_ref << std::endl;
                    sch_rsp.schedule (req_ref, std::chrono::duration_cast<reactor::Duration>(std::chrono::nanoseconds(processing_delay)));
                }
            );

        reaction("reaction_3").
            triggers(&sch_rsp).
            dependencies().
            effects(&rsp).
            function(
                [&](LogicalAction<int> &sch_rsp, Output<int> &rsp) {
                    auto req_ref = *sch_rsp.get();
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Sending task_id:" << req_ref << std::endl;
                    rsp.set(req_ref);
                }
            );

        reaction("reaction_4").
            triggers(&shutdown).
            dependencies().
            effects().
            function(
                [&](Shutdown &shutdown) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Shutdown\n";
                }
            );
    }
};


class Pool : public Reactor {
public:
    struct Parameters {
        int n_workers = 1;
    };

private:
    Parameters parameters;
    const int &n_workers = parameters.n_workers;

    LogicalAction<int> sch_rsp{"sch_rsp", this};

    ReactorBank<Worker> workers{"workers", this};
    std::unique_ptr<Relay> relay;

public:
    Pool(const std::string &name, Environment *env, Parameters &&param)
    : Reactor(name, env), parameters{std::forward<Parameters>(param)} {}
    Pool(const std::string &name, Reactor *container, Parameters &&param)
    : Reactor(name, container), parameters{std::forward<Parameters>(param)} {}

    Input<int> req{"req", this};
    Output<int> rsp{"rsp", this};

    Output<bool> all_workers_busy{"all_workers_busy", this};

    void construction() override {
        for (int i = 0; i < n_workers; ++i) {
            workers.create_reactor(Worker::Parameters{.processing_delay = 1s});
        }
        relay = std::make_unique<Relay>("relay", this, Relay::Parameters{.n_outputs = n_workers});

    }

    void assembling() override {
        reaction("reaction_1").
            triggers(&startup).
            dependencies().
            effects().
            function(
                [&](Startup& startup) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Startup\n";
                }
            );

        reaction("reaction_2").
            triggers(&shutdown).
            dependencies().
            effects().
            function(
                [&](Shutdown &shutdown) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Shutdown\n";
                }
            );

        req --> relay->in_req;
        relay->out_req --> workers.for_each(select_default(workers).req);
        workers.for_each(select_default(workers).rsp) --> relay->in_rsp;
        relay->out_rsp --> rsp;

        relay->all_workers_busy --> all_workers_busy;
    }
};

class Tasks : public Reactor {
public:
    struct Parameters {
        int n_tasks = 10;
        int n_pools = 1;
    };

private:
    int req_itr = 0;
    int rsp_itr = 0;
    bool *busy = 0;

    Parameters parameters;
    const int &n_tasks = parameters.n_tasks;
    const int &n_pools = parameters.n_pools;
    
    LogicalAction<int> sch{"sch", this};

public:
    Tasks(const std::string &name, Environment *env, Parameters &&param)
    : Reactor(name, env), parameters{std::forward<Parameters>(param)} {}
    Tasks(const std::string &name, Reactor *container, Parameters &&param)
    : Reactor(name, container), parameters{std::forward<Parameters>(param)} {}

    MultiportOutput<int> req{"req", this};
    MultiportInput<int> rsp{"rsp", this};
    MultiportInput<bool> hybernate{"hybernate", this};

    void construction() override {
        req.set_width (n_pools);
        rsp.set_width (n_pools);
        hybernate.set_width (n_pools);
    }

    void assembling() override {
        reaction("reaction_1").
            triggers(&startup).
            dependencies().
            effects(&sch).
            function(
                [&](Startup& startup, LogicalAction<int> &sch) {
                    sch.schedule (-1, std::chrono::duration_cast<reactor::Duration>(std::chrono::nanoseconds(0)));
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Startup n_pools:" << n_pools << "\n";
                    busy = (bool*) calloc (n_pools, sizeof(bool));
                }
            );

        reaction("reaction_2").
            triggers(&sch).
            dependencies().
            effects(&req).
            function(
                [&](LogicalAction<int> &sch, MultiportOutput<int> &req) {
                    auto index = *sch.get();
                    if (index < 0) {
                        for (int i = 0; i < n_pools; ++i) {
                            if (busy[i]) {
                                std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                        << fqn() << " Busy Pool:" << i << std::endl;
                                continue;
                            }
                            std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                        << fqn() << " Sending task_id:" << req_itr << " to pool:" << i << std::endl;
                            req[i].set (req_itr++);
                        }

                        int busy_count = 0;
                        for (int i = 0; i < n_pools; ++i) {
                            busy_count = busy[i] ? (busy_count + 1) : busy_count;
                        }

                        if (busy_count == n_pools) {
                            return;
                        }
                        sch.schedule (-1, std::chrono::duration_cast<reactor::Duration>(std::chrono::nanoseconds(0)));
                    } else {
                        std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                    << fqn() << " Sending task_id:" << req_itr << " to pool:" << index << std::endl;
                        req[index].set (req_itr++);
                    }
                }
            );

            reaction("reaction_3").
            triggers(&rsp).
            dependencies().
            effects(&sch).
            function(
                [&](MultiportInput<int> &rsp, LogicalAction<int> &sch) {
                    for (int i = 0; i < n_pools; ++i) {
                        if (rsp[i].is_present()) {
                            ++rsp_itr;
                            if (req_itr < n_tasks) {
                                std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                            << fqn() << " Scheduling task_id:" << req_itr << " to pool:" << i << std::endl;
                                sch.schedule (i, std::chrono::duration_cast<reactor::Duration>(std::chrono::nanoseconds(0)));
                            }
                        }
                    }
                    if (rsp_itr == n_tasks) {
                        std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                    << "Terminating Run\n";
                        request_stop();
                    }
                }
            );

        reaction("reaction_4").
            triggers(&hybernate).
            dependencies().
            effects().
            function(
                [&](MultiportInput<bool> &hybernate) {
                    for (int i = 0; i < n_pools; ++i) {
                        if (hybernate[i].is_present()) {
                            busy[i] = *hybernate[i].get();
                        }
                    }
                }
            );

        reaction("reaction_5").
            triggers(&shutdown).
            dependencies().
            effects().
            function(
                [&](Shutdown &shutdown) {
                    std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                                << fqn() << " Shutdown\n";
                }
            );
    }
};

class Main : public Reactor {
public:
    struct Parameters {
        int n_tasks = 10;
        int n_pools = 2;
        int n_workers = 4;
    };

    Main(const std::string &name, Environment *env, Parameters &&param)
    : Reactor(name, env), parameters{std::forward<Parameters>(param)} {}
    Main(const std::string &name, Reactor *container, Parameters &&param)
    : Reactor(name, container), parameters{std::forward<Parameters>(param)} {}
private:
    Parameters parameters;
    const int &n_tasks = parameters.n_tasks;
    const int &n_pools = parameters.n_pools;
    const int &n_workers = parameters.n_workers;

    std::unique_ptr<Tasks> tasks;
    ReactorBank<Pool> pool{"pool", this};

public:
    void construction() override {
        tasks = std::make_unique<Tasks>("tasks", this, Tasks::Parameters{.n_tasks = n_tasks, .n_pools = n_pools});
        for (int i = 0; i < n_pools; ++i) {
            pool.create_reactor(Pool::Parameters{.n_workers = n_workers});
        }
    }

    void assembling() override {
        tasks->req --> pool.for_each(select_default(pool).req);
        pool.for_each(select_default(pool).rsp) --> tasks->rsp;
        pool.for_each(select_default(pool).all_workers_busy) --> tasks->hybernate;
    }
};

int main(int argc, char **argv) {
    Environment env {nullptr, 4, false, reactor::Duration::max(), false};

    int n_tasks = 10;
    int n_pools = 2;
    int n_workers = 4;

    auto main = new Main("Main", &env, Main::Parameters{.n_tasks = n_tasks, .n_pools = n_pools, .n_workers = n_workers});

    env.run();
    return 0;
}
