A Reactor-Oriented Programming Framework in C++
---

![CI](https://github.com/lf-lang/reactor-cpp/workflows/CI/badge.svg)

> [!IMPORTANT]  
> This repository is currently not actively maintained and developed as the core contributors focus on other projects. Please check out the [Xronos SDK](https://github.com/xronos-inc/xronos) which uses a fork of this runtime and [reactor-uc](https://github.com/lf-lang/reactor-uc) which applies the same underlying ideas to embedded devices.

While reactor-cpp can be used as a standalone framework, it is designed to work
in conjunction with [Lingua Franca](https://github.com/lf-lang/lingua-franca/),
a polyglot metaprogramming language. Read the Lingua Franca [handbook](https://www.lf-lang.org/docs/) 
to get an overview of the reactor model. 

## Build

```sh
mkdir build
cd build
cmake ..
make
```

The examples need to be built explicitly.
Alternatively, take a look at the [CONTRIBUTING.md](CONTRIBUTING.md) for building with nix package manager.

## Extras
reactor-cpp can be built with [tracing support](https://github.com/lf-lang/reactor-cpp/tree/master/tracing).
This provides a powerful tool for analyzing and debugging reactor applications.

## Contributing

For general guidelines about contributing, see [CONTRIBUTING.md](CONTRIBUTING.md).

<!---
## Documentation

A live version of the latest commit on master is automatically deployed and
available [online](https://lf-lang.github.io/reactor-cpp/index.html). For tests
and other purposes, you can also build the documentation locally.

First doxygen needs to be installed. On Ubuntu or Debian, run:
Further, we need several Python packages. Assuming that both python3 and pip3
are set up, run
```sh
pip3 --user install -r doc/requirements.txt
```
If python3 is the default on your system, run:
```sh
pip --user install -r doc/requirements.txt
```

Now we can build the documentation with:
```sh
make html
```

This will output the project documentation in `doc`/build/html`.
-->

## Publications

* **Phd Thesis:** Christian Menard, ["Deterministic Reactive Programming for Cyber-physical Systems"](https://nbn-resolving.org/urn:nbn:de:bsz:14-qucosa2-916872), PhD thesis, TU Dresden, 205 pp., Jun 2024. 
* **TACO'23:** Christian Menard, Marten Lohstroh, Soroush Bateni, Matthew Chorlian, Arthur Deng, Peter Donovan, Clément Fournier, Shaokai Lin, Felix Suchert, Tassilo Tanneberger, Hokeun Kim, Jeronimo Castrillon, and Edward A. Lee. 2023. ["High-performance Deterministic Concurrency Using Lingua Franca"](https://doi.org/10.1145/3617687). ACM Transaction on Architecure and Code Optimization, Volume 20, Issue 4, Article 48 (December 2023), 29 pages. 
* **DATE'20:** Christian Menard, Andrés Goens, Marten Lohstroh, Jeronimo Castrillon, ["Achieving Determinism in Adaptive AUTOSAR"](https://doi.org/10.23919/DATE48585.2020.9116430), 2020 Design, Automation & Test in Europe Conference & Exhibition (DATE), Grenoble, France, 2020, pp. 822-827.

More related publications are available on the Lingua Franca [publication page](https://www.lf-lang.org/research).
