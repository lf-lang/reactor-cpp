A Reactor-Oriented Programming Framework in C++
---

![CI](https://github.com/lf-lang/reactor-cpp/workflows/CI/badge.svg)

While reactor-cpp can be used as a standalone framework, it is designed to work
in conjunction with [Lingua Franca](https://github.com/lf-lang/lingua-franca/),
a polyglot metaprogramming language. Have a look at the Lingua Franca
[wiki](https://github.com/lf-lang/lingua-franca/wiki) to get an overview of the
Reactor model. If you are working with AUTOSAR Adaptive Platform (AP), also
have a look at the [DEAR framework](https://github.com/tud-ccc/dear), which in
conjunction with reactor-cpp allows to implement AP Services using the reactor
model.


## Build

```sh
mkdir build
cd build
cmake ..
make
```

The examples need to be built explicitly.

```
make examples
```

Alternatively take a look at the [CONTRIBUTING.md](CONTRIBUTING.md) for building with nix package manager.

## Extras

reactor-cpp can be build with [tracing support](https://github.com/lf-lang/reactor-cpp/tree/master/tracing). This provides a powerful tool for analyzing and debugging reactor applications.

## Contributing

For general guidelines about contributing, see [CONTRIBUTING.md](CONTRIBUTING.md).

## Documentation

A live version of the latest commit on master is automatically deployed and
available [online](https://lf-lang.github.io/reactor-cpp/index.html). For tests
and other purposes, you can also build the documentation locally.

First doxygen needs to be installed. On Ubuntu or Debian, run:
```sh
sudo apt-get install doxygen
```

Further we need several python packages. Assuming that both python3 and pip3
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

This will output the project documentaion in `doc/build/html`.

## Publications

* **DATE'20:** Christian Menard, Andrés Goens, Marten Lohstroh, Jeronimo Castrillon, [Achieving Determinism in Adaptive AUTOSAR](https://arxiv.org/pdf/1912.01367), Proceedings of the 2020 Design, Automation and Test in Europe Conference (DATE), EDA Consortium, Mar 2020.

Also see the Lingua Franca [publications](https://github.com/lf-lang/lingua-franca/wiki/Publications-and-Presentations).
