A Reactor-Oriented Programming Framework in C++
---

![CI](https://github.com/tud-ccc/reactor-cpp/workflows/CI/badge.svg)

While reactor-cpp can be used as a standalone framework, it is designed to work
in conjunction with [Lingua Franca](https://github.com/icyphy/lingua-franca/),
a polyglot metaprogramming language. Have a look at the Lingua Franca
[wiki](https://github.com/icyphy/lingua-franca/wiki) to get an overview of the
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

## Extras

reactor-cpp can be build with [tracing support](https://github.com/tud-ccc/reactor-cpp/tree/master/tracing). This provides a powerful tool for analyzing and debugging reactor applications.

## Publications

* **DATE'20:** Christian Menard, Andrés Goens, Marten Lohstroh, Jeronimo Castrillon, [Achieving Determinism in Adaptive AUTOSAR](https://arxiv.org/pdf/1912.01367), Proceedings of the 2020 Design, Automation and Test in Europe Conference (DATE), EDA Consortium, Mar 2020.

Also see the Lingua Franca [publications](https://github.com/icyphy/lingua-franca/wiki/Publications-and-Presentations).
