# Contributing

**Contact:** <christian.menard@tu-dresden.de> or <tassilo.tanneberger@tu-dresden.de>

## Programming Style

The project is currently implemented in C++17 and follows primarly the cpp-core-guidlines.
Please make sure you subbmitted code follows the .clang-tidy and .clang-format file.

### Testing
There are CI Tests which automatically check your code. If you want to perform the tests manually take a look
at this [guide](https://github.com/lf-lang/lingua-franca/wiki/Regression-Tests).

## Building with Nix

**Listing Compilers**
```
    $ nix run .#packages.x86_64-linux.list-compilers
```

**Building a speical Package**
```
    $ nix build .#packages.x86_64-linux.ActionDelay-gcc-wrapper-10-3-0 --override-input reactor-cpp github:lf-lang/reactor-cpp/cpp-core-guidelines
```

**Building all Packages**
```
    $ nix build .#packages.x86_64-linux.all-tests --override-input reactor-cpp github:lf-lang/reactor-cpp/cpp-core-guidelines
```


### Benchmarking
If youre changes are performance critically it is adviced to run the test from [here](https://github.com/lf-lang/lingua-franca/wiki/Running-Benchmarks)


## Git






