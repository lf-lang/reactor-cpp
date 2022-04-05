# Contributing

**Contact:** <christian.menard@tu-dresden.de> or <tassilo.tanneberger@tu-dresden.de>

## Programming Style

The project is currently implemented in C++17 and follows primarly the cpp-core-guidlines.
Please make sure you subbmitted code follows the .clang-tidy and .clang-format file.

### Testing
There are CI Tests which automatically check your code. If you want to perform the tests manually take a look
at this [guide](https://github.com/lf-lang/lingua-franca/wiki/Regression-Tests).

## Building and Testing with Nix

**Listing Compilers**
```
    $ nix run .#packages.x86_64-linux.list-compilers
```

**Listing all available Tests**
```
    $  nix run .#packages.x86_64-linux.list-tests
```

**Building a special Package**
```
    $ nix build .#packages.x86_64-linux.ActionDelay-gcc-wrapper-10-3-0 --override-input reactor-cpp github:lf-lang/reactor-cpp/cpp-core-guidelines
```

The important thing to note is that the `--override-input` flag can take literally any source. In this example it takes the `cpp-core-guidleines` branch 
but you maybe also want to use your fork then the argument would look like this `--override-input reactor-cpp github:revol-xut/reactor-cpp`.

**Building all Packages**
```
    $ nix run .#packages.x86_64-linux.all-tests
```

This will build and run every test.


### Benchmarking
If youre changes are performance critically it is adviced to run the test from [here](https://github.com/lf-lang/lingua-franca/wiki/Running-Benchmarks)


## Git

There are no strict conventions on how your git messages need to be formatted just know they should be informative and summersing on your changes. 




