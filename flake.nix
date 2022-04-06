{
  description = "test and build script for the reactor-cpp runtime";

  inputs = {
    utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOs/nixpkgs/nixos-unstable";

    # input for the reactor-cpp
    reactor-cpp = {
      url = "github:lf-lang/reactor-cpp";
      flake = false;
    };

    # source for the lingu franca compiler
    lingua-franca-src = {
      url = "https://github.com/lf-lang/lingua-franca/releases/download/nightly/lfc_nightly_20220406-050300.tar.gz";
      flake = false;
    };

    # determines the the lingua-franca version from which the test are taken from
    lingua-franca-tests = {
      url = "github:lf-lang/lingua-franca";
      flake = false;
    };

    # repo that contains the benchmarks
    lingua-franca-benchmarks = {
      url = "github:lf-lang/benchmarks-lingua-franca";
      flake = false;
    };
  };

  outputs = inputs@{self, utils, nixpkgs, reactor-cpp, lingua-franca-src, lingua-franca-tests, lingua-franca-benchmarks, ...}: 
    utils.lib.eachDefaultSystem (system: let 
      pkgs = nixpkgs.legacyPackages.${system};
      allTests = (pkgs.callPackage ./nix/test.nix {
        reactor-cpp-src = reactor-cpp;
        lingua-franca-src = lingua-franca-src;
        lingua-franca-tests = lingua-franca-tests;
      });
      allBenchmarks = pkgs.callPackage ./nix/benchmark.nix {
        reactor-cpp-src = reactor-cpp;
        lingua-franca-src = lingua-franca-src;
        lingua-franca-benchmarks = lingua-franca-benchmarks;
      };
      in rec {
        checks = allTests;
        packages = pkgs.lib.mergeAttrs (pkgs.lib.mergeAttrs pkgs allTests) allBenchmarks;
      }
    );
}
