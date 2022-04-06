{
  description = "test and build script for the reactor-cpp runtime";

  inputs = {
    utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOs/nixpkgs/nixos-unstable";
    reactor-cpp = {
      url = "github:lf-lang/reactor-cpp";
      flake = false;
    };
    lingua-franca = {
      url = "github:lf-lang/lingua-franca";
      flake = false;
    };
    lingua-franca-benchmarks = {
      url = "github:lf-lang/benchmarks-lingua-franca";
      flake = false;
    };

  };

  outputs = inputs@{self, utils, nixpkgs, reactor-cpp, lingua-franca, lingua-franca-benchmarks, ...}: 
    utils.lib.eachDefaultSystem (system: let 
      pkgs = nixpkgs.legacyPackages.${system};
      allTests = (pkgs.callPackage ./nix/test.nix {
        reactor-cpp-src = reactor-cpp;
        lingua-franca-src = lingua-franca;
      });
      allBenchmarks = pkgs.callPackage ./nix/benchmark.nix {
        reactor-cpp-src = reactor-cpp;
        lingua-franca-src = lingua-franca;
        lingua-franca-benchmark-src = lingua-franca-benchmarks;
      };
      in rec {
        checks = allTests;
        packages = pkgs.lib.mergeAttrs (pkgs.lib.mergeAttrs pkgs allTests) allBenchmarks;
      }
    );
}
