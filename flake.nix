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
  };

  outputs = inputs@{self, utils, nixpkgs, reactor-cpp, lingua-franca, ...}: 
    utils.lib.eachDefaultSystem (system: let 
      pkgs = nixpkgs.legacyPackages.${system};
      allTests = (pkgs.callPackage ./test.nix {
        reactor-cpp-src = reactor-cpp;
        lingua-franca-src = lingua-franca;
      });
      in rec {
        checks = allTests;
        packages = pkgs.lib.mergeAttrs pkgs allTests;
      }
    );
}
