{ pkgs, mkDerivation, cmake, gcc, lingua-franca, reactor-cpp-src }:
mkDerivation {
  name = "cpp-lingua-franca-runtime";
  src = reactor-cpp-src;

  nativeBuildInputs = with pkgs; [ cmake gcc lingua-franca ];

  configurePhase = ''
    echo "Configuration"
  '';

  buildPhase = ''
    mkdir -p build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=./ ../
    make install
  '';

  installPhase = ''
    cp -r ./ $out/
  '';

  fixupPhase = ''
    echo "FIXUP PHASE SKIP"
  '';
}
