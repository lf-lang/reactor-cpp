{ pkgs, mkDerivation, cmake, gcc, reactor-cpp-src }:
mkDerivation {
  name = "cpp-lingua-franca-runtime";
  src = reactor-cpp-src;

  nativeBuildInputs = with pkgs; [ cmake gcc ];

  configurePhase = ''
    echo "Configuration"
  '';
  
  #TODO: remove debug build here
  buildPhase = ''
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=./ ../
    make install
  '';

  installPhase = ''
    cp -r ./ $out/
  '';

  fixupPhase = ''
    echo "FIXUP PHASE SKIP"
  '';
}
