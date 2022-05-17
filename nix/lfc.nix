{ pkgs
, config
, lib
, mkDerivation
, jdk17_headless
, lingua-franca-src
}:
let
  # takes the first file in the jars repo which should be the relavant jar file
  jar_path = (builtins.head (builtins.attrNames (builtins.readDir "${lingua-franca-src}/lib/jars")));

  # filter out name
  extracted_name = (builtins.head (lib.reverseList (builtins.split "/" jar_path)));

in
mkDerivation {
  pname = "lfc";
  version = "0.1.0";

  src = lingua-franca-src;

  buildInputs = [ jdk17_headless ];

  _JAVA_HOME = "${jdk17_headless}/";

  postPatch = ''
    substituteInPlace bin/lfc \
      --replace 'base=`dirname $(dirname ''${abs_path})`' "base='$out'" \
      --replace "run_lfc_with_args" "${jdk17_headless}/bin/java -jar $out/lib/jars/${extracted_name}"
  '';

  buildPhase = ''
    echo "SKIPPING BUILDPHASE FOR LFC"
  '';

  installPhase = ''
    cp -r ./ $out/
    chmod +x $out/bin/lfc
  '';

  meta = with lib; {
    description = "Polyglot coordination language";
    longDescription = ''
      Lingua Franca (LF) is a polyglot coordination language for concurrent
      and possibly time-sensitive applications ranging from low-level
      embedded code to distributed cloud and edge applications.
    '';
    homepage = "https://github.com/lf-lang/lingua-franca";
    license = licenses.bsd2;
    platforms = platforms.linux;
    maintainers = with maintainers; [ revol-xut ];
  };
}
