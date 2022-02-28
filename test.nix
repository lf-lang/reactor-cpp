{fetchFromGitHub, reactor-cpp-src, lingua-franca-src, pkgs, lib, stdenv}: 
let
mkDerivation = stdenv.mkDerivation;

# has file extension .lf there is one cmake file and that needs to filtered out
has_file_extensions_lf = file: lib.strings.hasSuffix ".lf" file;

# there are under lib/ lingua fracna programs that dont define an main reactor and therefore fail
is_regular_program = file: !(lib.strings.hasInfix "/lib/" file);

# functiosn combines the two rules above
filter_function = file: (is_regular_program file) && (has_file_extensions_lf file);

#  searches all lingua-franca files in that repo
tests = lib.filter filter_function (lib.filesystem.listFilesRecursive "${lingua-franca-src}/test/Cpp/src");

# extracts the name of the file without the .lf ending from a given file name
extract_name = file: builtins.head (builtins.split ".lf" (builtins.head (lib.reverseList (builtins.split "/" file))));

# given a set { name = ...; value = ... } it will retrun the derivation stored in value
extract_derivation = (attribute_set: attribute_set.value );

# list of special derivations which cannot be run
keep_alive = [
  "Keepalive.lf"
  "AsyncCallback.lf"
  "AsyncCallback2.lf"
];

# checks if a given file is in the unrunable file list
is_executable =  file: !((lib.lists.count (x: x == file) keep_alive) > 0);

# list of executable file names
executables = ( builtins.map extract_name (builtins.filter is_executable tests ) );

# executes all tests
execute_all = (lib.strings.concatStringsSep "\n" (builtins.map (x: "./${x}") executables));

# writes the execute command to file
run_all = (pkgs.writeScriptBin "run-all" ''
  #!${pkgs.runtimeShell}
'' + execute_all);

# downloading the cpp runtime
cpp-runtime = mkDerivation {
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
};

# function which build the derivation b
buildDerivation = (test_file: {
  name = extract_name test_file;   
  value = mkDerivation {
    name = extract_name test_file;

    src = ./.;

    buildInputs = with pkgs; [ lingua-franca which gcc cmake git boost ];

    configurePhase = ''
      echo "+++++ CURRENT TEST: ${test_file} +++++";
    '';

    buildPhase = ''
      mkdir -p include/reactor-cpp/
      cp -r ${cpp-runtime}/include/reactor-cpp/* include/reactor-cpp/
      ${pkgs.lingua-franca}/bin/lfc --external-runtime-path ${cpp-runtime}/ --output ./ ${test_file}
    '';

    installPhase = ''
      mkdir -p $out/bin
      cp -r ./bin/* $out/bin
    '';
  };
} );

# given a lists of files it will create a list of derivations
list_of_derivations = builtins.map extract_derivation (builtins.map buildDerivation tests);

# creates the copy command for every derivation
create_install_command = (lib.strings.concatStringsSep "\n" (builtins.map (x: "cp -r ${x}/bin/* $out/bin/") list_of_derivations ));

# package that triggers a build of every test file
ci_package = mkDerivation {
    src = ./.;
    name = "all-tests";
    buildInputs = list_of_derivations;
    installPhase = ''
      ${pkgs.coreutils}/bin/mkdir -p $out/bin
      ${pkgs.coreutils}/bin/cp -r ${run_all}/bin/run-all $out/bin/
    '' + create_install_command;
};


in lib.listToAttrs ( (builtins.map buildDerivation tests) ++ [ {name = "all-tests"; value = ci_package; } ] )

