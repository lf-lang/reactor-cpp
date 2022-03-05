{fetchFromGitHub, reactor-cpp-src, lingua-franca-src, pkgs, lib, stdenv}: 
let

# list of compiler for which packages are generated
compilers = with pkgs; [ gcc10 gcc9 gcc8 clang_11 clang_12 clang_13 ];

default_compiler = pkgs.gcc;

mkDerivation = stdenv.mkDerivation;

# has file extension .lf there is one cmake file and that needs to filtered out
has_file_extensions_lf = file: lib.strings.hasSuffix ".lf" file;

# there are under lib/ lingua fracna programs that dont define an main reactor and therefore fail
is_regular_program = file: !(lib.strings.hasInfix "/lib/" file);

borked_tests = [
  "StructAsState.lf"
  "StructAsType.lf"
];

# checks if a given file is in the unrunable file list
is_borked =  file: !((lib.strings.hasInfix "StructAsState.lf" file) || (lib.strings.hasInfix "StructAsType.lf" file) || (lib.strings.hasInfix "Struct" file));

# functiosn combines the two rules above
filter_function = file: (is_regular_program file) && (has_file_extensions_lf file) && (is_borked file);

#  searches all lingua-franca files in that repo
tests = (lib.filter filter_function (lib.filesystem.listFilesRecursive "${lingua-franca-src}/test/Cpp/src"));

# extracts the name of the file without the .lf ending from a given file name
extract_name = file: builtins.head (lib.strings.splitString ".lf" (builtins.head (lib.reverseList (builtins.split "/" file))));

# given a set { name = ...; value = ... } it will retrun the derivation stored in value
extract_derivation = (attribute_set: attribute_set.value );

# format version of the format x.y.z to x-y-z
fmt_version = (version: builtins.replaceStrings [ "." ] ["-"] "${version}");

# list of special derivations which cannot be run
keep_alive = [
  "Keepalive.lf"
  "AsyncCallback.lf"
  "AsyncCallback2.lf"
];

generate_package_name = (test: compiler: let 
  version = builtins.replaceStrings [ "." ] ["-"] "${compiler.version}";
  name = extract_name test;
  package_name = "${name}-${compiler.pname}-${version}";
in "${name}-${compiler.pname}-${version}" );

# creates the cartesian product
double_map = (list_1: list_2: func: lib.lists.flatten (builtins.map (x: builtins.map (y: func x y) list_2) list_1 ));

# list of available compilers
list_of_compilers = (lib.strings.concatStringsSep "\n" (builtins.map (compiler: (''echo "${compiler.pname}-${fmt_version compiler.version}"'')) compilers));

# writes a shell script which prints all the available compilers with their corresponding versions
all-compilers-script = (pkgs.writeScriptBin "all-compilers" (''
  #!${pkgs.runtimeShell}

'' + list_of_compilers));

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
buildDerivation = (test_file: compiler: 
let
  package_version = builtins.replaceStrings [ "." ] [ "-" ] "${compiler.version}";
  file_name = extract_name test_file;
  package_name = "${file_name}-${compiler.pname}-${package_version}";
in{
  name = package_name;
  value = mkDerivation {
    name = package_name;

    src = lingua-franca-src;

    buildInputs = with pkgs; [ lingua-franca which cmake git boost ] ++ [ compiler ];

    configurePhase = ''
      echo "+++++ CURRENT TEST: ${test_file} +++++";
    '';

    buildPhase = ''
      cd test/Cpp/src
      mkdir -p include/reactor-cpp/
      cp -r ${cpp-runtime}/include/reactor-cpp/* include/reactor-cpp/
      ${pkgs.lingua-franca}/bin/lfc --external-runtime-path ${cpp-runtime}/ --output ./ ${test_file}
    '';

    installPhase = ''
      mkdir -p $out/bin
      cp -r ./bin/${file_name} $out/bin/${package_name}
    '';
  };
} );

# given a lists of files it will create a list of derivations
list_of_derivations = builtins.map extract_derivation (builtins.map (test: buildDerivation test default_compiler) tests);

# checks if a given file is in the unrunable file list
is_executable =  file: !((lib.lists.count (x: x == file) keep_alive) > 0);

# list of executable file names
executables = (builtins.filter is_executable tests );

executables_derivations = (builtins.map extract_derivation (builtins.map (test: buildDerivation test default_compiler) executables));

# executes all tests
execute_all = (lib.strings.concatStringsSep "\n" (builtins.map (x: "${x}/bin/${x.name}") executables_derivations));

# writes the execute command to file
run_all = (pkgs.writeScriptBin "all-tests" (''
  #!${pkgs.runtimeShell}

'' + execute_all));


# creates the copy command for every derivation
create_install_command = (lib.strings.concatStringsSep "\n" (builtins.map (x: "cp -r ${x}/bin/* $out/bin/") list_of_derivations ));

# package that triggers a build of every test file
ci_package = mkDerivation {
    src = ./.;
    name = "all-tests";
    buildInputs = list_of_derivations;
    installPhase = ''
      ${pkgs.coreutils}/bin/mkdir -p $out/bin
      ls ${run_all}/
      ${pkgs.coreutils}/bin/cp ${run_all}/bin/* $out/bin/
    '' + create_install_command;
};

list-compilers = mkDerivation {
  src = ./.;
  name = "list-compilers";
  installPhase = ''
    mkdir -p $out/bin/
    cp ${all-compilers-script}/bin/all-compilers $out/bin/list-compilers
  '';
};


in lib.listToAttrs ((double_map tests compilers buildDerivation) ++ 
[ 
  {name = "all-tests"; value = ci_package; } 
  {name = "list-compilers"; value = list-compilers;}
] )

