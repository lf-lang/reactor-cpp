{ fetchFromGitHub
, reactor-cpp-src
, lingua-franca-src
, pkgs
, lib
, stdenv
}:
let

  # custom library which supplies essential functionality
  library = pkgs.callPackage ./library.nix {
    lingua-franca-src = lingua-franca-src;
    reactor-cpp-src = reactor-cpp-src;
  };

  borked_tests = [
    "StructAsState.lf"
    "StructAsType.lf"
  ];

  # list of special derivations which cannot be run
  keep_alive = [
    "Keepalive.lf"
    "AsyncCallback.lf"
    "AsyncCallback2.lf"
  ];

  mkDerivation = library.mkDerivation;


  #  searches all lingua-franca files in that repo
  tests = (library.search_files "${lingua-franca-src}/test/Cpp/src");

  # checks if a given file is in the unrunable file list
  is_executable = file: !((lib.lists.count (x: x == file) keep_alive) > 0);

  # list of executable file names
  executables = (builtins.filter is_executable tests);

  list_of_derivations = (library.create_derivations tests);

  executables_derivations = (library.create_derivations executables);

  # executes all tests
  execute_all = (lib.strings.concatStringsSep "\n" (builtins.map (x: "${x}/bin/${x.name}") executables_derivations));

  # writes the execute command to file
  run_all = (pkgs.writeScriptBin "all-tests" (''
    #!${pkgs.runtimeShell}

  '' + execute_all));

  # string of all tests
  list_of_tests = (lib.strings.concatStringsSep "\n" (builtins.map library.extract_name tests));

  # script of all tests
  all-tests-script = (pkgs.writeScriptBin "all-tests" (list_of_tests + "\n"));

  # derivation which hold the script
  list-tests = mkDerivation {
    src = ./.;
    name = "list-tests";
    installPhase = ''
      mkdir -p $out/bin
      echo "cat ${all-tests-script}/bin/all-tests" > $out/bin/list-tests
      chmod +x $out/bin/list-tests
    '';
  };

  install_command = (library.create_install_command (library.create_derivations tests));

  # package that triggers a build of every test file
  all-tests = mkDerivation {
    src = ./.;
    name = "all-tests";
    buildInputs = list_of_derivations;
    installPhase = ''
      ${pkgs.coreutils}/bin/mkdir -p $out/bin
      ${pkgs.coreutils}/bin/cp ${run_all}/bin/* $out/bin/
    '' + install_command;
  };
in
lib.listToAttrs ((library.double_map tests library.compilers library.buildDerivation) ++
  [
    { name = "all-tests"; value = all-tests; }
    { name = "list-tests"; value = list-tests; }
    { name = "list-compilers"; value = library.list-compilers; }
  ])

