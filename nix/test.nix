{ fetchFromGitHub
, pkgs
, lib
, stdenv
, reactor-cpp-src
, lingua-franca-src
, lingua-franca-tests
}:
let

  # custom library which supplies essential functionality
  library = pkgs.callPackage ./library.nix {
    lingua-franca-src = lingua-franca-src;
    reactor-cpp-src = reactor-cpp-src;
  };

  # list of special derivations which cannot be run
  keep_alive = [
    "Keepalive.lf"
    "AsyncCallback.lf"
    "AsyncCallback2.lf"
  ];

  mkDerivation = library.mkDerivation;


  #  searches all lingua-franca files in that repo
  tests = (library.search_files "${lingua-franca-tests}/test/Cpp/src");

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

  # copies all the binaries
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

  # function that takes a list of attributes [ { name = "a"; value = "b";}] and transforms it into
  # a list with all the values which are derivations in that case: e.g. ["b"]
  extract_derivations = (list: lib.attrValues (lib.listToAttrs list));

  # takes the cartisean product of packages and compilers
  attribute_set_derivations = (library.double_map tests library.compilers library.buildDerivation);

  # creates for every package a memtest version for debug purposes
  attribute_set_memory = (builtins.map library.memtest (extract_derivations attribute_set_derivations));
in
lib.listToAttrs (attribute_set_derivations
  ++ attribute_set_memory
  ++ [
  { name = "all-tests"; value = all-tests; }
  { name = "list-tests"; value = list-tests; }
  { name = "list-compilers"; value = library.list-compilers; }
])

