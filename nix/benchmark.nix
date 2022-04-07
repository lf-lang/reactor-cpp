{ fetchFromGitHub
, pkgs
, lib
, stdenv
, reactor-cpp-src
, lingua-franca-src
, lingua-franca-benchmarks
}:
let

  # custom library which supplies essential functionality
  library = pkgs.callPackage ./library.nix {
    lingua-franca-src = lingua-franca-src;
    reactor-cpp-src = reactor-cpp-src;
  };

  mkDerivation = library.mkDerivation;

  # Borked Benchmarks or not even Benchmarks
  borked_benchmarks = [
    "BenchmarkRunner.lf"
  ];

  # filter out name
  extract_name = (file: builtins.head (lib.reverseList (builtins.split "/" file)));

  borked_benchmarks_filter = (file: !(lib.lists.elem (extract_name file) borked_benchmarks));

  filter_function = (file: (borked_benchmarks_filter file) && (library.has_file_extensions_lf file));

  # searches all lingua-franca files in that repo
  benchmarks = (lib.filter filter_function (library.search_files "${lingua-franca-benchmarks}/Cpp/Savina/src"));

  list_of_derivations = (library.create_derivations benchmarks);

  # executes all benchmarks
  execute_all = (lib.strings.concatStringsSep "\n" (builtins.map (x: "${x}/bin/${x.name}") list_of_derivations));

  # writes the execute command to file
  run_all = (pkgs.writeScriptBin "all-benchmarks" (''
    #!${pkgs.runtimeShell}

  '' + execute_all));

  # string of all benchmarks
  list_of_benchmarks = (lib.strings.concatStringsSep "\n" (builtins.map library.extract_name benchmarks));

  # script of all benchmarks
  all-benchmarks-script = (pkgs.writeScriptBin "all-benchmarks" (list_of_benchmarks + "\n"));

  # derivation which hold the script
  list-benchmarks = mkDerivation {
    src = ./.;
    name = "list-benchmarks";
    installPhase = ''
      mkdir -p $out/bin
      echo "cat ${all-benchmarks-script}/bin/all-benchmarks" > $out/bin/list-benchmarks
      chmod +x $out/bin/list-benchmarks
    '';
  };

  # created a concatination of copy commands to collect them in one area
  install_command = (library.create_install_command (library.create_derivations benchmarks));

  # package that triggers a build of every test file
  all-benchmarks = mkDerivation {
    src = ./.;
    name = "all-benchmarks";
    buildInputs = list_of_derivations;
    installPhase = ''
      ${pkgs.coreutils}/bin/mkdir -p $out/bin
      ${pkgs.coreutils}/bin/cp ${run_all}/bin/* $out/bin/
    '' + install_command;
  };
in
lib.listToAttrs ((library.double_map benchmarks library.compilers library.buildDerivation) ++
  [
    { name = "all-benchmarks"; value = all-benchmarks; }
    { name = "list-benchmarks"; value = list-benchmarks; }
    { name = "list-compilers"; value = library.list-compilers; }
  ])

