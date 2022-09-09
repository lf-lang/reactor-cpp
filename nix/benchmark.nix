{ fetchFromGitHub
, pkgs
, lib
, stdenv
, lf-benchmark-runner
, reactor-cpp-src
, lingua-franca-src
, lingua-franca-benchmarks
, valgrind
, rev-reactor
, rev-lingua-franca
}:
let

  # custom library which supplies essential functionality
  library = pkgs.callPackage ./library.nix {
    lingua-franca-src = lingua-franca-src;
    reactor-cpp-src = reactor-cpp-src;
  };

  hashed-inputs = builtins.hashString "sha1" (rev-reactor + rev-lingua-franca);

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
  
  # package that builds all backages
  build-all-benchmarks = mkDerivation {
    src = ./.;
    name = "build-all-benchmarks";
    buildInputs = list_of_derivations;
    installPhase = ''
      ${pkgs.coreutils}/bin/mkdir -p $out/bin
    '' + install_command;
  };

  # runs our custom benchmark data extractor on the specified benchmark
  benchmark_command = (benchmark: "${lf-benchmark-runner}/bin/lf-benchmark-runner --target lf-cpp-${hashed-inputs} --binary ${benchmark}/bin/${benchmark.name} --file ./result.csv");

  # compiles a giant script to run every benchmark and collect their results into on csv file
  benchmark_commands = lib.strings.concatStringsSep "\n" (builtins.map benchmark_command list_of_derivations);

  # derivation defintion for running and collecting data from benchmarks
  make-benchmark = mkDerivation {
    src = ./.;
    name = "make-benchmark";

    buildPhase = benchmark_commands;

    installPhase = ''
      mkdir -p $out/data/
      cp result.csv $out/data/
    '';
  };

  individual-benchmark = (package: {
    name = "benchmark-${package.name}";
    value = mkDerivation {
      name = "benchmark-${package.name}";
      src = ./.;

      buildPhase = ''
        ${lf-benchmark-runner}/bin/lf-benchmark-runner --target lf-cpp --binary ${package}/bin/${package.name} --file ./result.csv --image result.svg
      '';

      installPhase = ''
        mkdir -p $out/data
        cp result.csv $out/data
      '';
    };
  });

  # derivation for call and cachegrind. measuring a given package
  profiler = (package: valgrind_check:
    {
      name = "${valgrind_check}-${package.name}";
      value = mkDerivation {
        name = "${valgrind_check}-${package.name}";
        src = ./.;
        nativeBuildInputs = [ valgrind ];

        buildPhase = ''
          ${valgrind}/bin/valgrind --tool=${valgrind_check} ${package}/bin/${package.name} 
        '';
        installPhase = ''
          mkdir -p $out/data
          cp ${valgrind_check}.out.* $out/data/${package.name}-${valgrind_check}.out
        '';
      };
    }
  );

  extract_derivations = (list: lib.attrValues (lib.listToAttrs list));
  attribute_set_derivations = (library.double_map benchmarks library.compilers library.buildDerivation);


  attribute_set_cachegrind = (builtins.map (x: profiler x "cachegrind") (extract_derivations attribute_set_derivations));
  attribute_set_callgrind = (builtins.map (x: profiler x "callgrind") (extract_derivations attribute_set_derivations));
  attribute_set_benchmarks = (builtins.map individual-benchmark (extract_derivations attribute_set_derivations));
  attribute_set_memory = (builtins.map library.memtest (extract_derivations attribute_set_derivations));
in
  lib.listToAttrs (
     attribute_set_derivations
  ++ attribute_set_cachegrind
  ++ attribute_set_callgrind
  ++ attribute_set_benchmarks
  ++ attribute_set_memory
  ++ [
  { name = "all-benchmarks"; value = all-benchmarks; }
  { name = "list-benchmarks"; value = list-benchmarks; }
  { name = "list-compilers"; value = library.list-compilers; }
  { name = "make-benchmark"; value = make-benchmark; }
])
