{ fetchFromGitHub
, lingua-franca-src
, reactor-cpp-src
, valgrind
, pkgs
, lib
, stdenv
}:
let
  # lingua-franca compiler package
  lingua-franca = pkgs.callPackage ./lfc.nix {
    lingua-franca-src = lingua-franca-src;
    mkDerivation = stdenv.mkDerivation;
  };

  # reactor-cpp runtime package 
  cpp-runtime = pkgs.callPackage ./reactor-cpp.nix {
    reactor-cpp-src = reactor-cpp-src;
    mkDerivation = stdenv.mkDerivation;
    debug = false;
  };

  # list of compiler for which packages are generated
  compilers = with pkgs; [ gcc11 gcc10 gcc9 clang_11 clang_12 clang_13 ];

  default_compiler = pkgs.gcc11;

  mkDerivation = stdenv.mkDerivation;

  # has file extension .lf there is one cmake file and that needs to filtered out
  has_file_extensions_lf = file: lib.strings.hasSuffix ".lf" file;

  # there are under lib/ lingua fracna programs that dont define an main reactor and therefore fail
  is_regular_program = file: !(lib.strings.hasInfix "/lib/" file);

  # checks if a given file is in the unrunable file list
  # TODO: this needs fixing
  is_borked = file: !((lib.strings.hasInfix "StructAsState.lf" file) || (lib.strings.hasInfix "StructAsType.lf" file) || (lib.strings.hasInfix "Struct" file));

  # functiosn combines the two rules above
  filter_function = file: (is_regular_program file) && (has_file_extensions_lf file) && (is_borked file);

  # extracts the name of the file without the .lf ending from a given file name
  extract_name = file: builtins.head (lib.strings.splitString ".lf" (builtins.head (lib.reverseList (builtins.split "/" file))));

  # given a set { name = ...; value = ... } it will retrun the derivation stored in value
  extract_derivation = (attribute_set: attribute_set.value);

  # format version of the format x.y.z to x-y-z
  fmt_version = (version: builtins.replaceStrings [ "." ] [ "-" ] "${version}");

  # list of available compilers
  list_of_compilers = (lib.strings.concatStringsSep "\n" (builtins.map (compiler: (''echo "${compiler.pname}-${fmt_version compiler.version}"'')) compilers));

  # writes a shell script which prints all the available compilers with their corresponding versions
  all-compilers-script = (pkgs.writeScriptBin "all-compilers" (''
    #!${pkgs.runtimeShell}

  '' + list_of_compilers));

  # function which build the derivation b
  buildDerivation = (test_file: compiler:
    let
      package_version = builtins.replaceStrings [ "." ] [ "-" ] "${compiler.version}";
      file_name = extract_name test_file;
      package_name = "${file_name}-${compiler.pname}-${package_version}";
    in
    {
      name = package_name;
      value = mkDerivation {
        name = package_name;

        src = ./.;

        # libgmp-dev is only required here because there is some special snowflake benchmark
        buildInputs = with pkgs; [ lingua-franca which cmake git boost gmp ] ++ [ compiler ];

        configurePhase = ''
          echo "+++++ CURRENT TEST: ${test_file} +++++";
        '';

        buildPhase = ''
          ${lingua-franca}/bin/lfc --external-runtime-path ${cpp-runtime}/ --output ./ ${test_file}
        '';

        installPhase = ''
          mkdir -p $out/bin
          mkdir -p $out/debug
          cp -r ./src-gen/* $out/debug/
          cp -r ./bin/${file_name} $out/bin/${file_name}-${compiler.pname}
          cp -r ./bin/${file_name} $out/bin/${package_name}
        '';
      };
    });
in
{
  compilers = compilers;
  buildDerivation = buildDerivation;
  mkDerivation = mkDerivation;
  has_file_extensions_lf = has_file_extensions_lf;

  # extracts the name of the file without the .lf ending from a given file name
  extract_name = file: builtins.head (lib.strings.splitString ".lf" (builtins.head (lib.reverseList (builtins.split "/" file))));

  # searches all lingua-franca files in that repo
  search_files = (path: lib.filter filter_function (lib.filesystem.listFilesRecursive path));

  # creates the cartesian product
  double_map = (list_1: list_2: func: lib.lists.flatten (builtins.map (x: builtins.map (y: func x y) list_2) list_1));

  # function which takes the set of files and computes a list of derivation out of those files
  create_derivations = (files: builtins.map extract_derivation (builtins.map (test: buildDerivation test default_compiler) files));

  # derivation which holds the script to list all available compilers
  list-compilers = mkDerivation {
    src = ./.;
    name = "list-compilers";
    installPhase = ''
      mkdir -p $out/bin/
      cp ${all-compilers-script}/bin/all-compilers $out/bin/list-compilers
    '';
  };

  # creates the copy command for every derivation
  create_install_command = (list_of_derivations: (lib.strings.concatStringsSep "\n" (builtins.map (x: "cp -r ${x}/bin/* $out/bin/") list_of_derivations)));

  # checks the given package for memory leaks and exports a the result
  memtest = (package:
    {
      name = "MLeaks-${package.name}";
      value = mkDerivation {
        name = "MLeaks-${package.name}";
        src = ./.;
        nativeBuildInputs = [ valgrind ];

        buildPhase = ''
          ${valgrind}/bin/valgrind --leak-check=full \
            --show-leak-kinds=all \
            --track-origins=yes \
            --verbose \
            --log-file=valgrind-out.txt \
            ${package}/bin/${package.name} 
        '';
        installPhase = ''
          mkdir -p $out/data
          cp valgrind-out.txt $out/data/${package.name}-memtest.out
        '';
      };
    }
  );
}
