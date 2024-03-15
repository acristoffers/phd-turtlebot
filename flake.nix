{
  inputs = {
    flake-utils.url = github:numtide/flake-utils;
    nixpkgs.url = github:NixOS/nixpkgs/nixos-unstable;

    nvim.url = github:acristoffers/nvim-flake;
    nvim.inputs.nixpkgs.follows = "nixpkgs";
    nvim.inputs.flake-utils.follows = "flake-utils";
  };
  outputs = { nixpkgs, flake-utils, nvim, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; config.allowUnfree = true; };
      in
      {
        formatter = nixpkgs.legacyPackages.${system}.nixpkgs-fmt;
        packages = {
          default = pkgs.buildEnv {
            name = "home";
            paths = [
              pkgs.nix
              nvim.packages.${system}.default
            ];
          };
        };
      });
}
