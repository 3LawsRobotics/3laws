#!/bin/bash

declare -a versions=("v0.1.0" "v0.2.0")

# git clone --single-branch --bare --branch master git@github.com:3LawsRobotics/3laws.git src
mkdir -p gh-pages
cd src

for v in "${versions[@]}"
do
  git fetch origin "refs/tags/$v":"refs/tags/$v"
  git worktree add -f "../$v" "refs/tags/$v"
  sphinx-build -W -b html -d $v/_build/.doctrees ../$v/docs/ ../gh-pages/en/$v
done

# sphinx-build -b html . html
