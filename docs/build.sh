#!/bin/bash

declare -A versions=()
versions["refs/tags/v0.1.0"]="0.1.0"
versions["refs/tags/v0.2.0"]="0.2.0"
versions["test"]="latest"

git clone --single-branch --bare --branch master git@github.com:3LawsRobotics/3laws.git src
mkdir -p gh-pages
cd src

for branch in "${!versions[@]}"
do
  folder="${versions[$branch]}"
  git fetch origin $branch:$branch
  git worktree add -f "../$folder" $branch
  sphinx-build -W -b html -d ../$folder/_build/.doctrees ../$folder/docs/ ../gh-pages/en/$folder
  # sphinx-build -W -b latex -d ../$folder/_build/.doctrees ../$folder/docs/ ../$folder/_build/latex
  # make -C ../$folder/_build/latex all-pdf
  # cp ../$folder/_build/latex/3laws.pdf ../gh-pages/en/$folder/3laws.pdf
done

cp ../latest/docs/index.html ../gh-pages
