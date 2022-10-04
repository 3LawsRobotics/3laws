#!/bin/bash
srcDir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
mkdir -p gh-pages
outDir="$PWD/gh-pages"
mkdir -p build
buildDir="$PWD/build"

configFile="$srcDir/conf.py"
versionsJson="$srcDir/metadata/versions.json"
versionsHtml="$srcDir/_templates/versions.html"

echo "Source directory: $srcDir"
echo "Build directory: $buildDir"
echo "Output directory: $outDir"
echo "Config file: $configFile"
echo "Version JSON: $versionsJson"
echo "Version HTML: $versionsHtml"
echo ""

# Clone
if [ -d "src" ]
then
    rm -rf src
fi
git clone --single-branch --bare --branch master git@github.com:3LawsRobotics/3laws.git src
cd src

# Load versions
versionsRaw=$(cat $versionsJson)
declare -A versions
versionsContent=$(jq -r '. | to_entries | .[] | "[\"" + .key + "\"]=" + (.value | @sh)' <<< "$versionsRaw")
versionsDef="versions=($versionsContent)"
eval "$versionsDef"

for branch in "${!versions[@]}"
do
  version="${versions[$branch]}"

  echo "Branch: $branch"
  echo "Version: $version"

  export DOC_VERSION=$version

  git fetch origin $branch:$branch
  git worktree add -f "$buildDir/$version" $branch
  cp $configFile $buildDir/$version/docs
  install -D $versionsJson $buildDir/$version/docs/metadata/versions.json
  install -D $versionsHtml $buildDir/$version/docs/_templates/versions.html
  workDir=$buildDir/$version/docs
  sphinx-build -W -b html -d $workDir/_build $workDir $outDir/en/$version
  sphinx-build -W -b latex -d $workDir/_build/ $workDir $workDir/_build/latex
  make -C $workDir/_build/latex all-pdf
  cp $workDir/_build/latex/3laws.pdf $outDir/en/$version/3laws_manual.pdf
done

cp $srcDir/index.html $outDir
touch $outDir/.nojekyll
