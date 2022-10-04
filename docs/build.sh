#!/bin/bash
srcDir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
mkdir -p gh-pages
outDir="${PWD}/gh-pages"
mkdir -p build
buildDir="${PWD}/build"

configFile="${srcDir}/conf.py"
versionsFile="${srcDir}/metadata/versions.json"

echo "Source directory: ${srcDir}"
echo "Build directory: ${buildDir}"
echo "Output directory: ${outDir}"
echo "Config file: ${configFile}"
echo "Version file: ${versionsFile}"
echo ""

# Clone
if [ -d "src" ]
then
    rm -rf src
fi
git clone --single-branch --bare --branch master git@github.com:3LawsRobotics/3laws.git src
cd src

# Load versions
versionsRaw=$(cat ${versionsFile})
declare -A versions
versionsContent=$(jq -r '. | to_entries | .[] | "[\"" + .key + "\"]=" + (.value | @sh)' <<< "$versionsRaw")
versionsDef="versions=($versionsContent)"
eval "$versionsDef"

for branch in "${!versions[@]}"
do
  version="${versions[$branch]}"
  echo "Branch: ${branch}"
  echo "Version: ${version}"
  git fetch origin $branch:$branch
  git worktree add -f "$buildDir/$version" $branch
  cp $configFile $buildDir/$version/docs
  install -D $versionsFile $buildDir/$version/docs/metadata/versions.json
  workDir=$buildDir/$version/docs
  sphinx-build -W -b html -d $workDir/_build $workDir $outDir/en/$version
#   sphinx-build -W -b latex -d ../$folder/_build/.doctrees ../$folder/docs/ ../$folder/_build/latex
#   make -C ../$folder/_build/latex all-pdf
#   cp ../$folder/_build/latex/3laws.pdf ../gh-pages/en/$folder/3laws.pdf
done

cp ${srcDir}/index.html ${outDir}
