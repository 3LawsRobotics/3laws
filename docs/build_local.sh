#!/bin/bash

# Exit on error
set -e

# Prepare paths
srcDir="$(
  cd -- "$(dirname "$0")" >/dev/null 2>&1
  pwd -P
)"
version="latest"

# Clean output directory
rm -rf "$srcDir/.gh-pages"
rm -rf "$srcDir/build"

# Create output directory
mkdir -p "$srcDir/.gh-pages"
outDir="$srcDir/.gh-pages"
buildDir="$srcDir/build"
mkdir -p "$buildDir/$version/docs"

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

# Create doc for each version

export DOC_VERSION=$version

cp "$configFile" "$buildDir/$version/docs"
install -D "$versionsJson" "$buildDir/$version/docs/metadata/versions.json"
install -D "$versionsHtml" "$buildDir/$version/docs/_templates/versions.html"
workDir=$buildDir/$version/docs
mkdir -p "$workDir/_static"
cp -R "$srcDir/sources" "$workDir/sources"
cp -R "$srcDir/index.rst" "$workDir/index.rst"

sphinx-build -b html -d "$workDir/_build" "$workDir" "$outDir/en/$version"

# Copy and update main index.html

# Create .nojekyll file so that github pages doesn't complain about folders starting with '_'
touch "$outDir/.nojekyll"
