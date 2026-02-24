#!/bin/sh
# Goppert's submission script for ray.
# usage: ./submit.sh <ARCHIVE NAME>

# Make a temp solution directory.
mkdir ray

# Copy important things into it.
cp -r src ray
cp -r cmake ray
cp -r ecm1 ray
cp -r ecm2 ray
cp README.md ray
cp CMakeLists.txt ray

# Make archive and clean up.
tar -czvf $1.tgz ray
rm -rf ray