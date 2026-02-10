#!/bin/sh
# Goppert's submission script for ray.
# usage: ./submit.sh <ARCHIVE NAME>

# Make a temp solution directory.
mkdir ray

# Copy important things into it.
cp -r src ray
cp -r cmake ray
cp -r extra_credit_mtgs ray
cp -r extra_credit_output ray
cp -r extra_credit_scenes ray
cp README.md ray
cp CMakeLists.txt ray

# Make archive and clean up.
tar -czvf $1.tgz ray
rm -rf ray