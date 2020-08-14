#!/bin/bash

# Quick and dirty script to move all the plots I might want into my thesis images folder and generate eps versions

cd ../plots || failed

names=(
  different_DO
  heuristics_trial
  long_horizon
  pf_vs_planner
  short_turning_radius
  slow_no_slow_1
)

thesis_path=~/Documents/thesis

for n in "${names[@]}"; do
  cd "$n" || failed
  for i in ./*; do
    cp "$i" $thesis_path/images/
    cd ~/Documents/thesis/images || failed
    inkscape "$i" --export-type="eps" --export-overwrite "${i%.pdf}.eps"
    cd - || failed
  done
  cd ..
done

function failed() {
  echo "Failed."
  exit
}
