#!/usr/bin/env bash

dataset=(
  "euroc"
  "tum"
)

img_freq=(
  "5"
  "10"
  "20"
)

td_set=(
  "50"
  "40"
  "30"
  "20"
  "10"
  "-10"
  "-20"
  "-30"
  "-40"
  "-50"
)

all_start_time="$(date -u +%s)"

# Loop through all datasets
for i in "${!dataset[@]}"; do

for j in "${!img_freq[@]}"; do

for k in "${!td_set[@]}"; do

# start timing
start_time="$(date -u +%s)"

# run our bin file (note we send console output to terminator)
roslaunch dataset_convertor read_kalibr.launch \
  dataset_name:=${dataset[i]} \
  img_freq:=${img_freq[j]} \
  td:=${td_set[k]} &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${dataset[i]} ${img_freq[j]} ${td_set[k]} took $elapsed seconds";

done

done

done

# print out the time elapsed
all_end_time="$(date -u +%s)"
all_elapsed="$(($all_end_time-$all_start_time))"
all_elapsed_min=$(bc -l <<< "scale=3; ($all_elapsed)/60.0")
echo "BASH: run all bags took $all_elapsed_min min";
