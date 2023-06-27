#!/bin/sh

script_path=cmake-build-debug-cmake-kitware/nbv_simulation
config_file=DefaultConfiguration.yaml
string_test_time=`date +"%Y-%m-%d-%H-%M"`
save_folder=../../../../media/alcoufr/These_Remy/results

for method in 10 11 101
# for method in 10
do
  for n_model in 0 1 2 3 4
  # for n_model in 0
  do
    for n_size in 1024 4096
    # for n_size in 1024
    do
      for n_iter in 1 2 3 4 5 10 30
      # for n_iter in 1
      do
#        echo ./$script_path $config_file $n_model $n_size $n_iter $method $string_test_time
        ./$script_path $config_file $n_model $n_size $n_iter $method $string_test_time $save_folder
      done
    done
  done
done