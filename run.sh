#!/bin/sh

script_path=cmake-build-debug-cmake-kitware/nbv_simulation
config_file=DefaultConfiguration.yaml
string_test_time=`date +"%Y-%m-%d-%H-%M"`

for method in 0 101 102
do
  for n_model in 0 1 2 3 4
  do
    for n_size in 1024 4096
    do
      for n_iter in 1 2 3 4 5
      do
#        echo ./$script_path $config_file $n_model $n_size $n_iter $method $string_test_time
        ./$script_path $config_file $n_model $n_size $n_iter $method $string_test_time
      done
    done
  done
done
