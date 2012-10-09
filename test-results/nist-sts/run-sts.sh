#! /bin/bash

cd $HOME/rng/sts-2.1.1
for i in $(seq --format="%03g" 0 9); do
  ./assess 1000000 <<EOF
0
../z/rand-$i.bin
1
0
1000
1
EOF
  mv experiments/AlgorithmTesting/finalAnalysisReport.txt experiments/AlgorithmTesting/finalAnalysisReport.txt-90B-1000-$i
done
