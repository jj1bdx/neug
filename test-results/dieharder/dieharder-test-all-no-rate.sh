#! /bin/sh

ARG_STDIN='-g 200'
OUTPUT_FORMAT='-D test_name -D ntuple -D tsamples -D psamples -D pvalues -D assessment'
SEQ="`seq 0 17` `seq 100 102`"

date

dieharder | grep '^#'

cat - <<'EOF'
        test_name   |ntup| tsamples |psamples|  p-value |Assessment
#=============================================================================#
EOF

for i in $SEQ; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d $i
done

for n in `seq 1 12`; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d 200 -n $n
done

for n in `seq 2 5`; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d 201 -n $n
done

for n in `seq 2 5`; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d 202 -n $n
done

for n in `seq 0 32`; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d 203 -n $n
done

for i in `seq 204 209`; do
  cat rand*.bin | dieharder $ARG_STDIN $OUTPUT_FORMAT -d $i
done

date
