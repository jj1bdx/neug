# /bin/bash

# The size of 125000-byte was chosen for the classic NIST STS test
# It's historical.

for i in $(seq -f "%03.0f" $1 $2); do
  if [ ! -f rand-$i.bin ]; then
    echo $i
    dd if=/dev/ttyACM0 count=1000 ibs=125000 iflag=fullblock of=rand-$i.bin
  fi
done
