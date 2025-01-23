set -e

# rm -rf bin lib
mkdir -p bin lib

ROOT=`pwd`

sudo apt install libeigen3-dev

cd lib
git clone https://github.com/raisimTech/raisimlib
git clone https://github.com/ethz-asl/sampling_based_control
