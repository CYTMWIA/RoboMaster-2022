SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cd ${SCRIPTPATH}/

mkdir -p build && cd build

ln -s ../asset/* ./

cmake .. && make -j$(($(nproc) + 1)) && ./RMCV
