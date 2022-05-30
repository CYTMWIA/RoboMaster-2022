SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cd ${SCRIPTPATH}/

echo $(pwd)

mkdir -p build && cd build

ln -s ../asset/* ./

cmake -DMODEL_RUNNER=openvino_yolox .. && make -j$(($(nproc) + 1))

if [ $? -eq 0 ] && [ "$1" == "--run" ]; then
    ./RMCV
fi