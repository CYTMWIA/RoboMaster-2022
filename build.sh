SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

cd ${SCRIPTPATH}/

echo $(pwd)

mkdir -p build && cd build

ln -s ../asset/* ./

cmake -DMODEL_RUNNER=openvino_yolox .. && make -j$(($(nproc) + 1))

if [ $? -eq 0 ] && [ "$1" == "--run" ]; then
    while true
    do
        ./RMCV
        ./usbreset $(lsusb | grep Dahua | awk 'NR==1{gsub("[^0-9]", " ", $0); print "/dev/bus/usb/"$1"/"$2}')
        ./usbreset $(lsusb | grep Daheng | awk 'NR==1{gsub("[^0-9]", " ", $0); print "/dev/bus/usb/"$1"/"$2}')
        sleep 5
    done
fi