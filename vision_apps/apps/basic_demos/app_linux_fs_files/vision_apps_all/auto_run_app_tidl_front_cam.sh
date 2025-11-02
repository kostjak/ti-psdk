cd /ti_fs/vision_apps 

PROCESSOR=`uname -a`
if [[ ${PROCESSOR} == *_"J722S"_* ]]; then
    k3conf write 0x000f4114 0x50007
    gpio -m0 -n68 -o -h
fi

. ./vision_apps_init.sh
sleep 8 
( sleep 3; echo "e11" ) | ./run_app_tidl_front_cam.sh

