#!/bin/bash
set -ex


# This script is used to compute images

# define the build directory
build_dir=$1

input_dir=$2
camera_dir=$3

im_dir="./saved_images/"

# CAD_helios
# surfaces=("00000002" "00000003" "00000004" "00000006" "00000008" "00000020" "00000022" "00000027" "00000030" "00000049" "00000050")
# cameras=(0 0 0 0 0 0 0 0 1 0 1)
# N=""

# CAD
surfaces=("0001" "0002" "0003" "0006" "1023")
cameras=(1 1 1 1 1)
N=""
property="--property real-normals"

# implicit 
# surfaces=("goursat" "goursat-hole" "tube" "selle" "torus" "paraboloid" "hyperboloid" "cylinder" "ellipsoid")
# cameras=(1 1 1 1 1 1 1 1 1 )
# N="_25000"

# create the directory if it does not exist
mkdir -p $im_dir

# get absolute path of the input file, camera file and output directory
input_dir=$(realpath $input_dir)
camera_dir=$(realpath $camera_dir)
im_dir=$(realpath $im_dir)

# check if the build directory exists
if [ ! -d $build_dir ] ; then
    echo "Build directory does not exist"
    exit 1
fi

# check if the input file exists
if [ ! -d $input_dir ] ; then
    echo "Input directory does not exist"
    exit 1
fi

# check if the camera file exists
if [ ! -d $camera_dir ] ; then
    echo "Camera directory does not exist"
    exit 1
fi

# go to the build directory
cd "${build_dir}/bin"

for ((i=0; i<${#surfaces[@]}; i++)); do
    does_have_camera=${cameras[i]}
    surface_name=${surfaces[i]}${N}

    if [ $does_have_camera -eq 0 ]; then
        ./poncascope -i "${input_dir}/${surface_name}.pts" -o "${im_dir}/${surface_name}.png"
    else
        camera_file="camera${surface_name}.json"
        ./poncascope -i "${input_dir}/${surface_name}.pts" -c "${camera_dir}/${camera_file}" -o "${im_dir}/${surface_name}.png"
    fi

    # compute the images
    
done