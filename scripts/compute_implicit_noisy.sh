#!/bin/bash
set -ex


# This script is used to compute images

# define the build directory
build_dir=$1
input_assets_dir=$2
camera_dir=$3
im_out_dir=$4

im_dir="./saved_images/implicit_noisy/"

method_list=("Plane (mean)" "Ellipsoid 3D" "FO2D" "CNC avg hexa" "ASO")
property_list=("Mean curvature" "Normals" "Shape index")

pointRadius=0.008

N=10000
shapes=("goursat" "torus")
index_by_shape=("8566 2186 5936" "989 7885 9132" "2800 815 5104")
property="Neighbors"

radii="0.2 0.1 0.05"

noiseP="0.0025 0.015"

# create the directory if it does not exist
mkdir -p $im_out_dir

# get absolute path of the input file, camera file and output directory
camera_dir=$(realpath $camera_dir)
im_out_dir=$(realpath $im_out_dir)
input_assets_dir=$(realpath $input_assets_dir)

# check if the build directory exists
if [ ! -d $build_dir ] ; then
    echo "Build directory does not exist"
    exit 1
fi

# check if the camera directory exists
if [ ! -d $camera_dir ] ; then
    echo "Camera directory does not exist"
    exit 1
fi

# check if the input directory exists
if [ ! -d $input_assets_dir ] ; then
    echo "Input directory does not exist"
    exit 1
fi

# go to the build directory
cd "${build_dir}/bin"

# for each shape indices
for i in $(seq 0 $((${#shapes[@]}-1))) ; do
    shape=${shapes[$i]}
    indices=${index_by_shape[$i]}
    echo "Computing neighbors for shape: $shape, index: $index, radius: $radius"
    ./poncascope -i "${input_assets_dir}/${shape}_${N}.pts" -c "${camera_dir}/cameraSettings_${shape}.json" -o "${im_out_dir}/${shape}_${property}.png" --property "${property}" --vertexQuery ${indices} -r ${radii} --vertexRadius ${pointRadius}

    for noise in $noiseP ; do
        echo "Computing images for shape: $shape, noise: $noise"
        ./poncascope -i "${input_assets_dir}/${shape}_${N}.pts" -c "${camera_dir}/cameraSettings_${shape}.json" -o "${im_out_dir}/${shape}_noise_${noise}.png" --noisePosition ${noise} --vertexRadius ${pointRadius}
    done

done