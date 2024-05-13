#!/bin/bash
set -ex


# This script is used to compute images

# define the build directory
build_dir=$1
input_file=$2
camera_file=$3

im_dir="./saved_images/"

method_list=("Plane (mean)" "Ellipsoid 3D" "FO2D" "CNC avg hexa" "ASO")
property_list=("Mean curvature" "Normals" "Shape index")

pointRadius=0.008
radius=0.028
kNN=15

# create the directory if it does not exist
mkdir -p $im_dir

# get absolute path of the input file, camera file and output directory
input_file=$(realpath $input_file)
camera_file=$(realpath $camera_file)
im_dir=$(realpath $im_dir)

# check if the build directory exists
if [ ! -d $build_dir ] ; then
    echo "Build directory does not exist"
    exit 1
fi

# check if the input file exists
if [ ! -f $input_file ] ; then
    echo "Input file does not exist"
    exit 1
fi

# check if the camera file exists
if [ ! -f $camera_file ] ; then
    echo "Camera file does not exist"
    exit 1
fi

# go to the build directory
cd "${build_dir}/bin"

# compute the images
for property in "${property_list[@]}" ; do
    for method in "${method_list[@]}" ; do
        echo "Computing images for method: $method"
        # if property is Shape index, then we put the minBound and maxBound to -1 and 1
        ./poncascope -i "${input_file}" -c "${camera_file}" -m "${method}" -o "${im_dir}/${method}_${property}.png" -r ${radius} --kNN ${kNN} --minBound "-10" --maxBound 10 --property "${property}"
    done
done