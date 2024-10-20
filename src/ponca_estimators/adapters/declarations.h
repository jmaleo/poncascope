#pragma once

#include "types.h"
#include "DifferentialQuantities.hpp"
#include "fitDefinitions.h"
#include "poncaAdapters.hpp"

// Global variables
// [TODO] Need to clean this up

KdTree ponca_kdtree;
KnnGraph* ponca_knnGraph;
bool use_kNNGraph;

MyVoxelGrid voxelGrid;
bool use_VoxelGrid;
int N_voxels;
int resolution;

int iVertexSource;
int kNN_for_graph;
Scalar radius;
int mls_iter;
int kNN;
