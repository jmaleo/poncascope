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
float radius;
int mls_iter;
int kNN;
int researchType = 1; // 0 : k Nearest Neighbors, 1 : Euclidian Nearest Neighbors