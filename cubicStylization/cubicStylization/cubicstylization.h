#pragma once

#include "args.h"
#include "localstep.h"
#include "globalstep.h"
#include "utils.h"
#include "Vertex.h"

#include "maya/MMatrix.h"

#include <chrono>
#include <stdio.h>
#include <vector>

// Main cubic stylization function
// Takes input vertices from the selected mesh and updates mesh with deformed vertices
void cubicStylization(std::vector<Vertex>& Vi, commandArgs &args);