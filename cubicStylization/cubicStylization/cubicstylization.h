#pragma once

#include "localstep.h"
#include "globalstep.h"
#include "utils.h"
#include "Vertex.h"

#include <vector>

// Main cubic stylization function
// Takes input vertices from the selected mesh and updates mesh with deformed vertices
void cubicStylization(std::vector<Vertex>& Vi, float cubeness);