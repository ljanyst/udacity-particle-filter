//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <vector>

struct Map {
  struct Landmark {
    int    id;
    double x;
    double y;
  };
  std::vector<Landmark> landmarks;
};
