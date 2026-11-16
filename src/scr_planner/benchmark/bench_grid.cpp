// Copyright 2026 SCR Project
//
// Offline benchmark of OrientedAStar on a plain grid.
//
// usage: bench_grid <grid.pgm> <pairs.txt> <turn_penalty> [repeats=5] [paths_out]
//   grid.pgm   binary P5, pixel value is used directly as a Nav2 cost
//              (0 = free, >= 253 = blocked); row index == y.
//   pairs.txt  one "sx sy gx gy" per line (cells).
// Prints CSV to stdout; optionally writes the corner list of each path.

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "scr_planner/oriented_astar.hpp"

namespace
{

bool readPgm(const std::string & path, unsigned int & w, unsigned int & h, std::vector<uint8_t> & d)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return false;}
  std::string magic;
  f >> magic;
  if (magic != "P5") {return false;}
  auto next = [&f]() {
      std::string tok;
      while (f >> tok) {
        if (tok[0] == '#') {std::string rest; std::getline(f, rest); continue;}
        return std::stoi(tok);
      }
      return -1;
    };
  const int iw = next(), ih = next(), maxv = next();
  if (iw <= 0 || ih <= 0 || maxv != 255) {return false;}
  f.get();  // single whitespace after header
  w = iw;
  h = ih;
  d.resize(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char *>(d.data()), d.size());
  return static_cast<size_t>(f.gcount()) == d.size();
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "usage: bench_grid <grid.pgm> <pairs.txt> <turn_penalty> [repeats] [paths_out]\n";
    return 2;
  }
  unsigned int w = 0, h = 0;
  std::vector<uint8_t> data;
  if (!readPgm(argv[1], w, h, data)) {
    std::cerr << "cannot read grid " << argv[1] << "\n";
    return 1;
  }
  std::ifstream pf(argv[2]);
  std::vector<std::pair<scr_planner::Cell, scr_planner::Cell>> pairs;
  int sx, sy, gx, gy;
  while (pf >> sx >> sy >> gx >> gy) {pairs.push_back({{sx, sy}, {gx, gy}});}

  scr_planner::SearchParams p;
  p.turn_penalty = std::atof(argv[3]);
  p.cost_weight = 0.0;
  p.allow_unknown = false;
  const int repeats = argc > 4 ? std::max(1, std::atoi(argv[4])) : 5;
  std::ofstream paths_out;
  if (argc > 5) {paths_out.open(argv[5]);}

  scr_planner::GridView grid{w, h, data.data()};
  scr_planner::OrientedAStar planner;
  std::cout << "id,status,cost,length_cells,turns,corners,expansions,time_ms_median,time_ms_min\n";
  for (size_t i = 0; i < pairs.size(); ++i) {
    std::vector<double> times;
    scr_planner::SearchResult r;
    for (int k = 0; k < repeats; ++k) {
      r = planner.plan(grid, pairs[i].first, pairs[i].second, p);
      times.push_back(r.time_ms);
    }
    std::sort(times.begin(), times.end());
    const size_t len = r.cells.empty() ? 0 : r.cells.size() - 1;
    std::cout << i << ',' << scr_planner::toString(r.status) << ',' << r.cost << ',' << len << ','
              << r.turns << ',' << r.corners.size() << ',' << r.expansions << ','
              << times[times.size() / 2] << ',' << times.front() << '\n';
    if (paths_out) {
      for (size_t c = 0; c < r.corners.size(); ++c) {
        paths_out << (c ? ";" : "") << r.corners[c].x << ',' << r.corners[c].y;
      }
      paths_out << '\n';
    }
  }
  return 0;
}
