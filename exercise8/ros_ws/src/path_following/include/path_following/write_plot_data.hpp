#ifndef WRITE_PLOT_DATA_HPP
#define WRITE_PLOT_DATA_HPP

#include <fstream>
#include <stdexcept>

namespace write_plot_data {

struct PlotDataWriter {
  std::ofstream out;

  PlotDataWriter(const std::string &path) {
    // Open in append mode so we don't keep truncating on each node restart
    out.open(path, std::ios::out | std::ios::app);
    if (!out.is_open()) {
      throw std::ios_base::failure("Couldn't open data file '"+path+"' for writing");
    }
  }

  void write(double measured, double desired, double error) {
    out << measured << " " << desired << " " << error << "\n";
    // std::endl would flush too, but explicit flush is okay if you really want:
    out.flush();
  }

  void close() {
    out.close();
  }
};

} // namespace write_plot_data

#endif // WRITE_PLOT_DATA_HPP
