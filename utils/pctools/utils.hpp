//
// Created by Masayuki IZUMI on 5/11/16.
//
#include <pcl/console/parse.h>
#include <pcl/console/print.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace bfs = boost::filesystem;
namespace pc = pcl::console;

namespace pctools {

void shutdown_with_error(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  pc::print_error(format, ap);
  va_end(ap);
  exit(-1);
}

void each_files(std::string path, std::function<void(bfs::directory_entry)> proc) {
  for (auto file : boost::make_iterator_range(bfs::directory_iterator(path), bfs::directory_iterator())) {
    proc(file);
  }
}
}

