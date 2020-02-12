#ifndef C2M_C2M_WRITER_HPP
#define C2M_C2M_WRITER_HPP

#include <crash2mesh/core/collectors.hpp>

namespace c2m
{

// TODO
class C2MWriter
{
  public:
    static void write(const std::string& filename, const Scene::Ptr& scene, bool binary = true);
};

} // namespace c2m

#endif
