#ifndef C2M_SURFACE_EXTRACTOR_HPP
#define C2M_SURFACE_EXTRACTOR_HPP

#include <crash2mesh/core/collectors.hpp>
#include <crash2mesh/core/structure_elements.hpp>
#include <map>
#include <vector>

namespace c2m
{

/**
 * @brief Class to extract the surface elements of 3D volumetric elements partwise
 *
 */
class SurfaceExtractor
{
  public:
    /**
     * @brief Extract the surface elements of the given parts
     *
     * @param parts the parts to extract surface elements from
     * @param deleteVolumes whether 3D elements may be removed from the part after surface extraction
     * @return true if any surfaces could be extracted
     * @return false else
     */
    static bool extract(std::vector<Part::Ptr>& parts, bool deleteVolumes = true);

  private:
    /**
     * @brief Auxiliary comparator for faces represented as vectors of nodes.
     *
     */
    struct FaceCompare
    {
        /**
         * @brief Returns a < b
         *
         * @param a face A
         * @param b face B
         * @return true if size(\p a) < size(\p b) or after ordering the nodes for both, any Node::Ptr of \p a
         *               compares to less than that of \p b, starting from the beginning
         * @return false else
         */
        bool operator()(const std::vector<Node::Ptr>& a, const std::vector<Node::Ptr>& b) const;
    };
};
} // namespace c2m

#endif
