#ifndef C2M_COLLECTORS_HPP
#define C2M_COLLECTORS_HPP

#include <crash2mesh/core/mesh.hpp>
#include <crash2mesh/core/structure_elements.hpp>
#include <map>
#include <vector>

namespace c2m
{
/**
 * @brief Represents collections of atomic entities
 *
 */
class Collector
{
  public:
    using Ptr = std::shared_ptr<Collector>;

    // For comparison and ordering (compares ID)
    bool operator==(const Collector& other) const;
    bool operator!=(const Collector& other) const;
    bool operator<(const Collector& other) const;
    bool operator>(const Collector& other) const;

    std::vector<Element1D::Ptr> elements1D;           ///< All contained 1D elements
    std::vector<Element2D::Ptr> elements2D;           ///< All contained 2D elements
    std::vector<SurfaceElement::Ptr> surfaceElements; ///< All contained surface elements
    std::vector<Element3D::Ptr> elements3D;           ///< All contained 3D elements
    const entid_t entityID;                        ///< The unique ID identifying this collector

    void markElement1DNodes();

  protected:
    static entid_t maxID; ///< To track the entityID given to any newly created Collector

    /**
     * @brief Construct a new Collector given its contained atomic elements
     *
     * @param _elements1D contained 1D elements
     * @param _elements2D contained 2D elements
     * @param _surfaceElements contained surface elements
     * @param _elements3D contained 3D elements
     */
    Collector(const std::vector<Element1D::Ptr>& _elements1D,
              const std::vector<Element2D::Ptr>& _elements2D,
              const std::vector<SurfaceElement::Ptr>& _surfaceElements,
              const std::vector<Element3D::Ptr>& _elements3D);
};

class Part : public Collector
{
  public:
    using Ptr = std::shared_ptr<Part>;
    const partid_t ID;    ///< A part's identifier
    const entid_t userID; ///< The user-specified part-id
    CMesh mesh;
    MatX3 centers;

    /**
     * @brief Construct a new Collector given its part identifier and contained atomic elements
     *
     * @param _ID this parts identifier
     * @param _elements1D contained 1D elements
     * @param _elements2D contained 2D elements
     * @param _surfaceElements contained surface elements
     * @param _elements3D contained 3D elements
     */
    Part(partid_t _ID,
         entid_t _userID,
         const std::vector<Element1D::Ptr>& _elements1D = {},
         const std::vector<Element2D::Ptr>& _elements2D = {},
         const std::vector<SurfaceElement::Ptr>& _surfaceElements = {},
         const std::vector<Element3D::Ptr>& _elements3D = {});
};

/**
 * @brief Represents a collection of parts
 *
 */
class Scene
{
  public:
    using Ptr = std::shared_ptr<Scene>;

    std::vector<Part::Ptr> parts; //< References to the parts that form the scene

    CMesh mesh; //< Should be used to join separate part meshes into one mesh

    /**
     * @brief Merge parts into a scene
     *
     * @param _parts parts to merge into a scene
     */
    Scene(const std::vector<Part::Ptr> _parts) : parts(_parts)
    {
    }
};

} // namespace c2m

#endif
