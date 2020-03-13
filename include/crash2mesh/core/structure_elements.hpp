#ifndef C2M_STRUCTURE_ELEMENTS_HPP
#define C2M_STRUCTURE_ELEMENTS_HPP

#include <crash2mesh/core/types.hpp>
#include <crash2mesh/io/erfh5/file_contents.hpp>

#include <set>
#include <vector>

namespace c2m
{

/**
 * @brief Represents any N-Dimensional atomic element (0D to 3D)
 *
 */
class FiniteElement
{
  public:
    using Ptr = std::shared_ptr<FiniteElement>;
    /**
     * @brief Get the dimension of derived elements
     *
     * @return int dimension of derived elements
     */
    virtual int dim() = 0;

    const erfh5::FEType& type; ///< The type associated with any finite element
    const partid_t partID;     ///< The part any finite element belongs to (may be ID_NULL)
    const entid_t entityID;    ///< The unique ID identifying any finite element

    // For comparison and ordering (compares ID)
    bool operator==(const FiniteElement& other) const;
    bool operator!=(const FiniteElement& other) const;
    bool operator<(const FiniteElement& other) const;
    bool operator>(const FiniteElement& other) const;

  protected:
    static entid_t maxID; ///< To track the entityID given to any newly created FiniteElement
    /**
     * @brief Construct a new Finite Element with given type and belonging to given part
     *
     * @param _type type
     * @param _partID id of containing part
     */
    FiniteElement(const erfh5::FEType& _type, partid_t _partID);
};

/**
 * @brief Represents 0-dimensional points
 *
 */
class Node : public FiniteElement
{
  public:
    using Ptr = std::shared_ptr<Node>;
    const static int DIM = 0;
    virtual int dim()
    {
        return DIM;
    }

    /**
     * @brief Construct a new Node given a node identifier and positions
     *
     * @param _ID node identifier
     * @param positions 3D positions
     */
    Node(nodeid_t _ID, const MatX3& _positions);
    const nodeid_t ID;     ///< A node's ID (not the same as FiniteElement::entityID)
    MatX3 positions;       ///< A node's positions
    uint referencingParts; ///< Number of parts referencing this vertex
};

/**
 * @brief Represents any higher dimensional atomic elements created from connecting nodes
 *
 */
class ConnectedElement : public FiniteElement
{
  public:
    using Ptr = std::shared_ptr<ConnectedElement>;
    virtual int dim() = 0;
    std::vector<Node::Ptr> nodes; ///< This elements connected nodes
    std::vector<bool> active;     ///< Whether the element is active or inactive

  protected:
    /**
     * @brief Construct a new Connected Element given its type, its containing part identifier
     *        and its connected nodes
     *
     * @param _type type
     * @param _partID part identifier
     * @param _nodes connected nodes
     */
    ConnectedElement(const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes, const std::vector<bool>& _active);
};

/**
 * @brief Represents 1D connected atomic elements
 *
 */
class Element1D : public ConnectedElement
{
  public:
    using Ptr = std::shared_ptr<Element1D>;
    const static int DIM = 1;
    virtual int dim()
    {
        return DIM;
    }
    const static std::vector<const erfh5::FEType*> allTypes; ///< Contains all valid types for Element1D
    const elemid_t elem1dID;                                 ///< Identifier (unique among Element1Ds)

    /**
     * @brief Construct a new Element1D given its identifier, type, containing part and connected nodes
     *
     * @param _ID element1D identifier
     * @param _type type
     * @param _partID containing part identifier
     * @param _nodes connected nodes
     */
    Element1D(elemid_t _ID, const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes, const std::vector<bool>& _active);
};

/**
 * @brief Represents 2D connected atomic elements
 *
 */
class Element2D : public ConnectedElement
{
  public:
    using Ptr = std::shared_ptr<Element2D>;
    const static int DIM = 2;
    virtual int dim()
    {
        return DIM;
    }
    const static std::vector<const erfh5::FEType*> allTypes; ///< Contains all valid types for Element2D
    const elemid_t elem2dID;                                 ///< Identifier (unique among Element2Ds)
    const VecX plasticStrains;                               ///< Plastic strain timeseries
    const float plasticStrain0;                              ///< Initial Plastic strain

    /**
     * @brief Construct a new Element2D given its identifier, type, containing part, connected nodes
     *        and plastic strain time series.
     *
     * @param _ID element2D identifier
     * @param _type type
     * @param _partID containing part identifier
     * @param _nodes connected nodes
     * @param _plasticStrain0 initial plastic strain
     * @param _plasticStrains plastic strain time series
     */
    Element2D(elemid_t _ID,
              const erfh5::FEType& _type,
              partid_t _partID,
              const std::vector<Node::Ptr>& _nodes,
              const std::vector<bool>& _active,
              float _plasticStrain0,
              const VecX& _plasticStrains);
};

/**
 * @brief Represents 3D connected atomic elements
 *
 */
class Element3D : public ConnectedElement
{
  public:
    using Ptr = std::shared_ptr<Element3D>;
    const static int DIM = 3;
    virtual int dim()
    {
        return DIM;
    }
    const static std::vector<const erfh5::FEType*> allTypes; ///< Contains all valid types for Element3D
    const elemid_t elem3dID;                                 ///< Identifier (unique among Element3Ds)
    const VecX ePlasticStrains;                              ///< equivalent plastic strain timeseries
    const float ePlasticStrain0;                             ///< Initial Plastic strain

    /**
     * @brief Construct a new Element3D given its identifier, type, containing part, connected nodes
     *        and equivalent plastic strain time series.
     *
     * @param _ID element3D identifier
     * @param _type type
     * @param _partID containing part identifier
     * @param _nodes connected nodes
     * @param _ePlasticStrain0 initial plastic strain
     * @param _ePlasticStrains equivalent plastic strain time series
     */
    Element3D(elemid_t _ID,
              const erfh5::FEType& _type,
              partid_t _partID,
              const std::vector<Node::Ptr>& _nodes,
              const std::vector<bool>& _active,
              float _ePlasticStrain0,
              const VecX& _ePlasticStrains);
};

/**
 * @brief Represents atomic 2D surface elements extracted from 3D volume elements
 *
 */
class SurfaceElement : public Element2D
{
  public:
    using Ptr = std::shared_ptr<SurfaceElement>;
    const static int DIM = 2;
    virtual int dim()
    {
        return DIM;
    }
    const static std::vector<const erfh5::FEType*> allTypes; ///< Contains all valid types for SurfaceElements
    const elemid_t surfaceElemID;                            ///< Identifier (unique among SurfaceElements)
    const Element3D::Ptr volume;                             ///< The volume belonging to this surface element

    /**
     * @brief Construct a new SurfaceElement given its identifier, type, containing part, connected nodes
     *        and the volume element it belongs to.
     *
     * @param _ID SurfaceElement identifier
     * @param _type type
     * @param _partID containing part identifier
     * @param _nodes connected nodes
     * @param _volume volume element this belongs to
     */
    SurfaceElement(elemid_t _ID,
                   const erfh5::FEType& _type,
                   partid_t _partID,
                   const std::vector<Node::Ptr>& _nodes,
                   Element3D::Ptr _volume);
};

} // namespace c2m

#endif
