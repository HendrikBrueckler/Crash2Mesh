#include <crash2mesh/core/structure_elements.hpp>

namespace c2m
{
entid_t FiniteElement::maxID = 0;

FiniteElement::FiniteElement(const erfh5::FEType& _type, partid_t _partID)
    : type(_type), partID(_partID), entityID(++maxID)
{
}

bool FiniteElement::operator==(const FiniteElement& other) const
{
    return other.entityID == entityID;
}
bool FiniteElement::operator!=(const FiniteElement& other) const
{
    return !(other == *this);
}
bool FiniteElement::operator<(const FiniteElement& other) const
{
    return other.entityID < entityID;
}
bool FiniteElement::operator>(const FiniteElement& other) const
{
    return other < *this;
}

Node::Node(nodeid_t _ID, const Vec3& _coord, const std::vector<Vec3>& _displacements)
    : FiniteElement(erfh5::FEType::NODE, ID_NULL), ID(_ID), coord(_coord), displacements(_displacements),
      referencingParts(0)
{
}

ConnectedElement::ConnectedElement(const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes)
    : FiniteElement(_type, _partID), nodes(_nodes)
{
}

Element1D::Element1D(elemid_t _ID, const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes)
    : ConnectedElement(_type, _partID, _nodes), elem1dID(_ID)
{
}

const std::vector<const erfh5::FEType*> Element1D::allTypes({
    &erfh5::FEType::BEAM,
    &erfh5::FEType::BAR,
    &erfh5::FEType::SPRING6DOF,
    &erfh5::FEType::SPHERICALJOINT,
    &erfh5::FEType::FLEXTORSJOINT,
    &erfh5::FEType::KJOINT,
    &erfh5::FEType::PLINK,
    &erfh5::FEType::MBSJOINT,
    &erfh5::FEType::MBSSPRING,
    &erfh5::FEType::SPRINGBEAM,
    &erfh5::FEType::DRAWBEAD,
    &erfh5::FEType::MTOJN,
    &erfh5::FEType::MUSCLE,
    &erfh5::FEType::JET,
    &erfh5::FEType::MPCPLINK,
    &erfh5::FEType::GAP,
    &erfh5::FEType::TIED,
});

Element2D::Element2D(elemid_t _ID,
                     const erfh5::FEType& _type,
                     partid_t _partID,
                     const std::vector<Node::Ptr>& _nodes,
                     const std::vector<float>& _plasticStrains)
    : ConnectedElement(_type, _partID, _nodes), elem2dID(_ID), plasticStrains(_plasticStrains)
{
}

const std::vector<const erfh5::FEType*> Element2D::allTypes({&erfh5::FEType::MEMBR,
                                                             &erfh5::FEType::THICKSHELL,
                                                             &erfh5::FEType::SHELL,
                                                             &erfh5::FEType::SHEL6,
                                                             &erfh5::FEType::SHEL8,
                                                             &erfh5::FEType::SHEL3});

Element3D::Element3D(elemid_t _ID,
                     const erfh5::FEType& _type,
                     partid_t _partID,
                     const std::vector<Node::Ptr>& _nodes,
                     const std::vector<float>& _ePlasticStrains)
    : ConnectedElement(_type, _partID, _nodes), elem3dID(_ID), ePlasticStrains(_ePlasticStrains)
{
}

const std::vector<const erfh5::FEType*> Element3D::allTypes({&erfh5::FEType::HEXA8,
                                                             &erfh5::FEType::BRICKSHELL,
                                                             &erfh5::FEType::TETRA10,
                                                             &erfh5::FEType::TETRA4,
                                                             &erfh5::FEType::PENTA6,
                                                             &erfh5::FEType::PENTA15,
                                                             &erfh5::FEType::HEXA20});

SurfaceElement::SurfaceElement(elemid_t _ID,
                               const erfh5::FEType& _type,
                               partid_t _partID,
                               const std::vector<Node::Ptr>& _nodes,
                               Element3D::Ptr _volume)
    : ConnectedElement(_type, _partID, _nodes), surfaceElemID(_ID), volume(_volume)
{
}

const std::vector<const erfh5::FEType*> SurfaceElement::allTypes({&erfh5::FEType::SURFACE3, &erfh5::FEType::SURFACE4});

entid_t Collector::maxID = 0;

Collector::Collector(const std::vector<Element1D::Ptr>& _elements1D,
                     const std::vector<Element2D::Ptr>& _elements2D,
                     const std::vector<SurfaceElement::Ptr>& _surfaceElements,
                     const std::vector<Element3D::Ptr>& _elements3D)
    : elements1D(_elements1D.begin(), _elements1D.end()), elements2D(_elements2D.begin(), _elements2D.end()),
      surfaceElements(_surfaceElements.begin(), _surfaceElements.end()),
      elements3D(_elements3D.begin(), _elements3D.end()), entityID(++maxID)
{
}

bool Collector::operator==(const Collector& other) const
{
    return other.entityID == entityID;
}
bool Collector::operator!=(const Collector& other) const
{
    return !(other == *this);
}
bool Collector::operator<(const Collector& other) const
{
    return other.entityID < entityID;
}
bool Collector::operator>(const Collector& other) const
{
    return other < *this;
}

Part::Part(partid_t _ID,
           const std::vector<Element1D::Ptr>& _elements1D,
           const std::vector<Element2D::Ptr>& _elements2D,
           const std::vector<SurfaceElement::Ptr>& _surfaceElements,
           const std::vector<Element3D::Ptr>& _elements3D)
    : Collector(_elements1D, _elements2D, _surfaceElements, _elements3D), ID(_ID)
{
}

} // namespace c2m
