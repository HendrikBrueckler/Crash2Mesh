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

Node::Node(nodeid_t _ID, const MatX3& _positions)
    : FiniteElement(erfh5::FEType::NODE, ID_NULL), ID(_ID), positions(_positions),
      referencingParts(0)
{
}

ConnectedElement::ConnectedElement(const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes, const std::vector<bool>& _active)
    : FiniteElement(_type, _partID), nodes(_nodes), active(_active)
{
}

Element1D::Element1D(elemid_t _ID, const erfh5::FEType& _type, partid_t _partID, const std::vector<Node::Ptr>& _nodes, const std::vector<bool>& _active)
    : ConnectedElement(_type, _partID, _nodes, _active), elem1dID(_ID)
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
                     const std::vector<bool>& _active,
                     float _plasticStrain0,
                     const VecX& _plasticStrains)
    : ConnectedElement(_type, _partID, _nodes, _active), elem2dID(_ID), plasticStrains(_plasticStrains), plasticStrain0(_plasticStrain0)
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
                     const std::vector<bool>& _active,
                     float _ePlasticStrain0,
                     const VecX& _ePlasticStrains)
    : ConnectedElement(_type, _partID, _nodes, _active), elem3dID(_ID), ePlasticStrains(_ePlasticStrains), ePlasticStrain0(_ePlasticStrain0)
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
    : Element2D(_ID, _type, _partID, _nodes, _volume->active, _volume->ePlasticStrain0, _volume->ePlasticStrains), surfaceElemID(_ID), volume(_volume)
{
}

const std::vector<const erfh5::FEType*> SurfaceElement::allTypes({&erfh5::FEType::SURFACE3, &erfh5::FEType::SURFACE4});

} // namespace c2m
