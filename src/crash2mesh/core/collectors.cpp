#include <crash2mesh/core/collectors.hpp>

namespace c2m
{
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

void Collector::markElement1DNodes()
{
    for (Element1D::Ptr elem: elements1D)
    {
        for (Node::Ptr node: elem->nodes)
        {
            node->referencingParts = std::numeric_limits<uint>::max();
        }
    }
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

}
