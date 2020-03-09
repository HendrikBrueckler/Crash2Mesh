#ifndef C2M_MESH_HPP
#define C2M_MESH_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/decimater/modules/normal_cone.hpp>
#include <OpenMesh/Core/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/QuadricT.hh>

#include <list>

namespace c2m
{

struct MeshTraits : public OpenMesh::DefaultTraits
{
    VertexAttributes(OpenMesh::Attributes::Status);
    EdgeAttributes(OpenMesh::Attributes::Status);
    HalfedgeAttributes(OpenMesh::Attributes::Status);
    FaceAttributes(OpenMesh::Attributes::Status);

    VertexTraits
    {
        Node::Ptr node; //< Finite Element node belonging to a vertex
        bool fixed; //< Whether this vertex is a fixated fixed non-manifold vertex
        typename Refs::VertexHandle duplicate; //< Cyclic fixed non-manifold vertex list
        std::vector<Quadric> quadrics; //< Error quadrics accumulated in QuadricDecimationModule
    };

    FaceTraits
    {
        Element2D::Ptr element;
        std::list<Element2D::Ptr> additionalElements;
        std::vector<NormalCone> normalCones; //< Error quadrics accumulated in QuadricDecimationModule
    };
};

using CMesh = OpenMesh::TriMesh_ArrayKernelT<MeshTraits>;
using VHandle = CMesh::VertexHandle;
using FHandle = CMesh::FaceHandle;
using HEHandle = CMesh::HalfedgeHandle;

}
#endif
