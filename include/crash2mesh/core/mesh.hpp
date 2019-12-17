#ifndef C2M_MESH_HPP
#define C2M_MESH_HPP

#include <crash2mesh/core/structure_elements.hpp>
#include <OpenMesh/Core/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

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
        Node::Ptr node;
        bool fixed;
    };

    FaceTraits
    {
        Element2D::Ptr element;
    };
};

using CMesh = OpenMesh::TriMesh_ArrayKernelT<MeshTraits>;
using VHandle = CMesh::VertexHandle;
using FHandle = CMesh::FaceHandle;
using HEHandle = CMesh::HalfedgeHandle;
}
#endif
