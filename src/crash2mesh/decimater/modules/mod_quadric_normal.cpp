#include <crash2mesh/decimater/modules/mod_quadric_normal.hpp>

#include <crash2mesh/util/logger.hpp>

namespace c2m
{

/// Constructor
ModQuadricNormal::ModQuadricNormal(CMesh& _mesh, float _max_err, float _max_dev) : ModBase(_mesh, false), ModQuadric(_mesh), ModNormal(_mesh)
{
    ModQuadric::set_max_err(_max_err);
    ModQuadric::set_area_weighting(false);
    ModNormal::set_max_normal_deviation(_max_dev);
}

/// Destructor
ModQuadricNormal::~ModQuadricNormal()
{
}

void ModQuadricNormal::initialize()
{
    ModQuadric::initialize();
    ModNormal::initialize();

    bbox_min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    bbox_max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (VHandle v: mesh_.vertices())
    {
        const OMVec3& pv = mesh_.point(v);
        bbox_min[0] = std::min(bbox_min[0], pv[0]);
        bbox_min[1] = std::min(bbox_min[1], pv[1]);
        bbox_min[2] = std::min(bbox_min[2], pv[2]);
        bbox_max[0] = std::max(bbox_max[0], pv[0]);
        bbox_max[1] = std::max(bbox_max[1], pv[1]);
        bbox_max[2] = std::max(bbox_max[2], pv[2]);
    }
    Vec3 diag = bbox_max - bbox_min;
    bbox_diag = diag.norm();
    bbox_vol = diag[0] * diag[1] * diag[2];
}

float ModQuadricNormal::collapse_priority(const CollapseInfo& _ci)
{
    float priorityQuadric = ModQuadric::collapse_priority(_ci);
    if (priorityQuadric == Base::ILLEGAL_COLLAPSE)
        return Base::ILLEGAL_COLLAPSE;
    float priorityNormal = ModNormal::collapse_priority(_ci);
    if (priorityNormal == Base::ILLEGAL_COLLAPSE)
        return Base::ILLEGAL_COLLAPSE;

    if (ModQuadric::area_weighting())
    {
        // 1/1000 th of bbox volume quadric error should be as important as 2° normal deviation
        float prioQ = priorityQuadric / bbox_vol * 1000.0f;
        float prioN = priorityNormal /  M_PI * 180.0f * 0.5f;
        return prioQ + prioN;
    }
    else
    {
        // 1/1000 th of bbox diagonal quadric error should be as important as 2° normal deviation
        float prioQ = priorityQuadric / bbox_diag * 1000.0f;
        float prioN = priorityNormal /  M_PI * 180.0f * 0.5f;
        return prioQ + prioN;
    }
}

void ModQuadricNormal::set_error_tolerance_factor(double _factor)
{
    ModQuadric::set_error_tolerance_factor(_factor);
    ModNormal::set_error_tolerance_factor(_factor);
}

void ModQuadricNormal::postprocess_collapse(const CollapseInfo& _ci)
{
    ModQuadric::postprocess_collapse(_ci);
    ModNormal::postprocess_collapse(_ci);
}

} // namespace c2m
