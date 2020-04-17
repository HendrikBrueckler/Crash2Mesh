#include <crash2mesh/decimater/modules/mod_quadric.hpp>

#include <crash2mesh/core/structure_elements.hpp>
#include <crash2mesh/util/logger.hpp>

namespace c2m
{
using std::vector;

#ifdef C2M_PROB_QUADRICS
Mat3 ModQuadric::crossProductMatrix(const Vec3& v) const
{
    Mat3 m;
    m << 0,      -v[2],   v[1],
         v[2],   0,       -v[0],
         -v[1],   v[0],   0;
    return m;
}

Mat3 ModQuadric::crossInterferenceMatrix(const Mat3& A, const Mat3& B) const
{
    Mat3 CI(Mat3::Zero());
    CI(0, 0) = A(1, 1) * B(2, 2) - 2 * A(1, 2) * B(1, 2) + A(2, 2) * B(1, 1);
    CI(0, 1) = -A(0, 1) * B(2, 2) + A(0, 2) * B(1, 2) + A(1, 2) * B(0, 2) - A(2, 2) * B(0, 1);
    CI(0, 2) = A(0, 1) * B(1, 2) - A(0, 2) * B(1, 1) - A(1, 1) * B(0, 2) + A(1, 2) * B(0, 1);
    CI(1, 1) = A(0, 0) * B(2, 2) - 2 * A(0, 2) * B(0, 2) + A(2, 2) * B(0, 0);
    CI(1, 2) = -A(0, 0) * B(1, 2) + A(0, 1) * B(0, 2) + A(0, 2) * B(0, 1) - A(1, 2) * B(0, 0);
    CI(2, 2) = A(0, 0) * B(1, 1) - 2 * A(0, 1) * B(0, 1) + A(1, 1) * B(0, 0);

    CI(1, 0) = CI(0, 1);
    CI(2, 0) = CI(0, 2);
    CI(2, 1) = CI(1, 2);
    return CI;
}

Mat4 ModQuadric::probabilisticTriQuadric(const Vec3& p, const Vec3& q, const Vec3& r) const
{
    float sigma = 1;
    if (!area_weighting_)
    {
        sigma = 0.5; // TODO test and find good value
    }
    float sigmaSq = sigma * sigma;

    Mat3 varianceP(Mat3::Zero()), varianceQ(Mat3::Zero()), varianceR(Mat3::Zero());
    varianceP << sigmaSq, 0,       0,
                 0,       sigmaSq, 0,
                 0,       0,       sigmaSq;
    varianceQ << sigmaSq, 0,       0,
                 0,       sigmaSq, 0,
                 0,       0,       sigmaSq;
    varianceR << sigmaSq, 0,       0,
                 0,       sigmaSq, 0,
                 0,       0,       sigmaSq;

    Vec3 pXq = p.cross(q);
    Vec3 qXr = q.cross(r);
    Vec3 rXp = r.cross(p);

    float pqr = (pXq.transpose() * r);

    Vec3 pMq = p-q;
    Vec3 qMr = q-r;
    Vec3 rMp = r-p;

    Vec3 n = pXq + qXr + rXp;
    Mat3 nnT = n * n.transpose();
    Mat3 pMqX = crossProductMatrix(pMq);
    Mat3 qMrX = crossProductMatrix(qMr);
    Mat3 rMpX = crossProductMatrix(rMp);

    Mat3 CIpq = crossInterferenceMatrix(varianceP, varianceQ);
    Mat3 CIqr = crossInterferenceMatrix(varianceQ, varianceR);
    Mat3 CIrp = crossInterferenceMatrix(varianceR, varianceP);

    Mat3 A = nnT
             + pMqX * varianceR * pMqX.transpose()
             + qMrX * varianceP * qMrX.transpose()
             + rMpX * varianceQ * rMpX.transpose()
             + CIpq + CIqr + CIrp;

    Vec3 b = n * pqr
             - pMq.cross(varianceR * pXq)
             - qMr.cross(varianceP * qXr)
             - rMp.cross(varianceQ * rXp)
             + CIpq * r + CIqr * p + CIrp * q;

    float c = pqr * pqr
              + pXq.transpose() * varianceR * pXq
              + qXr.transpose() * varianceP * qXr
              + rXp.transpose() * varianceQ * rXp
              + p.transpose() * CIqr * p
              + q.transpose() * CIrp * q
              + r.transpose() * CIpq * r
              + static_cast<float>((varianceR * CIpq).trace());

    Mat4 quadricM(Mat4::Zero());
    quadricM.block<3,3>(0, 0) = A;
    quadricM.block<3, 1>(0, 3) = -b;
    quadricM.block<1, 3>(3, 0) = -b.transpose();
    quadricM(3, 3) = c;

    if (!area_weighting_)
    {
        quadricM /= n.squaredNorm();
    }

    return quadricM;
}

Mat4 ModQuadric::probabilisticTriQuadric(const OMVec3& ppp, const OMVec3& qqq, const OMVec3& rrr) const
{
    Vec3 pp = Vec3(ppp[0], ppp[1], ppp[2]);
    Vec3 qq = Vec3(qqq[0], qqq[1], qqq[2]);
    Vec3 rr = Vec3(rrr[0], rrr[1], rrr[2]);

    return probabilisticTriQuadric(pp, qq, rr);
}

Mat4 ModQuadric::probabilisticPlaneQuadric(const Vec3& p, const Vec3& q, const Vec3& r) const
{
    float sigma = 0.15;
    float sigmaSq = sigma * sigma;
    if (area_weighting_)
    {
        sigmaSq *= 1e4/((q-p).cross(r - p)).norm(); // TODO test and find good value
    }

    Mat3 varianceN(Mat3::Zero()), variancePt(Mat3::Zero());
    varianceN << 5*sigmaSq/50, 0,       0,
                 0,       5*sigmaSq/50, 0,
                 0,       0,       5*sigmaSq/50;
    variancePt << sigmaSq, 0,       0,
                 0,       sigmaSq, 0,
                 0,       0,       sigmaSq;

    Vec3 pt = (p + q + r) / 3.0f;
    Vec3 n = (q-p).cross(r - p);
    float area = n.norm();
    n /= n.norm();
    Mat3 nnT = n * n.transpose();

    Mat3 A = nnT + varianceN;

    Vec3 b = A * pt;

    float c = static_cast<float>(pt.transpose() * A * pt) + static_cast<float>(n.transpose() * variancePt * n) + static_cast<float>((varianceN * variancePt).trace());

    Mat4 quadricM(Mat4::Zero());
    quadricM.block<3,3>(0, 0) = A;
    quadricM.block<3, 1>(0, 3) = -b;
    quadricM.block<1, 3>(3, 0) = -b.transpose();
    quadricM(3, 3) = c;

    if (area_weighting_)
    {
        quadricM *= area * 0.5;
    }

    return quadricM;
}

Mat4 ModQuadric::probabilisticPlaneQuadric(const OMVec3& ppp, const OMVec3& qqq, const OMVec3& rrr) const
{
    Vec3 pp = Vec3(ppp[0], ppp[1], ppp[2]);
    Vec3 qq = Vec3(qqq[0], qqq[1], qqq[2]);
    Vec3 rr = Vec3(rrr[0], rrr[1], rrr[2]);

    return probabilisticPlaneQuadric(pp, qq, rr);
}
#endif // C2M_PROB_QUADRICS

ModQuadric::ModQuadric(CMesh& _mesh) : ModBase(_mesh, false), max_err_(FLT_MAX), area_weighting_(false)
{
}

/// Destructor
ModQuadric::~ModQuadric()
{
}

void ModQuadric::initialize(void)
{
    // alloc quadrics if not already there
    bool preDefined = false;
    bool preEmpty = false;
    for (VHandle v : mesh_.vertices())
    {
        if (mesh_.data(v).quadrics.empty())
        {
            preEmpty = true;
            if (preDefined)
            {
                throw std::logic_error("Some quadrics allocated, others aren't, can't handle this.");
            }
#ifdef C2M_PROB_QUADRICS
            mesh_.data(v).quadrics = vector<Quadric>((optimize_position_ ? num_frames() : frame_seq().size()), Mat4::Zero());
#else
            mesh_.data(v).quadrics = vector<Quadric>(optimize_position_ ? num_frames() : frame_seq().size(), Quadric());
#endif
        }
        else
        {
            if (mesh_.data(v).quadrics.size() != (optimize_position_ ? num_frames() : frame_seq().size()))
            {
                throw std::logic_error("Unexpected number of quadrices per vertex (!= number of frames).");
            }
            preDefined = true;
            if (preEmpty)
            {
                throw std::logic_error("Some quadrics allocated, others aren't, can't handle this.");
            }
        }
    }

    // If quadrics are supplied (from preprocessing or previous decimation -> keep them)
    if (preDefined)
        return;

    // calc quadric
    Mesh::FaceIter f_it = mesh_.faces_begin(), f_end = mesh_.faces_end();

    Mesh::FaceHalfedgeCCWIter fh_it;
    Mesh::HalfedgeHandle heh[3];
    Mesh::VertexHandle vh[3];
    Vec3 points[3];

    for (; f_it != f_end; ++f_it)
    {
        fh_it = mesh_.fh_ccwiter(*f_it);
        heh[0] = *fh_it;
        vh[0] = mesh_.from_vertex_handle(*fh_it);
        ++fh_it;
        heh[1] = *fh_it;
        vh[1] = mesh_.from_vertex_handle(*fh_it);
        ++fh_it;
        heh[2] = *fh_it;
        vh[2] = mesh_.from_vertex_handle(*fh_it);

        vector<uint> frames(frame_seq());

        // DO NOT INCLUDE FAILED FACES
        bool skip = false;
        for (size_t frame = 0; frame < mesh_.data(*f_it).element->active.size(); frame++)
        {
            // TODO do this properly
            if (!mesh_.data(*f_it).element->active[frame])
            {
                skip = true;
                break;
            }
        }
        if (skip)
            continue;

        for (size_t i = 0; i < (optimize_position_ ? num_frames() : frames.size()); i++)
        {
            uint frame = (optimize_position_ ? i : frames[i]);
            for (uint j = 0; j < 3; j++)
            {
                points[j] = mesh_.data(vh[j]).node->positions.row(frame).transpose();
            }

            // Sum face quadrices up at each corner vertex
            for (uint j = 0; j < 3; j++)
            {                
                mesh_.data(vh[j]).quadrics[i] += calc_face_quadric(frame, points[0], points[1], points[2]);
            }

            // Sum edge quadrices up at both edge vertices
            for (uint j = 0; j < 3; j++)
            {
                Quadric qEdge = calc_edge_quadric(heh[j], frame, points[j], points[(j + 1) % 3], points[(j + 2) % 3]);
                mesh_.data(vh[j]).quadrics[i] += qEdge;
                mesh_.data(vh[(j + 1) % 3]).quadrics[i] += qEdge;
            }
        }
    }
}

float ModQuadric::collapse_priority(const CollapseInfo& _ci)
{
    const vector<Quadric>& quadricsRemoved = mesh_.data(_ci.v0).quadrics;
    const vector<Quadric>& quadricsRemaining = mesh_.data(_ci.v1).quadrics;

    float error = 0;
    const MatX3& positions = mesh_.data(_ci.v1).node->positions;
    vector<uint> frames(frame_seq());

    for (size_t i = 0; i < frames.size(); i++)
    {
        uint frame = frames[i];
        Quadric q = quadricsRemaining[(optimize_position_ ? frame : i)] + quadricsRemoved[(optimize_position_ ? frame : i)];
        Vec3 optPos = positions.row(frame).transpose();
        if (optimize_position_ && !mesh_.status(_ci.v1).locked() && !mesh_.data(_ci.v0).duplicate.is_valid()
            && !mesh_.data(_ci.v1).duplicate.is_valid())
        {
            optimal_position(q, optPos);
        }
#ifdef C2M_PROB_QUADRICS
        Vec4 pointRemaining(optPos[0], optPos[1], optPos[2], 1);
        float err = pointRemaining.transpose() * (q * pointRemaining);
        error = std::max(error, err);
#else
        OMVec3 pointRemaining = OMVec3(optPos[0], optPos[1], optPos[2]);
        error = std::max(error, q(pointRemaining));
#endif
        if (error > max_err_)
            return Base::ILLEGAL_COLLAPSE;
    }

    return error;
}

void ModQuadric::postprocess_collapse(const CollapseInfo& _ci)
{
    const vector<Quadric>& quadricsRemoved = mesh_.data(_ci.v0).quadrics;
    vector<Quadric>& quadricsRemaining = mesh_.data(_ci.v1).quadrics;
    vector<uint> frames(frame_seq());
    for (size_t frame = 0; frame < quadricsRemoved.size(); frame++)
    {
        quadricsRemaining[frame] += quadricsRemoved[frame];
        // TODO check usefulness
        // if (!area_weighting_)
        // {
        //     quadricsRemaining[frame] *= 0.5f;
        // }
    }

    if (optimize_position_ && !mesh_.status(_ci.v1).locked() && !mesh_.data(_ci.v0).duplicate.is_valid()
        && !mesh_.data(_ci.v1).duplicate.is_valid())
    {
        for (size_t frame = 0; frame < num_frames(); frame++)
        {
            Vec3 optPos = mesh_.data(_ci.v1).node->positions.row(frame).transpose();
            optimal_position(quadricsRemaining[frame], optPos);
            mesh_.data(_ci.v1).node->positions.row(frame) = optPos.transpose();
        }
    }
}

void ModQuadric::set_error_tolerance_factor(double _factor)
{
    if (this->is_binary())
    {
        if (_factor >= 0.0 && _factor <= 1.0)
        {
            // the smaller the factor, the smaller max_err_ gets
            // thus creating a stricter constraint
            // division by error_tolerance_factor_ is for normalization
            double max_err = max_err_ * _factor / this->error_tolerance_factor_;
            set_max_err(max_err);
            this->error_tolerance_factor_ = _factor;

            initialize();
        }
    }
}

bool ModQuadric::optimal_position(Quadric& q, Vec3& optimalPos) const
{
#ifdef C2M_PROB_QUADRICS
    Mat3 A = q.block<3, 3>(0, 0);
    Vec3 b = -q.block<3, 1>(0, 3);

    optimalPos = A.llt().solve(b);
    return true;
#else
    Mat3 A(Mat3::Zero());
    A << q.a(), q.b(), q.c(),
         q.b(), q.e(), q.f(),
         q.c(), q.f(), q.h();

    Vec3 reference = optimalPos;

    // float tolerance = 0.15;
    // if (area_weighting_)
    // {
    //     tolerance = 0.15 * A.trace() / 20;
    // }
    auto svd = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    float tolerance = 1e-3;

    const auto &singularValues = svd.singularValues();
    Mat3 singularValuesInv(Mat3::Zero());
    float maxSV = singularValues.maxCoeff();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) / maxSV > tolerance)
        {
            singularValuesInv(i, i) = 1.0f / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = 0.0f;
        }
    }
    Mat3 Ainv = svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();

    optimalPos = reference - Ainv * (A * reference + Vec3(q.d(), q.g(), q.i()));
    return true;
#endif
}

float ModQuadric::factor_dist_to_epicenter(Vec3 pt, Vec3 epicenter, float mean_dist) const
{
    // TODO implement a proper function here
    float factor = 0.5f + 0.5f * ((pt - epicenter).squaredNorm() / (mean_dist * mean_dist));
    return factor;
}

Quadric ModQuadric::calc_face_quadric(uint frame, const Vec3& p, const Vec3& q, const Vec3& r) const
{
#ifdef C2M_PROB_QUADRICS
    Quadric qFace;
    if (area_weighting_)
    {
        qFace = probabilisticTriQuadric(p, q, r);
    }
    else
    {
        qFace = probabilisticPlaneQuadric(p, q, r);
    }
#else
    Vec3 faceNormal = (q - p).cross(r - p);
    double area = faceNormal.norm();
    if (area > FLT_MIN)
    {
        faceNormal /= area;
        area *= 0.5;
    }

    float a = faceNormal[0];
    float b = faceNormal[1];
    float c = faceNormal[2];
    float d = -p.transpose() * faceNormal;

    Quadric qFace(a, b, c, d);
    if (area_weighting_)
    {
        qFace *= area;
    }
#endif
    qFace *= 1.0 / dist2epicenter_f((p + q + r) / 3.0, frame);

    return qFace;
}

Quadric ModQuadric::calc_edge_quadric(HEHandle he, uint frame, const Vec3& p, const Vec3& q, const Vec3& r) const
{
#ifdef C2M_PROB_QUADRICS
    Quadric qEdge = Mat4::Zero();
#else
    Quadric qEdge;
#endif
    float weightFactor = 0;
    if (mesh_.is_boundary(mesh_.opposite_halfedge_handle(he)))
    {
        if (MeshAnalyzer::dupes(mesh_, he).size() != 1)
            return qEdge;
        // preserve boundary contours
        weightFactor = 2.0;
    }
    else
    {
        // preserve feature edges, across which plastic strains differ
        float strain = mesh_.data(mesh_.face_handle(he)).element->plasticStrains(frame);
        float neighbor_strain = mesh_.data(mesh_.face_handle(mesh_.opposite_halfedge_handle(he))).element->plasticStrains(frame);
        if (abs(neighbor_strain - strain) < 0.005)
            return qEdge;
        weightFactor = abs(neighbor_strain - strain) * 50;
    }

    // Plane quadric for plane passing through edge, perpendicular to triangle
    Vec3 faceNormal = (q - p).cross(r - p);
    faceNormal /= faceNormal.norm();
    Vec3 edgeNormal = (q - p).cross(faceNormal);
    double edgePlaneArea = edgeNormal.squaredNorm(); // n is length 1 so square the influence of boundary
#ifdef C2M_PROB_QUADRICS
    if (area_weighting_)
    {
        qEdge = probabilisticTriQuadric(p, q, p + faceNormal * sqrt(edgePlaneArea));
    }
    else
    {
        qEdge = probabilisticPlaneQuadric(p, q, p + faceNormal * sqrt(edgePlaneArea));
    }
#else
    if (edgePlaneArea > FLT_MIN)
    {
        edgeNormal /= sqrt(edgePlaneArea);
        edgePlaneArea *= 0.5;
    }

    float a = edgeNormal[0];
    float b = edgeNormal[1];
    float c = edgeNormal[2];
    float d = -p.transpose() * edgeNormal;

    qEdge = Quadric(a, b, c, d);
    if (area_weighting_)
    {
        qEdge *= edgePlaneArea;
    }
#endif
    qEdge *= weightFactor / dist2epicenter_f((p + q) / 2.0, frame);

    return qEdge;
}

} // namespace c2m
