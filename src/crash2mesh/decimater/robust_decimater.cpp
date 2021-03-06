#include <crash2mesh/decimater/robust_decimater.hpp>

#include <crash2mesh/util/logger.hpp>

#include <algorithm>

namespace c2m
{
using std::map;
using std::set;
using std::vector;

RobustDecimater::RobustDecimater(CMesh& _mesh, bool _optimize_position)
    : Base(_mesh), mesh_(_mesh), optimize_position_(_optimize_position),
#if (defined(_MSC_VER) && (_MSC_VER >= 1800)) || __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
      heap_(nullptr)
#else
      heap_(nullptr)
#endif

{

    // private vertex properties
    mesh_.add_property(collapse_target_);
    mesh_.add_property(priority_);
    mesh_.add_property(heap_position_);
    if (optimize_position_)
        mesh_.add_property(optimalCollapseTargets_, "collapseTargets");
}

//-----------------------------------------------------------------------------

RobustDecimater::~RobustDecimater()
{

    // private vertex properties
    mesh_.remove_property(collapse_target_);
    mesh_.remove_property(priority_);
    mesh_.remove_property(heap_position_);
    if (optimize_position_)
        mesh_.remove_property(optimalCollapseTargets_);
}

//-----------------------------------------------------------------------------

bool RobustDecimater::is_multi_collapse_legal(const std::vector<VHandle>& v0Dupes,
                                              const std::vector<VHandle>& v1Dupes,
                                              const std::vector<CollapseInfo>& ciDupes,
                                              bool separated)
{
    if (v0Dupes.size() > 1 && v1Dupes.size() == 1 || v0Dupes.size() > 1 && ciDupes.size() == 1)
        return false;

    // Check collapses separately
    for (const CollapseInfo& ciDupe : ciDupes)
    {
        if (mesh_.is_boundary(ciDupe.v0v1) || mesh_.is_boundary(ciDupe.v1v0))
        {
            HEHandle boundary = mesh_.is_boundary(ciDupe.v0v1) ? ciDupe.v0v1 : ciDupe.v1v0;
            HEHandle next = boundary;
            for (uint i = 0; i < 3; i++)
                next = mesh_.next_halfedge_handle(next);

            // Should isolated triangles be allowed for decimation?
            // if (next == boundary)
            //     continue;

            // Quads are preserved!
            if (mesh_.next_halfedge_handle(next) == boundary)
                return false;
        }

        if (!this->is_collapse_legal(ciDupe))
            return false;
    }

    // Check collapses as a sequence by testing them on a duplicated test submesh
    if (!separated && ciDupes.size() > 1)
    {
        CMesh testMesh;
        map<VHandle, VHandle> main2test;
        set<FHandle> neighborhood;
        for (const CollapseInfo& ciDupe : ciDupes)
        {
            if (main2test.find(ciDupe.v0) == main2test.end())
                main2test[ciDupe.v0] = testMesh.add_vertex(mesh_.point(ciDupe.v0));
            for (const VHandle& v0RingV : mesh_.vv_range(ciDupe.v0))
            {
                if (main2test.find(v0RingV) == main2test.end())
                    main2test[v0RingV] = testMesh.add_vertex(mesh_.point(v0RingV));
            }
            if (main2test.find(ciDupe.v1) == main2test.end())
                main2test[ciDupe.v1] = testMesh.add_vertex(mesh_.point(ciDupe.v1));
            for (const VHandle& v1RingV : mesh_.vv_range(ciDupe.v1))
            {
                if (main2test.find(v1RingV) == main2test.end())
                    main2test[v1RingV] = testMesh.add_vertex(mesh_.point(v1RingV));
            }
        }
        for (const CollapseInfo& ciDupe : ciDupes)
        {
            for (const FHandle& v0RingF : mesh_.vf_range(ciDupe.v0))
            {
                if (neighborhood.find(v0RingF) == neighborhood.end())
                {
                    neighborhood.insert(v0RingF);
                    std::vector<VHandle> vs;
                    for (const VHandle& v0RingFV : mesh_.fv_range(v0RingF))
                    {
                        assert(main2test.find(v0RingFV) != main2test.end());
                        vs.emplace_back(main2test[v0RingFV]);
                    }
                    testMesh.add_face(vs);
                }
            }
            for (const FHandle& v1RingF : mesh_.vf_range(ciDupe.v1))
            {
                if (neighborhood.find(v1RingF) == neighborhood.end())
                {
                    neighborhood.insert(v1RingF);
                    std::vector<VHandle> vs;
                    for (const VHandle& v1RingFV : mesh_.fv_range(v1RingF))
                    {
                        assert(main2test.find(v1RingFV) != main2test.end());
                        vs.emplace_back(main2test[v1RingFV]);
                    }
                    testMesh.add_face(vs);
                }
            }
        }
        RobustDecimater testDecimater(testMesh, false);
        for (const CollapseInfo ciDupe : ciDupes)
        {
            assert(ciDupe.v0v1.is_valid());
            HEHandle testv0v1 = testMesh.find_halfedge(main2test[ciDupe.v0], main2test[ciDupe.v1]);
            if (!testv0v1.is_valid())
            {
                // TODO find out why this happens and properly fix it
                // NO IDEA WHY THIS CAN HAPPEN BUT IT DOES
                // Logger::lout(Logger::DEBUG) << "Some error in duplicating submesh must have occurred" << std::endl;
                return false;
            }
            CollapseInfo ciTest(testMesh, testv0v1);

            if (testDecimater.is_collapse_legal(ciTest))
                testMesh.collapse(testv0v1);
            else
                return false;
        }
    }

    return true;
}

void RobustDecimater::get_dupes(const CollapseInfo& _ci,
                               std::vector<VHandle>& v0Dupes,
                               std::vector<VHandle>& v1Dupes,
                               std::vector<CollapseInfo>& ciDupes) const
{
    v0Dupes.clear();
    v1Dupes.clear();
    ciDupes.clear();

    // std::cout << "PREV0 " << _ci.v0.idx() << std::endl;
    VHandle vDupe = _ci.v0;
    do
    {
        assert(v0Dupes.size() < 20);
        v0Dupes.emplace_back(vDupe);
        vDupe = mesh_.data(vDupe).duplicate;
    } while (vDupe.is_valid() && vDupe != _ci.v0);

    vDupe = _ci.v1;
    do
    {
        assert(v1Dupes.size() < 20);
        v1Dupes.emplace_back(vDupe);
        vDupe = mesh_.data(vDupe).duplicate;
    } while (vDupe.is_valid() && vDupe != _ci.v1);

    // std::cout << "PRECI" << std::endl;
    ciDupes.emplace_back(_ci);
    HEHandle heDupe;
    for (const VHandle& v0Dupe : v0Dupes)
    {
        assert(ciDupes.size() < 20);
        bool skip = false;
        for (const CollapseInfo& ci : ciDupes)
            if (ci.v0 == v0Dupe || ci.v1 == v0Dupe)
            {
                skip = true;
                break;
            }
        if (skip)
            continue;
        for (const VHandle& v1Dupe : v1Dupes)
        {
            heDupe = mesh_.find_halfedge(v0Dupe, v1Dupe);
            if (heDupe.is_valid())
                ciDupes.emplace_back(CollapseInfo(mesh_, heDupe));
        }
    }
    // std::cout << "POSTCI" << std::endl;
}
float RobustDecimater::collapse_priority(const CollapseInfo& _ci)
{
    if (optimize_position_)
    {
        const vector<Quadric>& quadricsRemoved = mesh_.data(_ci.v0).quadrics;
        const vector<Quadric>& quadricsRemaining = mesh_.data(_ci.v1).quadrics;

        MatX3& collapseTargets = mesh_.property(optimalCollapseTargets_, _ci.v0v1);
        collapseTargets = mesh_.data(_ci.v1).node->positions;
        if (!mesh_.status(_ci.v1).locked() && !mesh_.data(_ci.v0).duplicate.is_valid()
            && !mesh_.data(_ci.v1).duplicate.is_valid())
        {
            for (size_t i = 0; i < quadricsRemoved.size(); i++)
            {
                Quadric q = quadricsRemaining[i] + quadricsRemoved[i];
                Vec3 optPos = collapseTargets.row(i).transpose();
                    ModQuadric::optimal_position(q, optPos);
                    collapseTargets.row(i) = optPos.transpose();
            }
        }
    }
    return Base::collapse_priority(_ci);
}

/// Post-process a collapse
void RobustDecimater::preprocess_collapse(CollapseInfo& _ci)
{
    if (optimize_position_)
    {
        if (!optimalCollapseTargets_.is_valid())
            throw std::logic_error("Impossible, there must be a programming error here");
        MatX3& collapseTargets = mesh_.property(optimalCollapseTargets_, _ci.v0v1);
        if (collapseTargets.size() == 0)
        {
            throw std::logic_error("Impossible, there must be a programming error here");
        }
        mesh_.data(_ci.v1).node->positions = collapseTargets;
    }
    Base::preprocess_collapse(_ci);
}

void RobustDecimater::heap_vertex(VHandle _vh)
{
    //   std::clog << "heap_vertex: " << _vh << std::endl;

    float prio(0.0f), sumPrio(0.0f), best_prio(FLT_MAX);
    HEHandle heh, collapse_target;

    std::vector<VHandle> v0Dupes;
    std::vector<VHandle> v1Dupes;
    std::vector<CollapseInfo> ciDupes;

    // find best target in one ring
    CMesh::VertexOHalfedgeIter voh_it(mesh_, _vh);
    for (; voh_it.is_valid() && !mesh_.data(_vh).fixed; ++voh_it)
    {
        heh = *voh_it;
        CollapseInfo _ci(mesh_, heh);
        get_dupes(_ci, v0Dupes, v1Dupes, ciDupes);

        if (!this->is_multi_collapse_legal(v0Dupes, v1Dupes, ciDupes, true))
            continue;

        bool skip = false;
        for (VHandle v0Dupe: v0Dupes)
        {
            if (mesh_.data(v0Dupe).mustRemove)
            {
                best_prio = 0.0f;
                collapse_target = heh;
                skip = true;
                break;
            }
        }
        if (skip)
            break;

        sumPrio = 0.0;
        for (const CollapseInfo& ciDupe : ciDupes)
        {
            prio = this->collapse_priority(ciDupe);
            if (prio < 0.0)
            {
                sumPrio = Module::ILLEGAL_COLLAPSE;
            }
            if (sumPrio != Module::ILLEGAL_COLLAPSE)
                sumPrio += prio;
        }

        if (sumPrio >= 0.0 && sumPrio < best_prio)
        {
            best_prio = sumPrio;
            collapse_target = heh;
        }
    }

    // target found -> put vertex on heap
    if (collapse_target.is_valid())
    {
        //     std::clog << "  added|updated" << std::endl;
        mesh_.property(collapse_target_, _vh) = collapse_target;
        mesh_.property(priority_, _vh) = best_prio;

        if (heap_->is_stored(_vh))
            heap_->update(_vh);
        else
            heap_->insert(_vh);
    }
    // not valid -> remove from heap
    else
    {
        //     std::clog << "  n/a|removed" << std::endl;
        if (heap_->is_stored(_vh))
            heap_->remove(_vh);

        mesh_.property(collapse_target_, _vh) = collapse_target;
        mesh_.property(priority_, _vh) = -1;
    }
}

//-----------------------------------------------------------------------------
size_t RobustDecimater::decimate_to_faces(size_t _nv, size_t _nf)
{

    if (!this->is_initialized())
        return 0;

    if (_nv >= mesh_.n_vertices() || _nf >= mesh_.n_faces())
        return 0;

    CMesh::VertexIter v_it, v_end(mesh_.vertices_end());
    CMesh::VHandle vp;
    CMesh::HalfedgeHandle v0v1;
    CMesh::VertexVertexIter vv_it;
    CMesh::VertexFaceIter vf_it;
    unsigned int n_collapses(0);
    size_t nv = mesh_.n_vertices();
    size_t nf = mesh_.n_faces();

    using Support = set<CMesh::VHandle>;
    using SupportIterator = Support::iterator;

    Support support;
    SupportIterator s_it, s_end;

    std::vector<VHandle> v0Dupes;
    std::vector<VHandle> v1Dupes;
    std::vector<CollapseInfo> ciDupes;
    std::vector<VHandle> coveredV0Dupes;

    // initialize heap
    HeapInterface HI(mesh_, priority_, heap_position_);
#if (defined(_MSC_VER) && (_MSC_VER >= 1800)) || __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
    heap_ = std::unique_ptr<DeciHeap>(new DeciHeap(HI));
#else
    heap_ = std::auto_ptr<DeciHeap>(new DeciHeap(HI));
#endif
    heap_->reserve(mesh_.n_vertices());

    for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it)
    {
        heap_->reset_heap_position(*v_it);
        if (!mesh_.status(*v_it).deleted())
            heap_vertex(*v_it);
    }

    const bool update_normals = mesh_.has_face_normals();

    // process heap
    while ((!heap_->empty()) && (_nv < nv) && (_nf < nf))
    {
        // get 1st heap entry
        vp = heap_->front();
        v0v1 = mesh_.property(collapse_target_, vp);
        heap_->pop_front();

        assert(!mesh_.status(vp).deleted());
        assert(!mesh_.status(v0v1).deleted());
        assert(!mesh_.status(mesh_.to_vertex_handle(v0v1)).deleted());

        // setup collapse info
        CollapseInfo _ci(mesh_, v0v1);
        get_dupes(_ci, v0Dupes, v1Dupes, ciDupes);

        if (!is_multi_collapse_legal(v0Dupes, v1Dupes, ciDupes, false))
            continue;

        // store support (= combined one ring of all v0Dupes)
        support.clear();
        
        for (const VHandle& v0Dupe : v0Dupes)
        {
            assert(!mesh_.status(v0Dupe).deleted());
            vv_it = mesh_.vv_iter(v0Dupe);
            for (; vv_it.is_valid(); ++vv_it)
            {
                bool insert = true;
                for (const VHandle& notIncluded : v0Dupes)
                    if (*vv_it == notIncluded)
                        insert = false;
                if (insert)
                    support.insert(*vv_it);
            }
        }

        if (optimize_position_)
        {
            for (const VHandle& v1Dupe : v1Dupes)
            {
                assert(!mesh_.status(v1Dupe).deleted());
                vv_it = mesh_.vv_iter(v1Dupe);
                for (; vv_it.is_valid(); ++vv_it)
                {
                    bool insert = true;
                    for (const VHandle& notIncluded : v1Dupes)
                        if (*vv_it == notIncluded)
                            insert = false;
                    if (insert)
                        support.insert(*vv_it);
                }
            }
        }


        for (CollapseInfo& ciDupe : ciDupes)
        {
            // adjust complexity in advance (need boundary status)
            --nv;
            if (mesh_.is_boundary(ciDupe.v0v1) || mesh_.is_boundary(ciDupe.v1v0))
                --nf;
            else
                nf -= 2;

            // pre-processing
            this->preprocess_collapse(ciDupe);
        }

        // Check which duplicates of v0 are not part of a dupe edge of v0v1
        // and thus need to be made dupes of v1 after all v0v1 dupes are collapsed into v1
        if (v0Dupes.size() > 1)
        {
            for (const VHandle& v0Dupe : v0Dupes)
            {
                bool willGetDeleted = false;
                for (const CollapseInfo& ci : ciDupes)
                {
                    if (ci.v0 == v0Dupe)
                    {
                        willGetDeleted = true;
                        break;
                    }
                }
                // v0 is collapsed into v1, so all duplicates of v0 not collapsed via a v0v1 dupe edge
                // need to be manually made dupes of v1.
                // Any dupes that are covered by a dupe edge of v0v1 will be collapsed/deleted anyways,
                // so they dont need to be explicitly taken care of.
                if (!willGetDeleted)
                {
                    assert(std::find(v1Dupes.begin(), v1Dupes.end(), v0Dupe) == v1Dupes.end());
                    mesh_.data(v0Dupe).node = mesh_.data(v1Dupes.front()).node;
                    mesh_.data(v0Dupe).fixed = true;
                    mesh_.point(v0Dupe) = mesh_.point(v1Dupes.front());
                    mesh_.data(v0Dupe).duplicate = v1Dupes.front();
                    mesh_.data(v1Dupes.back()).duplicate = v0Dupe;
                    v1Dupes.emplace_back(v0Dupe);
                }
            }
        }

        // perform collapse
        for (const CollapseInfo& ciDupe : ciDupes)
        {
            assert(this->is_collapse_legal(ciDupe));
            mesh_.collapse(ciDupe.v0v1);
            ++n_collapses;
        }

        for (const VHandle& v1Dupe : v1Dupes)
        {
            if (update_normals)
            {
                // update triangle normals
                vf_it = mesh_.vf_iter(v1Dupe);
                for (; vf_it.is_valid(); ++vf_it)
                    if (!mesh_.status(*vf_it).deleted())
                        mesh_.set_normal(*vf_it, mesh_.calc_face_normal(*vf_it));
            }
        }

        for (CollapseInfo& ciDupe : ciDupes)
        {
            // post-process collapse
            this->postprocess_collapse(ciDupe);
        }

        // update heap (former one ring of all duped decimated vertices)
        for (s_it = support.begin(), s_end = support.end(); s_it != s_end; ++s_it)
        {
            assert(!mesh_.status(*s_it).deleted());
            heap_vertex(*s_it);
        }

        // notify observer and stop if the observer requests it
        if (!this->notify_observer(n_collapses))
            return n_collapses;
    }

    // delete heap
    heap_.reset();

    // DON'T do garbage collection here! It's up to the application.
    return n_collapses;
}

} // namespace c2m
