
// KdTree custom ----------------------------------------------------------------

template<typename Traits>
template<typename PointUserContainer, typename Converter>
inline void KdTreeCustom<Traits>::build(PointUserContainer&& points, Converter c)
{
    IndexContainer ids(points.size());
    std::iota(ids.begin(), ids.end(), 0);
    this->buildWithSampling(std::forward<PointUserContainer>(points), std::move(ids), std::move(c));
}

template<typename Traits>
template<typename PointUserContainer, typename IndexUserContainer, typename Converter>
inline void KdTreeCustom<Traits>::buildWithSampling(PointUserContainer&& points,
                                                  IndexUserContainer sampling,
                                                  Converter c)
{
    PONCA_DEBUG_ASSERT(points.size() <= MAX_POINT_COUNT);
    this->clear();

    // Move, copy or convert input samples
    c(std::forward<PointUserContainer>(points), Base::m_points);

    Base::m_nodes = NodeContainer();
    Base::m_nodes.reserve(4 * Base::point_count() / Base::m_min_cell_size);
    Base::m_nodes.emplace_back();

    Base::m_indices = std::move(sampling);

    this->build_rec(0, 0, Base::index_count(), 1);
    // this->build_fit_rec(0); // Only used if the aggregation of the children for a node is a working method.

    PONCA_DEBUG_ASSERT(this->valid());
}

template<typename Traits>
void KdTreeCustom<Traits>::build_rec(NodeIndexType node_id, IndexType start, IndexType end, int level)
{
    NodeType& node = Base::m_nodes[node_id];
    AabbType aabb;
    for(IndexType i=start; i<end; ++i)
        aabb.extend(Base::m_points[Base::m_indices[i]].pos());

    node.set_is_leaf(
        end-start <= Base::m_min_cell_size ||
        level >= Traits::MAX_DEPTH ||
        // Since we add 2 nodes per inner node we need to stop if we can't add
        // them both
        (NodeIndexType)Base::m_nodes.size() > Base::MAX_NODE_COUNT - 2);

    // node.configure_range(this, start, end-start, aabb);
    node.configure_range(start, end-start, aabb); // start, size, aabb
    node.configure_fitting(*this, start, end-start);

    if (node.is_leaf())
    {
        ++Base::m_leaf_count;
    }
    else
    {
        int split_dim = 0;
        (Scalar(0.5) * aabb.diagonal()).maxCoeff(&split_dim);
        node.configure_inner(aabb.center()[split_dim], Base::m_nodes.size(), split_dim);
        Base::m_nodes.emplace_back();
        Base::m_nodes.emplace_back();

        IndexType mid_id = this->partition(start, end, split_dim, node.inner_split_value());
        build_rec(node.inner_first_child_id(), start, mid_id, level+1);
        build_rec(node.inner_first_child_id()+1, mid_id, end, level+1);
    }
}

// Seems to be useless, as we don't need to compute the fit in that way, but for each node/leaf.
template<typename Traits>
void KdTreeCustom<Traits>::build_fit_rec(NodeIndexType node_id)
{
    const NodeType& node = Base::m_nodes[node_id];
    if ( ! node.is_leaf() )
    {
        // check if children are fitted before fitting the parent
        if ( ! Base::m_nodes[node.inner_first_child_id()].is_fitted() )
            build_fit_rec(node.inner_first_child_id());
        if ( ! Base::m_nodes[node.inner_first_child_id()+1].is_fitted() )
            build_fit_rec(node.inner_first_child_id()+1);

        // Fit the parent
        Base::m_nodes[node_id].compute_inner_fit(*this);
    }
}

// template<typename Traits>
// bool KdTreeCustom<Traits>::valid() const
// {
//     if (m_points.empty())
//         return m_nodes.empty() && m_indices.empty();

//     if(m_nodes.empty() || m_indices.empty())
//     {
//         return false;
//     }

//     std::vector<bool> b(point_count(), false);
//     for(IndexType idx : m_indices)
//     {
//         if(idx < 0 || point_count() <= idx || b[idx])
//         {
//             return false;
//         }
//         b[idx] = true;
//     }

//     for(NodeIndexType n=0;n<node_count();++n)
//     {
//         const NodeType& node = m_nodes[n];
//         if(node.is_leaf())
//         {
//             if(index_count() <= node.leaf_start() || node.leaf_start()+node.leaf_size() > index_count())
//             {
//                 return false;
//             }
//         }
//         else
//         {
//             if(node.inner_split_dim() < 0 || DataPoint::Dim-1 < node.inner_split_dim())
//             {
//                 return false;
//             }
//             if(node_count() <= node.inner_first_child_id() || node_count() <= node.inner_first_child_id()+1)
//             {
//                 return false;
//             }
//         }
//     }

//     return true;
// }

template<typename Traits>
std::string KdTreeCustom<Traits>::to_string(bool verbose) const
{
    std::stringstream str;

    str << "KdTree:";
    str << "\n  MaxNodes: " << Base::MAX_NODE_COUNT;
    str << "\n  MaxPoints: " << Base::MAX_POINT_COUNT;
    str << "\n  MaxDepth: " << Traits::MAX_DEPTH;
    str << "\n  PointCount: " << Base::point_count();
    str << "\n  SampleCount: " << Base::index_count();
    str << "\n  NodeCount: " << Base::node_count();

    if (!verbose)
    {
        return str.str();
    }

    str << "\n  Samples: [";
    static constexpr IndexType SAMPLES_PER_LINE = 10;
    for (IndexType i = 0; i < Base::index_count(); ++i)
    {
        str << (i == 0 ? "" : ",");
        str << (i % SAMPLES_PER_LINE == 0 ? "\n    " : " ");
        str << Base::m_indices[i];
    }

    str << "]\n  Nodes:";
    for (NodeIndexType n = 0; n < Base::node_count(); ++n)
    {
        const NodeType& node = Base::m_nodes[n];
        if (node.is_leaf())
        {
            str << "\n    - Type: Leaf";
            str << "\n      Start: " << node.leaf_start();
            str << "\n      Size: "  << node.leaf_size();
            str << "\n      Fit: "   << node.leaf_is_fitted();
        }
        else
        {
            str << "\n    - Type: Inner";
            str << "\n      SplitDim: "   << node.inner_split_dim();
            str << "\n      SplitValue: " << node.inner_split_value();
            str << "\n      FirstChild: " << node.inner_first_child_id();
            str << "\n      Fit: "        << node.inner_is_fitted();
        }
    }

    return str.str();
}

