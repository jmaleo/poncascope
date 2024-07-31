// NDTree ----------------------------------------------------------------------

template<typename Traits>
NDTreeBase<Traits>::NDTreeBase()
    : m_min_cell_size(64), m_leaf_count(0)
{
}

template<typename Traits>
template<typename PointUserContainer>
NDTreeBase<Traits>::NDTreeBase(PointUserContainer&& points)
    : m_min_cell_size(64), m_leaf_count(0)
{
    this->build(std::forward<PointUserContainer>(points));
}

template<typename Traits>
void NDTreeBase<Traits>::clear()
{
    m_points.clear();
    m_nodes.clear();
    m_indices.clear();
    m_leaf_count = 0;
}

template<typename Traits>
template <typename Input>
inline void NDTreeBase<Traits>::DefaultConverter::operator()(Input&& i, PointContainer& o)
{
    using InputContainer = typename std::remove_reference<Input>::type;
    if constexpr (std::is_same<InputContainer, PointContainer>::value)
        o = std::forward<Input>(i);
    else
        std::transform(i.cbegin(), i.cend(), std::back_inserter(o),
                       [](const typename InputContainer::value_type &p) -> DataPoint { return DataPoint(p); });
}

template<typename Traits>
template<typename PointUserContainer>
inline void NDTreeBase<Traits>::build(PointUserContainer&& points)
{
    build(std::forward<PointUserContainer>(points), DefaultConverter());
}

template<typename Traits>
template<typename PointUserContainer, typename Converter>
inline void NDTreeBase<Traits>::build(PointUserContainer&& points, Converter c)
{
    clear();
    c(std::forward<PointUserContainer>(points), m_points);
    
    m_nodes.reserve(4 * point_count() / m_min_cell_size);
    m_nodes.emplace_back();

    m_indices.resize(point_count());
    std::iota(m_indices.begin(), m_indices.end(), 0);

    // Calculer la AABB initiale
    AabbType initial_aabb;
    for (const auto& point : m_points)
    {
        initial_aabb.extend(point.pos());
    }

    // Trouver la plus grande dimension
    VectorType extents = initial_aabb.max() - initial_aabb.min();
    Scalar max_extent = extents.maxCoeff();

    // Créer une AABB cubique
    VectorType center = initial_aabb.center();
    AabbType root_aabb;
    for (int i = 0; i < DIM; ++i)
    {
        root_aabb.min()[i] = center[i] - max_extent / 2;
        root_aabb.max()[i] = center[i] + max_extent / 2;
    }

    build_rec(0, 0, index_count(), root_aabb, 1);
}

template<typename Traits>
template<typename PointUserContainer, typename IndexUserContainer>
inline void NDTreeBase<Traits>::buildWithSampling(PointUserContainer&& points, IndexUserContainer sampling)
{
    buildWithSampling(std::forward<PointUserContainer>(points), std::move(sampling), DefaultConverter());
}

template<typename Traits>
template<typename PointUserContainer, typename IndexUserContainer, typename Converter>
inline void NDTreeBase<Traits>::buildWithSampling(PointUserContainer&& points, IndexUserContainer sampling, Converter c)
{
    clear();
    c(std::forward<PointUserContainer>(points), m_points);
    
    m_nodes.reserve(4 * sampling.size() / m_min_cell_size);
    m_nodes.emplace_back();

    m_indices = std::move(sampling);

    AabbType initial_aabb;
    for (const auto& idx : m_indices)
    {
        initial_aabb.extend(m_points[idx].pos());
    }
    // Trouver la plus grande dimension
    VectorType extents = initial_aabb.max() - initial_aabb.min();
    Scalar max_extent = extents.maxCoeff();

    // Créer une AABB cubique
    VectorType center = initial_aabb.center();
    AabbType root_aabb;
    for (int i = 0; i < DIM; ++i)
    {
        root_aabb.min()[i] = center[i] - max_extent / 2;
        root_aabb.max()[i] = center[i] + max_extent / 2;
    }

    build_rec(0, 0, index_count(), root_aabb, 1);

    build_rec(0, 0, index_count(), root_aabb, 1);
}


template<typename Traits>
bool NDTreeBase<Traits>::valid() const
{
    if (m_points.empty())
        return m_nodes.empty() && m_indices.empty();

    if(m_nodes.empty() || m_indices.empty())
    {
        return false;
    }

    std::vector<bool> b(point_count(), false);
    for(IndexType idx : m_indices)
    {
        if(idx < 0 || point_count() <= idx || b[idx])
        {
            return false;
        }
        b[idx] = true;
    }

    for(NodeIndexType n = 0; n < node_count(); ++n)
    {
        const NodeType& node = m_nodes[n];
        if(node.is_leaf())
        {
            if(index_count() <= node.leaf_start() || node.leaf_start() + node.leaf_size() > index_count())
            {
                return false;
            }
        }
        else
        {
            for(int i = 0; i < CHILD_COUNT; ++i)
            {
                if(node.inner_child_id(i) != (NodeIndexType)-1 && 
                   (node_count() <= node.inner_child_id(i)))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

template<typename Traits>
std::string NDTreeBase<Traits>::to_string(bool verbose) const
{
    std::stringstream str;

    str << "NDTree:";
    str << "\n  Dimension: " << DIM;
    str << "\n  ChildCount: " << CHILD_COUNT;
    str << "\n  MaxDepth: " << MAX_DEPTH;
    str << "\n  PointCount: " << point_count();
    str << "\n  NodeCount: " << node_count();
    str << "\n  LeafCount: " << leaf_count();

    if (verbose)
    {
        // Add detailed information about nodes and points if needed
    }

    return str.str();
}

template<typename Traits>
void NDTreeBase<Traits>::build_rec(NodeIndexType node_id, IndexType start, IndexType end, const AabbType& aabb, int level)
{
    NodeType& node = m_nodes[node_id];

    node.set_is_leaf(
        end - start <= m_min_cell_size ||
        level >= MAX_DEPTH ||
        m_nodes.size() > (std::numeric_limits<NodeIndexType>::max)() - CHILD_COUNT);

    node.configure_range(start, end - start, aabb);
    node.set_point_count(end - start);

    if (node.is_leaf())
    {
        ++m_leaf_count;
    }
    else
    {
        VectorType center = aabb.center();
        std::array<IndexContainer, CHILD_COUNT> child_points;

        for (IndexType i = start; i < end; ++i)
        {
            int child_index = 0;
            const auto& pos = m_points[m_indices[i]].pos();
            for (int d = 0; d < DIM; ++d)
            {
                if (pos[d] >= center[d])
                {
                    child_index |= (1 << d);
                }
            }
            child_points[child_index].push_back(m_indices[i]);
        }

        std::array<NodeIndexType, CHILD_COUNT> child_ids;
        for (int i = 0; i < CHILD_COUNT; ++i)
        {
            if (!child_points[i].empty())
            {
                child_ids[i] = m_nodes.size();
                m_nodes.emplace_back();

                AabbType child_aabb = aabb;
                for (int d = 0; d < DIM; ++d)
                {
                    if (i & (1 << d))
                    {
                        child_aabb.min()[d] = center[d];
                    }
                    else
                    {
                        child_aabb.max()[d] = center[d];
                    }
                }

                IndexType child_start = start + (i > 0 ? child_points[i - 1].size() : 0);
                std::copy(child_points[i].begin(), child_points[i].end(), m_indices.begin() + child_start);
                build_rec(child_ids[i], child_start, child_start + child_points[i].size(), child_aabb, level + 1);
            }
            else
            {
                child_ids[i] = (NodeIndexType)-1; // Invalid child id
            }
        }

        node.configure_inner(child_ids);
    }
}

// Accessors
template<typename Traits>
typename NDTreeBase<Traits>::NodeIndexType NDTreeBase<Traits>::node_count() const
{
    return m_nodes.size();
}

template<typename Traits>
typename NDTreeBase<Traits>::IndexType NDTreeBase<Traits>::index_count() const
{
    return (IndexType)m_indices.size();
}

template<typename Traits>
typename NDTreeBase<Traits>::IndexType NDTreeBase<Traits>::point_count() const
{
    return (IndexType)m_points.size();
}

template<typename Traits>
typename NDTreeBase<Traits>::NodeIndexType NDTreeBase<Traits>::leaf_count() const
{
    return m_leaf_count;
}

template<typename Traits>
typename NDTreeBase<Traits>::PointContainer& NDTreeBase<Traits>::point_data()
{
    return m_points;
}

template<typename Traits>
const typename NDTreeBase<Traits>::PointContainer& NDTreeBase<Traits>::point_data() const
{
    return m_points;
}

template<typename Traits>
const typename NDTreeBase<Traits>::NodeContainer& NDTreeBase<Traits>::node_data() const
{
    return m_nodes;
}

template<typename Traits>
const typename NDTreeBase<Traits>::IndexContainer& NDTreeBase<Traits>::index_data() const
{
    return m_indices;
}

// Parameters
template<typename Traits>
typename NDTreeBase<Traits>::LeafSizeType NDTreeBase<Traits>::min_cell_size() const
{
    return m_min_cell_size;
}

template<typename Traits>
void NDTreeBase<Traits>::set_min_cell_size(LeafSizeType min_cell_size)
{
    m_min_cell_size = min_cell_size;
}

// You can add more methods here, such as nearest neighbor search, range queries, etc.
// For example:

// template<typename Traits>
// std::vector<typename NDTreeBase<Traits>::IndexType> 
// NDTreeBase<Traits>::range_query(const VectorType& point, Scalar radius) const
// {
//     std::vector<IndexType> result;
//     Scalar squared_radius = radius * radius;

//     std::function<void(const NodeType&)> search = [&](const NodeType& node) {
//         if (node.is_leaf())
//         {
//             for (IndexType i = node.leaf_start(); i < node.leaf_start() + node.leaf_size(); ++i)
//             {
//                 if ((m_points[m_indices[i]].pos() - point).squaredNorm() <= squared_radius)
//                 {
//                     result.push_back(m_indices[i]);
//                 }
//             }
//         }
//         else
//         {
//             for (int i = 0; i < CHILD_COUNT; ++i)
//             {
//                 NodeIndexType child_id = node.inner_child_id(i);
//                 if (child_id != (NodeIndexType)-1)
//                 {
//                     const NodeType& child = m_nodes[child_id];
//                     if (child.aabb.squaredExteriorDistance(point) <= squared_radius)
//                     {
//                         search(child);
//                     }
//                 }
//             }
//         }
//     };

//     if (!m_nodes.empty())
//     {
//         search(m_nodes[0]);
//     }

//     return result;
// }

// template<typename Traits>
// typename NDTreeBase<Traits>::IndexType 
// NDTreeBase<Traits>::nearest_neighbor(const VectorType& point) const
// {
//     if (m_points.empty())
//     {
//         return -1;
//     }

//     IndexType nearest_index = -1;
//     Scalar min_distance = std::numeric_limits<Scalar>::max();

//     std::function<void(const NodeType&)> search = [&](const NodeType& node) {
//         if (node.is_leaf())
//         {
//             for (IndexType i = node.leaf_start(); i < node.leaf_start() + node.leaf_size(); ++i)
//             {
//                 Scalar distance = (m_points[m_indices[i]].pos() - point).squaredNorm();
//                 if (distance < min_distance)
//                 {
//                     min_distance = distance;
//                     nearest_index = m_indices[i];
//                 }
//             }
//         }
//         else
//         {
//             std::array<std::pair<Scalar, int>, CHILD_COUNT> distances;
//             for (int i = 0; i < CHILD_COUNT; ++i)
//             {
//                 NodeIndexType child_id = node.inner_child_id(i);
//                 if (child_id != (NodeIndexType)-1)
//                 {
//                     const NodeType& child = m_nodes[child_id];
//                     distances[i] = {child.aabb.squaredExteriorDistance(point), i};
//                 }
//                 else
//                 {
//                     distances[i] = {std::numeric_limits<Scalar>::max(), i};
//                 }
//             }

//             std::sort(distances.begin(), distances.end());

//             for (const auto& [dist, i] : distances)
//             {
//                 if (dist > min_distance)
//                 {
//                     break;
//                 }
//                 NodeIndexType child_id = node.inner_child_id(i);
//                 if (child_id != (NodeIndexType)-1)
//                 {
//                     search(m_nodes[child_id]);
//                 }
//             }
//         }
//     };

//     if (!m_nodes.empty())
//     {
//         search(m_nodes[0]);
//     }

//     return nearest_index;
// }

// Add any additional methods as needed