#pragma once

#include <iterator>
#include <queue>

namespace Ponca
{

template<typename IndexType_, typename ScalarType_>
class NDTreeKNearestIterator
{
public:
    using IndexType = IndexType_;
    using ScalarType = ScalarType_;
    using iterator_category = std::forward_iterator_tag;
    using value_type = IndexType;
    using difference_type = std::ptrdiff_t;
    using pointer = IndexType*;
    using reference = IndexType&;

    using QueueType = std::priority_queue<std::pair<ScalarType, IndexType>>;

    inline NDTreeKNearestIterator(typename QueueType::const_iterator it) : m_it(it) {}

    inline IndexType operator*() const { return m_it->second; }

    inline NDTreeKNearestIterator& operator++()
    {
        ++m_it;
        return *this;
    }

    inline NDTreeKNearestIterator operator++(int)
    {
        NDTreeKNearestIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend inline bool operator==(const NDTreeKNearestIterator& lhs, const NDTreeKNearestIterator& rhs)
    {
        return lhs.m_it == rhs.m_it;
    }

    friend inline bool operator!=(const NDTreeKNearestIterator& lhs, const NDTreeKNearestIterator& rhs)
    {
        return !(lhs == rhs);
    }

private:
    typename QueueType::const_iterator m_it;
};

} // namespace Ponca