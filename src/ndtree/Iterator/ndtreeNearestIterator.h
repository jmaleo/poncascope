#pragma once

#include <iterator>

namespace Ponca
{

template<typename IndexType_>
class NDTreeNearestIterator
{
public:
    using IndexType = IndexType_;
    using iterator_category = std::forward_iterator_tag;
    using value_type = IndexType;
    using difference_type = std::ptrdiff_t;
    using pointer = IndexType*;
    using reference = IndexType&;

    inline NDTreeNearestIterator(IndexType index = -1) : m_index(index) {}

    inline IndexType operator*() const { return m_index; }

    inline NDTreeNearestIterator& operator++()
    {
        m_index = -1;
        return *this;
    }

    inline NDTreeNearestIterator operator++(int)
    {
        NDTreeNearestIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend inline bool operator==(const NDTreeNearestIterator& lhs, const NDTreeNearestIterator& rhs)
    {
        return lhs.m_index == rhs.m_index;
    }

    friend inline bool operator!=(const NDTreeNearestIterator& lhs, const NDTreeNearestIterator& rhs)
    {
        return !(lhs == rhs);
    }

private:
    IndexType m_index;
};

} // namespace Ponca