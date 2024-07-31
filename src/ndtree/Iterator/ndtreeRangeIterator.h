#pragma once

#include <iterator>

namespace Ponca
{

template<typename IndexType_, typename DataPoint_, typename QueryType_>
class NDTreeRangeIterator
{
public:
    using IndexType = IndexType_;
    using DataPoint = DataPoint_;
    using QueryType = QueryType_;
    using iterator_category = std::forward_iterator_tag;
    using value_type = IndexType;
    using difference_type = std::ptrdiff_t;
    using pointer = IndexType*;
    using reference = IndexType&;

    inline NDTreeRangeIterator(QueryType* query = nullptr, IndexType index = -1)
        : m_query(query), m_index(index), m_start(0), m_end(0) {}

    inline IndexType operator*() const { return m_index; }

    inline NDTreeRangeIterator& operator++()
    {
        m_query->advance(*this);
        return *this;
    }

    inline NDTreeRangeIterator operator++(int)
    {
        NDTreeRangeIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend inline bool operator==(const NDTreeRangeIterator& lhs, const NDTreeRangeIterator& rhs)
    {
        return lhs.m_index == rhs.m_index;
    }

    friend inline bool operator!=(const NDTreeRangeIterator& lhs, const NDTreeRangeIterator& rhs)
    {
        return !(lhs == rhs);
    }

private:
    friend QueryType;
    QueryType* m_query;
    IndexType m_index;
    IndexType m_start;
    IndexType m_end;
};

} // namespace Ponca