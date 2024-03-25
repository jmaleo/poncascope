#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>


namespace Ponca {

    template <typename Scalar>
    struct GaussianWeightKernelParameters
    {
        Scalar m_sigma_0;
        Scalar m_a;
        int m_k;

        // default constructor
        GaussianWeightKernelParameters() : m_sigma_0(1.), m_a(1.5), m_k(1) {}

        // constructor
        GaussianWeightKernelParameters(const Scalar& _sigma, const Scalar& _a, const int& _k) : m_sigma_0(_sigma), m_a(_a), m_k(_k) {}
    };

    template <typename Scalar>
    struct RationalWeightKernelParameters
    {
        Scalar m_k;
        Scalar m_epsilon;

        // default constructor
        RationalWeightKernelParameters() : m_k(1), m_epsilon(0.0) {}

        // constructor
        RationalWeightKernelParameters(const Scalar& _k, const Scalar& _epsilon) : m_k(_k), m_epsilon(_epsilon) {}
    };


    template <typename DataPoint>
    class GaussianMixtureWeightKernel {

        public:

            using Scalar = typename DataPoint::Scalar;
            using VectorType = typename DataPoint::VectorType;
            using WeightParam = GaussianWeightKernelParameters<Scalar>;

            /// @brief 
            /// @param _sigma Value of the parameter sigma_0 in the paper, used to compute the others sigma_i
            /// @param _a Value of the parameter a in the paper, used to compute sigma_i
            /// @param _k Number of basis functions (sigmas)
            /// @return 
            PONCA_MULTIARCH inline GaussianMixtureWeightKernel(const GaussianWeightKernelParameters<Scalar>& _param = GaussianWeightKernelParameters<Scalar>()){
                // Assert, m_a must be > 1.
                static_assert(_param.m_a > 1, "a must be > 1");
                m_parameters = _param;
            }

            PONCA_MULTIARCH inline void init( const VectorType& _evalPos )
            {
                m_q = _evalPos; // query point
            }

            PONCA_MULTIARCH inline const VectorType& basisCenter() const { return m_q; }

            PONCA_MULTIARCH inline Scalar w(const VectorType& point) const
            {
                Scalar weight = Scalar(0);

                Scalar dist = (m_q - point).squaredNorm();
                for ( int i = 0; i < m_parameters.m_k; i++ ){
                    Scalar sigma = std::pow( m_parameters.m_a, i ) * m_parameters.m_sigma_0;
                    weight += ( std::pow ( sigma, -3 ) * std::exp( -dist / (2 * sigma * sigma) ) );
                }

                return weight;
            }

        protected :

            GaussianWeightKernelParameters<Scalar> m_parameters;
            VectorType m_q;

    }; // class GaussianMixtureWeightKernel


    template <typename DataPoint>
    class RationalWeightKernel {

        public : 

            using Scalar = typename DataPoint::Scalar;
            using VectorType = typename DataPoint::VectorType;
            using WeightParam = RationalWeightKernelParameters<Scalar>;


            /// @brief 
            /// @param _k Value of the parameter k in the paper 
            /// @param _epsilon When epsilon is set to 0, it is for an interpolation. Otherwise (> 0) it is for an approximation (smoothing)
            /// @return 
            PONCA_MULTIARCH inline RationalWeightKernel( const RationalWeightKernelParameters<Scalar>& _param = RationalWeightKernelParameters<Scalar>() ){
                // m_espilon must be >= 0
                // static_assert(_param.m_epsilon >= Scalar ( 0 ), "epsilon must be >= 0");
                m_parameters = _param;
            }

            PONCA_MULTIARCH inline void init( const VectorType& _evalPos )
            {
                m_q = _evalPos; // query point
            }

            PONCA_MULTIARCH inline const VectorType& basisCenter() const { return m_q; }

            PONCA_MULTIARCH inline Scalar w(const VectorType& point) const
            {
                Scalar dist = (m_q - point).squaredNorm();
                Scalar weight = std::pow ( ( dist + m_parameters.m_epsilon ), -( m_parameters.m_k / Scalar(2) ) );
                return weight;
            }

            RationalWeightKernelParameters<Scalar> m_parameters;
        protected :

            VectorType m_q;

    }; // class RationalWeightKernel



} // namespace Ponca
