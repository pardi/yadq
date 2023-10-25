
#ifndef QUATERNION_HPP
#define QUATERNION_HPP

// Version
#define QUATERNION_VERSION_MAJOR 1
#define QUATERNION_VERSION_MINOR 0
#define QUATERNION_VERSION_PATCH 0

#define QUATERNION_VERSION_CODE (QUATERNION_VERSION_MAJOR * 10000 + \
                                 QUATERNION_VERSION_MINOR * 100 + \
                                 QUATERNION_VERSION_PATCH)

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>
#include <array>
#include <yadq/yadq_type_traits.hpp>

namespace yadq{

    enum class InterpType {SLERP, LERP};

    /**
    * \class quaternion
    * \brief A class describing general quaternions and providing operations between quaternions objects.
    */
    template<typename _T>
    class quaternion{
        static_assert(std::is_same_v<_T, float> || std::is_same_v<_T, double>, "This class only supports floating point types");
        private:
            using qT = quaternion<_T>;    

        protected:
            std::array<_T, 4> data_;
            _T& w_{data_[0]};
            _T& x_{data_[1]};
            _T& y_{data_[2]};
            _T& z_{data_[3]};  
        public:
            
            using value_type = _T;

            /**
             * \brief Empty constructor
             */
            quaternion(): data_{1, 0, 0, 0} {}
            /**
             * \brief Constructor with single parameters
             * \param x X component of the quaternion
             * \param y Y component of the quaternion
             * \param z Z component of the quaternion
             * \param w W component of the quaternion
             */
            quaternion(_T w, _T x, _T y, _T z): data_{w, x, y, z} {}
            /**
             * \brief Copy constructor
             * \param q_in object to copy
             */
            quaternion(const qT& q_in): data_(q_in.data_) {}
            /**
             * \brief Assignment operator
             * \param q_in object to copy
             */
            constexpr qT& operator=(const qT& q_in){
                data_ = q_in.data_;
                return (*this);
            } 
            /**
             * \brief Sum and assign to quaternions
             * \param q_in object to add
             */
            constexpr qT& operator+=(const qT& q_in) noexcept{
    
                w_ += q_in.w_;
                x_ += q_in.x_;
                y_ += q_in.y_;
                z_ += q_in.z_;

                return (*this);
            }
            /**
             * \brief Divide the quaternion by a scalar
             * \param rhv value to divide by
             */
            constexpr qT& operator/=(const _T rhv) {
                w_ /= rhv;
                x_ /= rhv;
                y_ /= rhv;
                z_ /= rhv;

                return (*this);
            }
            /**
             * \brief Compute the product of two quaternions
             * \param q_in quaternion to multipyly
             */
            constexpr qT& operator*=(const qT& q_in) noexcept{

                *this = (*this) * q_in;            

                return (*this);
            }
            /**
             * \brief Multiply a scalar to the quaternion
             * \param value value to multiply
             */
            constexpr qT operator*(double value) const noexcept{
                return qT(  w_ * value, 
                            x_ * value, 
                            y_ * value, 
                            z_ * value);
            }
            /**
             * \brief Add a scalar to the quaternion
             * \param value value to add
             */
            constexpr qT operator+(double value) const noexcept{
                return qT(  w_ + value, 
                            x_ + value, 
                            y_ + value, 
                            z_ + value);
            }
            /**
             * \brief Subtract a scalar to the quaternion-
             * \param value value to subtract
             */
            constexpr qT operator-(double value) const noexcept{

                return qT(  w_ - value,
                            x_ - value,
                            y_ - value,
                            z_ - value);
            }
            /**
             * \brief Check if the quaternion is empty
             */
            constexpr bool empty() const{
                return (w_ == 0 && x_ == 0 && y_ == 0 && z_ == 0);
            }
            /**
             * \brief Compute the norm of the quaternion
             */
            constexpr inline auto norm() const noexcept{
                return std::sqrt(std::pow(w_, 2) + std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2));
            }
            /**
             * \brief Return w component of the quaternion
             */
            inline auto w() const noexcept{
                return w_;
            }
            /**
             * \brief Return x component of the quaternion
             */
            inline auto x() const noexcept{
                return x_;
            }
            /**
             * \brief Return y component of the quaternion
             */
            inline auto y() const noexcept{
                return y_;
            }
            /**
             * \brief Return z component of the quaternion
             */
            inline auto z() const noexcept{
                return z_;
            }
            /**
             * \brief Get the raw data
             */
            inline auto& get() const noexcept{
                return data_;
            }
            /**
             * \brief Normalise the quaternion
             */
            constexpr inline void normalise() {

                if(auto d = norm(); d != 0.0) {
                    w_ /= d;
                    x_ /= d;
                    y_ /= d;
                    z_ /= d;
                }
            }
            /**
             * \brief Conjugate the quaternion
             */
            constexpr inline void conjugate() {

                x_ *= -1;
                y_ *= -1;
                z_ *= -1;

            }

    };

    /**
    * \class quaternionU
    * \brief A class representing unitary quaternions. The normalisation is done internally at every operations.
    */
    template<typename _T>
    class quaternionU : public quaternion<_T>{
        public:
            /**
            * Exposing quaternion<T> functions
            */
            using quaternion<_T>::operator*;
            using quaternion<_T>::operator+;
            using quaternion<_T>::operator-;

            /**
             * \brief Empty constructor
             */
            quaternionU(): quaternion<_T>(){}
            /**
             * \brief Copy constructor from a quaternion object
             */
            quaternionU(const quaternion<_T>& q_in): quaternion<_T>(q_in){
                this->normalise();
            }
            /**
             * \brief Constructor with single parameters
             * \param x X component of the quaternion
             * \param y Y component of the quaternion
             * \param z Z component of the quaternion
             * \param w W component of the quaternion
             */
            quaternionU(_T w, _T x, _T y, _T z): quaternion<_T>(w, x, y, z) {
                quaternion<_T>::normalise();
            }
            /**
             * \brief Constructor by axis/angle
             * \param axis three coordinates of the axis
             * \param angle rotation angle around the axis
             */
            quaternionU(const std::array<_T, 3>& axis, _T angle) {
                
                this->w_ = std::cos(angle / 2.0);

                auto axis_norm = std::sqrt(std::pow(axis[0], 2) + std::pow(axis[1], 2) + std::pow(axis[2], 2));
                this->x_ = axis[0] * std::sin(angle / 2.0) / axis_norm;
                this->y_ = axis[1] * std::sin(angle / 2.0) / axis_norm;
                this->z_ = axis[2] * std::sin(angle / 2.0) / axis_norm;

                this->normalise();
            }
            /**
             * \brief Copy contructor by unitary quaternions
             * \param q_in unitary quaternion to copy
             */
            quaternionU(const quaternionU<_T>& q_in): quaternion<_T>(q_in) {
                this->normalise();
            }
            /**
             * \brief Default assignment operator
             * \param q_in unitary quaternion to copy
             */
            constexpr quaternionU<_T>& operator=(const quaternionU<_T>& q_in) = default;
           /**
             * \brief Sum and assign to unitary quaternions
             * \param q_in object to add
             */
            constexpr quaternionU<_T>& operator+=(const quaternionU<_T>& q_in){
                    
                *this = static_cast<quaternion<_T>>(*this) + q_in;

                quaternion<_T>::normalise();
                return (*this);
            }
    };

}

#include <yadq/impl/quaternion.tpp>

#endif
