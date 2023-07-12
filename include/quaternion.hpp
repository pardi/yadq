
#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>
#include <array>
#include <yadq_type_traits.hpp>

namespace yadq{

    enum class InterpType {SLERP, LERP};

    template<typename _T>
    class quaternion{
        static_assert(std::is_same_v<_T, float> || std::is_same_v<_T, double>, "This class only supports floating point types");
        private:
            using qT = quaternion<_T>;    

        protected:
            _T w_;
            _T x_;
            _T y_;
            _T z_;  
        
        public:
            using value_type = _T;
        
            quaternion(): w_(1), x_(0), y_(0), z_(0) {}
            quaternion(_T w, _T x, _T y, _T z): w_(w), x_(x), y_(y), z_(z) {}
            
            quaternion(const qT& q_in) = default;
            constexpr qT& operator=(const qT& q_in) = default;
            constexpr qT& operator+=(const qT& q_in) {
                w_ += q_in.w_;
                x_ += q_in.x_;
                y_ += q_in.y_;
                z_ += q_in.z_;

                return (*this);
            }
            constexpr qT& operator/=(const _T rhv) {
                w_ /= rhv;
                x_ /= rhv;
                y_ /= rhv;
                z_ /= rhv;

                return (*this);
            }
            constexpr qT& operator*=(const qT& q_in){

                *this = (*this) * q_in;            

                return (*this);
            }
            constexpr bool empty() const{
                return (w_ == 0 && x_ == 0 && y_ == 0 && z_ == 0);
            }
            
            constexpr inline auto norm() const noexcept{
                return std::sqrt(std::pow(w_, 2) + std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2));
            }

            inline auto w() const noexcept{
                return w_;
            }

            inline auto x() const noexcept{
                return x_;
            }

            inline auto y() const noexcept{
                return y_;
            }

            inline auto z() const noexcept{
                return z_;
            }

            constexpr inline void normalise() {

                if(auto d = norm(); d != 0.0) {
                    w_ /= d;
                    x_ /= d;
                    y_ /= d;
                    z_ /= d;
                }
            }

            constexpr inline void conjugate() {

                x_ *= -1;
                y_ *= -1;
                z_ *= -1;

            }

    };

    template<typename _T>
    class quaternionU : public quaternion<_T>{
        public:
            quaternionU(): quaternion<_T>(){}
            quaternionU(_T w, _T x, _T y, _T z): quaternion<_T>(w, x, y, z) {
                this->normalise();
            }

            quaternionU(const std::array<_T, 3>& axis, _T angle) {
                
                this->w_ = std::cos(angle / 2.0);

                auto axis_norm = std::sqrt(std::pow(axis[0], 2) + std::pow(axis[1], 2) + std::pow(axis[2], 2));
                this->x_ = axis[0] * std::sin(angle / 2.0) / axis_norm;
                this->y_ = axis[1] * std::sin(angle / 2.0) / axis_norm;
                this->z_ = axis[2] * std::sin(angle / 2.0) / axis_norm;

                this->normalise();
            }

            quaternionU(const quaternionU<_T>& q_in): quaternion<_T>(q_in) {}
            constexpr quaternionU<_T>& operator=(const quaternionU<_T>& q_in) = default;

            constexpr quaternionU<_T>& operator+=(const quaternionU<_T>& q_in){
            
                static_cast<quaternion<_T>>(*this) = static_cast<quaternion<_T>>(*this) + static_cast<quaternion<_T>>(q_in);

                this->normalise();
                return (*this);
            }
    };


    using quaternionf = quaternion<float>;
    using quaterniond = quaternion<double>;
    using quaternionUf = quaternionU<float>;
    using quaternionUd = quaternionU<double>;

    /*
        ------------------------------ Operators definition ------------------------------
    */ 

    template<   typename T, 
                typename = std::enable_if_t<is_base_of_template_v<T, quaternion>>>
    constexpr inline auto operator/(const T& q_lhv, double rhv) {

        T q_res(    q_lhv.w() / rhv,
                    q_lhv.x() / rhv,
                    q_lhv.y() / rhv,
                    q_lhv.z() / rhv);

       if constexpr (is_quaternionU_v<T>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }

    template<   typename T, 
                typename = std::enable_if_t<is_base_of_template_v<T, quaternion>>>
    constexpr inline auto operator+(const T& q_lhv, const T& q_rhv) noexcept{

        T q_res(    q_lhv.w() + q_rhv.w(),
                    q_lhv.x() + q_rhv.x(),
                    q_lhv.y() + q_rhv.y(),
                    q_lhv.z() + q_rhv.z());

        if constexpr (is_quaternionU_v<T>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }    


    template<   typename T,
                typename U,
                typename = std::enable_if_t<is_base_of_template_v<T, quaternion> && is_base_of_template_v<U, quaternion>>>
    constexpr auto operator*(const T& q_lhv, const U& q_rhv) noexcept{

        if constexpr (is_quaternion_v<T> && is_quaternionU_v<U>){
            return hamilton_prod(q_lhv, static_cast<T>(q_rhv));
        }
        
        if constexpr (is_quaternionU_v<T> && is_quaternion_v<U>){
            return hamilton_prod(static_cast<U>(q_lhv), q_rhv);
        }

        if constexpr ((is_quaternionU_v<T> && is_quaternionU_v<U>) || (is_quaternion_v<T> && is_quaternion_v<U>)){
            return hamilton_prod(q_lhv, q_rhv);
        }
    }

    /*
        ------------------------------ Fcn definition ------------------------------
    */

    template<   typename T, 
                typename =  std::enable_if_t<is_base_of_template_v<T, quaternion>>>
    constexpr auto normalise(const T& q_in) noexcept{

        if(auto d = q_in.norm(); d != 0.0) {

            T q_res(    q_in.w() / d, 
                        q_in.x() / d, 
                        q_in.y() / d, 
                        q_in.z() / d);
            return q_res;
        }
    
        return q_in;
  
    }

    template<typename T>
    std::ostream& operator<<(std::ostream &os, const quaternion<T>& q_in) noexcept{ 
        return os << "w: " << q_in.w() << " x: " << q_in.x() << " y: " << q_in.y() << " z: " << q_in.z();
    }

    template<   typename T,
                typename =  std::enable_if_t<is_base_of_template_v<T, quaternion>>>
    constexpr inline auto hamilton_prod(const T& q_lhv, const T& q_rhv) noexcept{

        T q_res(    q_lhv.w() * q_rhv.w() - q_lhv.x() * q_rhv.x() - q_lhv.y() * q_rhv.y() - q_lhv.z() * q_rhv.z(),
                    q_lhv.w() * q_rhv.x() + q_lhv.x() * q_rhv.w() + q_lhv.y() * q_rhv.z() - q_lhv.z() * q_rhv.y(),
                    q_lhv.w() * q_rhv.y() - q_lhv.x() * q_rhv.z() + q_lhv.y() * q_rhv.w() + q_lhv.z() * q_rhv.x(),
                    q_lhv.w() * q_rhv.z() + q_lhv.x() * q_rhv.y() - q_lhv.y() * q_rhv.x() + q_lhv.z() * q_rhv.w());

        return q_res;
    }
}

#endif