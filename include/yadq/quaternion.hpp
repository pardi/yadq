
#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>
#include <array>
#include <yadq/yadq_type_traits.hpp>

namespace yadq{

    enum class InterpType {SLERP, LERP};

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
        
            quaternion(): data_{1, 0, 0, 0} {}
            quaternion(_T w, _T x, _T y, _T z): data_{w, x, y, z} {}
            quaternion(const qT& q_in): data_(q_in.data_) {}
            constexpr qT& operator=(const qT& q_in){
                data_ = q_in.data_;
                return (*this);
            } 

            constexpr qT& operator+=(const qT& q_in) noexcept{
    
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

            constexpr qT& operator*=(const qT& q_in) noexcept{

                *this = (*this) * q_in;            

                return (*this);
            }

            constexpr qT operator*(double value) const noexcept{
                return qT(  w_ * value, 
                            x_ * value, 
                            y_ * value, 
                            z_ * value);
            }

            constexpr qT operator+(double value) const noexcept{
                return qT(  w_ + value, 
                            x_ + value, 
                            y_ + value, 
                            z_ + value);
            }

            constexpr qT operator-(double value) const noexcept{

                return qT(  w_ - value,
                            x_ - value,
                            y_ - value,
                            z_ - value);
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

            inline auto& data() const noexcept{
                return data_;
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
            quaternionU(const quaternion<_T>& q_in): quaternion<_T>(q_in){
                this->normalise();
            }
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

            quaternionU(const quaternionU<_T>& q_in): quaternion<_T>(q_in) {
                this->normalise();
            }

            constexpr quaternionU<_T>& operator=(const quaternionU<_T>& q_in) = default;

            constexpr quaternionU<_T>& operator+=(const quaternionU<_T>& q_in){
            
                *this = static_cast<quaternion<_T>>(*this) + q_in;

                this->normalise();
                return (*this);
            }

            constexpr quaternionU<_T> operator*(double value) const noexcept{
                return quaternionU<_T>(static_cast<quaternion<_T>>(*this) * value);
            }

            constexpr quaternionU<_T> operator+(double value) const noexcept{
                return quaternionU<_T>(static_cast<quaternion<_T>>(*this) + value);
            }

            constexpr quaternionU<_T> operator-(double value) const noexcept{
                return quaternionU<_T>(static_cast<quaternion<_T>>(*this) - value);
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
                typename = std::enable_if_t<is_base_of_quaternion_v<T>>>
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
                typename U, 
                typename = std::enable_if_t<is_base_of_quaternion_v<T> && is_base_of_quaternion_v<U>>>
    constexpr inline auto operator+(const T& q_lhv, const U& q_rhv) noexcept{

        T q_res(    q_lhv.w() + q_rhv.w(),
                    q_lhv.x() + q_rhv.x(),
                    q_lhv.y() + q_rhv.y(),
                    q_lhv.z() + q_rhv.z());

        if constexpr (is_quaternionU_v<T> && is_quaternionU_v<U>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }    

    template<   typename T,
                typename U,
                typename = std::enable_if_t<is_base_of_quaternion_v<T> && is_base_of_quaternion_v<U>>>
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

    template<   typename T,
                typename = std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr auto operator*(double lhv, T q_rhv) noexcept{
        return q_rhv * lhv;
    }

    template<   typename T,
                typename = std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr auto operator+(double lhv, T q_rhv) noexcept{
        return q_rhv + lhv;
    }

    template<   typename T,
                typename U,
                typename = std::enable_if_t<is_base_of_quaternion_v<T> && is_base_of_quaternion_v<U>>>
    constexpr inline auto operator-(const T& q_lhv, const U& q_rhv) noexcept{

        T q_res(    q_lhv.w() - q_rhv.w(),
                    q_lhv.x() - q_rhv.x(),
                    q_lhv.y() - q_rhv.y(),
                    q_lhv.z() - q_rhv.z());

        if constexpr (is_quaternionU_v<T> && is_quaternionU_v<U>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }

    /*
        ------------------------------ Fcn definition ------------------------------
    */

    template<   typename T, 
                typename =  std::enable_if_t<is_base_of_quaternion_v<T>>>
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
                typename =  std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr inline auto hamilton_prod(const T& q_lhv, const T& q_rhv) noexcept{

        T q_res(    q_lhv.w() * q_rhv.w() - q_lhv.x() * q_rhv.x() - q_lhv.y() * q_rhv.y() - q_lhv.z() * q_rhv.z(),
                    q_lhv.w() * q_rhv.x() + q_lhv.x() * q_rhv.w() + q_lhv.y() * q_rhv.z() - q_lhv.z() * q_rhv.y(),
                    q_lhv.w() * q_rhv.y() - q_lhv.x() * q_rhv.z() + q_lhv.y() * q_rhv.w() + q_lhv.z() * q_rhv.x(),
                    q_lhv.w() * q_rhv.z() + q_lhv.x() * q_rhv.y() - q_lhv.y() * q_rhv.x() + q_lhv.z() * q_rhv.w());

        return q_res;
    }


    template< typename T>
    constexpr auto inverse(const quaternionU<T>& q_in) {

        if(!q_in.empty()){
            quaternionU<T> q_conj = q_in;
            q_conj.conjugate();

            return q_conj / pow(q_in.norm(), 2);
        }
        else{
            return quaternionU<T>(0, 0, 0, 0);
        }
    }

    template<   typename T,
                typename =  std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr auto conjugate(const T& q) noexcept{
        T q_res(q);
        q_res.conjugate();
        return q_res;
    }

    template<   typename T,
                typename =  std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr auto exp(const T& q_in) noexcept{

        T v(0, q_in.x(), q_in.y(), q_in.z());
        auto v_norm = v.norm();

        return std::exp(q_in.w()) + (std::cos(v_norm) + (v / v_norm) * std::sin(v_norm));
    }

    template<   typename T,
                typename =  std::enable_if_t<is_base_of_quaternion_v<T>>>
    constexpr std::optional<T> log(const T& q_in) noexcept{

        if (q_in.empty()){
            return std::nullopt;
        }

        T v(0, q_in.x(), q_in.y(), q_in.z());
        auto v_norm = v.norm();

        auto theta = std::acos(q_in.w() / q_in.norm());

        return std::log(v_norm) + (v / v_norm) * theta;
    }


    template<   typename T,
                typename U,
                typename = std::enable_if_t<is_base_of_quaternion_v<T> && is_base_of_quaternion_v<U>>>
    constexpr auto dot(const T& q_lhv, const U& q_rhv) noexcept{
        return q_lhv.x() * q_rhv.x() + q_lhv.y() * q_rhv.y() + q_lhv.z() * q_rhv.z();
    }

    template< typename T>
    constexpr quaternionU<T> interpolation(const quaternionU<T>& q_start, const quaternionU<T>& q_end, double t, InterpType interp_type = InterpType::LERP) noexcept{
        switch (interp_type)
        {
        case InterpType::LERP:{                
                return q_start * (1.0 - t) + q_end * t;
            }
        case InterpType::SLERP:{
                auto omega = dot(q_start, q_end);
                return ((std::sin(1 - t) * omega) / std::sin(omega)) * q_start + (sin(t * omega) / sin(omega)) * q_end;
            }
        default: {
                return q_start;
            }
        }
    }

    template< typename T>
    constexpr auto quatToRotation(const quaternionU<T>& q_in) noexcept{

        auto a0 = pow(q_in.w(), 2);
        auto a1 = pow(q_in.x(), 2);
        auto a2 = pow(q_in.y(), 2);
        auto a3 = pow(q_in.z(), 2);

        auto a4 = q_in.w() * q_in.x();
        auto a5 = q_in.w() * q_in.y();
        auto a6 = q_in.w() * q_in.z();

        auto a7 = q_in.x() * q_in.y();
        auto a8 = q_in.x() * q_in.z();
        
        auto a9 = q_in.y() * q_in.z();

        std::array<T, 9> R = {  2 * (a0 + a1) - 1, 2 * (a7 - a6), 2 * (a8 + a5),
                                2 * (a7 + a6), 2 * (a0 + a2) - 1, 2 * (a9 - a4),
                                2 * (a8 - a5), 2 * (a9 + a4), 2 * (a0 + a3) - 1};

        return R;
    }

}


#endif