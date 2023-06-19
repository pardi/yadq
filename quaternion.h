#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <numeric>

enum class InterpType {SLERP, LERP};

template<typename T>
class quaternion;

template<typename T>
std::ostream& operator<<(std::ostream &os, const quaternion<T>& q_in) { 
    return os << "w: " << q_in.w() << " x: " << q_in.x() << " y: " << q_in.y() << " z: " << q_in.z();
}

template<typename T, typename U>
quaternion<T> operator*(double val, U&& q) { 

    quaternion<T> q_res(std::forward<U>(q));

    q_res.w_ *= val; 
    q_res.x_ *= val; 
    q_res.y_ *= val; 
    q_res.z_ *= val; 

    return normalise(q_res);
}

template<typename T, typename U>
quaternion<T> operator*(U&& q_in, double val) { 
    return val * std::forward(q_in);
}

template<typename T>
constexpr inline quaternion<T> operator*(const quaternion<T>& q_lhv, const quaternion<T>& q_rhv) {
    return hamilton_prod(q_lhv, q_rhv);
}

template<typename T, typename U>
quaternion<T> mod(U&& q){
    
    quaternion<T> q_res(std::forward<U>(q));

    q_res.w_ /= fabs(q.w());
    q_res.x_ /= fabs(q.x());
    q_res.y_ /= fabs(q.y());
    q_res.z_ /= fabs(q.z());

    return q_res;
}

template<typename T, typename U>
quaternion<T> operator+(quaternion<T> q_lhv, U&& q_rhv){

    q_lhv.w_ += q_rhv.w();
    q_lhv.x_ += q_rhv.x();
    q_lhv.y_ += q_rhv.y();
    q_lhv.z_ += q_rhv.z();

    return normalise(q_lhv);
}

template<typename T, typename U>
quaternion<T> operator-(U&& q_lhv, U&& q_rhv){

    quaternion<T> q_res(std::forward<U>(q_lhv));

    q_res.w_ -= q_rhv.w();
    q_res.x_ -= q_rhv.x();
    q_res.y_ -= q_rhv.y();
    q_res.z_ -= q_rhv.z();

    return normalise(q_res);
}

template<typename T>
constexpr inline quaternion<T> hamilton_prod(const quaternion<T>& q_lhv, const quaternion<T>& q_rhv) {
    quaternion<T> q_res;

    q_res.w_ = q_lhv.w() * q_rhv.w() - q_lhv.x() * q_rhv.x() - q_lhv.y() * q_rhv.y() - q_lhv.z() * q_rhv.z();
    q_res.x_ = q_lhv.w() * q_rhv.x() + q_lhv.x() * q_rhv.w() + q_lhv.y() * q_rhv.z() - q_lhv.z() * q_rhv.y();
    q_res.y_ = q_lhv.w() * q_rhv.y() - q_lhv.x() * q_rhv.z() + q_lhv.y() * q_rhv.w() + q_lhv.z() * q_rhv.x();
    q_res.z_ = q_lhv.w() * q_rhv.z() + q_lhv.x() * q_rhv.y() - q_lhv.y() * q_rhv.x() + q_lhv.z() * q_rhv.w();

    return normalise(q_res);
}

template<typename T>
 quaternion<decltype(std::declval<T>())> operator/(T&& q_lhv, double rhv) {

    quaternion<decltype(std::declval<T>())> q_res(std::forward<T>(q_lhv));

    q_res.w_ /= rhv; 
    q_res.x_ /= rhv; 
    q_res.y_ /= rhv; 
    q_res.z_ /= rhv; 

    return normalise(q_res);
}

template<typename T>
constexpr inline quaternion<T> normalise(quaternion<T> q){

    auto d = q.norm();

    if (d != 1.0) {
        return q / d;
    }

    return q;
}

template<typename T, typename U>
inline constexpr quaternion<T> conjugate(U&& q) {
    quaternion<T> q_res(std::forward(q));
    
    q_res.conjugate();

    return q_res;
}


template<typename T>
class quaternion{

    private:
    using qT = quaternion<T>;    

    public:

        quaternion(): w_(1), x_(0), y_(0), z_(0) {}
        quaternion(T w, T x, T y, T z): w_(w), x_(x), y_(y), z_(z) {
            normalise();
        }

        quaternion(const qT& q_in) = default;
        quaternion(qT&& q_in) = default;
        constexpr qT& operator=(const qT& q_in) = default;
        constexpr qT& operator=(qT&& q_in) = default;
        constexpr qT& operator+=(const qT& q_in) {
            w_ += q_in.w_;
            x_ += q_in.x_;
            y_ += q_in.y_;
            z_ += q_in.z_;

            normalise();
            return *this;
        }
        constexpr qT& operator*=(const qT& q_in){

            *this = (*this) * q_in;            

            return (*this);
        }

        qT interpolation(const qT& q_start, const qT& q_end, double t, InterpType interp_type = InterpType::LERP){
            switch (interp_type)
            {
            case InterpType::LERP:
                return q_start * (1.0 - t) + q_end * t;
                break;
            default:
                return q_start;
            }

        }

        qT exp(const qT& q, double t){
            
            qT q_conj = q.conjugate();

            qT u = (q - q_conj ) / 2;
            
            return q_conj;
            
        }

        constexpr bool empty() const{
            return (w_ == 0 && x_ == 0 && y_ == 0 && z_ == 0);
        }

        constexpr T sgn(const qT& q) const{

            if (q.empty()){
                return 0;
            } else {
                return q / q.mod();
            }            
        }

        inline constexpr void normalise() {

            auto d = norm();

            if (d != 1.0) {
                w_ /= d;
                x_ /= d;
                y_ /= d;
                z_ /= d;
            }
        }

        inline constexpr qT& conjugate() {
            x_ *= -1;
            y_ *= -1;
            z_ *= -1;
            return (*this);
        }

        inline constexpr qT inverse() {
            // Check if a non-zero quaternion
            // qT q_conj(std::move(conjugate(*this)));
            // qT q_tmp(q_conj * (*this));

            // auto val = (q_tmp.w() + q_tmp.x() + q_tmp.y() + q_tmp.z());

            // q_conj.w_ /= val;
            // q_conj.x_ /= val;
            // q_conj.y_ /= val;
            // q_conj.z_ /= val;
            qT q_conj;

            return q_conj;
        }

        inline T norm() const noexcept{
            return std::sqrt(std::pow(w_, 2) + std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2));
        }

        inline T w() const noexcept{
            return w_;
        }

        inline T x() const noexcept{
            return x_;
        }

        inline T y() const noexcept{
            return y_;
        }

        inline T z() const noexcept{
            return z_;
        }

        T w_;
        T x_;
        T y_;
        T z_;  

};
#endif