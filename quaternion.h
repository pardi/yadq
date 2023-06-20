#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <numeric>

enum class InterpType {SLERP, LERP};

template<typename T, typename = std::enable_if_t<std::is_same<T, float>::value || std::is_same<T, double>::value>>
class quaternion;

template<bool _activate_flag = true>
inline auto normalise(auto q){

    if constexpr (_activate_flag){
        return q.normalise();
    }
    else {
        return q;
    }
}

template<typename T>
constexpr inline T sgn(const quaternion<T>& q_in){

    if (q_in.empty()){
        return quaternion<T>(0, 0, 0, 0);
    } else {
        return (q_in / mod(q_in));
    }            
}

template<typename T>
std::ostream& operator<<(std::ostream &os, const quaternion<T>& q_in) { 
    return os << "w: " << q_in.w() << " x: " << q_in.x() << " y: " << q_in.y() << " z: " << q_in.z();
}

template<typename T>
auto operator*(double val, quaternion<T>&& q){ 

    auto q_res(std::forward<T>(q));

    q_res.w_ *= val; 
    q_res.x_ *= val; 
    q_res.y_ *= val; 
    q_res.z_ *= val; 

    return normalise(q_res);
}

template<typename T>
auto operator*(quaternion<T>&& q_in, double val) { 
    return val * std::forward(q_in);
}

template<typename T>
constexpr inline auto operator*(const quaternion<T>& q_lhv, const quaternion<T>& q_rhv) {
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

template<typename T>
auto operator+(quaternion<T> q_lhv, const quaternion<T>& q_rhv){

    q_lhv.w_ += q_rhv.w();
    q_lhv.x_ += q_rhv.x();
    q_lhv.y_ += q_rhv.y();
    q_lhv.z_ += q_rhv.z();

    return normalise(q_lhv);
}

template<typename T, typename U>
constexpr inline auto operator-(quaternion<T>&& q_lhv, quaternion<T>&& q_rhv){

    auto q_res(std::forward<U>(q_lhv));

    q_res.w_ -= q_rhv.w();
    q_res.x_ -= q_rhv.x();
    q_res.y_ -= q_rhv.y();
    q_res.z_ -= q_rhv.z();

    return normalise(q_res);
}

template<typename T>
constexpr inline auto hamilton_prod(const quaternion<T>& q_lhv, const quaternion<T>& q_rhv) {
    
    quaternion<T> q_res;

    q_res.w_ = q_lhv.w() * q_rhv.w() - q_lhv.x() * q_rhv.x() - q_lhv.y() * q_rhv.y() - q_lhv.z() * q_rhv.z();
    q_res.x_ = q_lhv.w() * q_rhv.x() + q_lhv.x() * q_rhv.w() + q_lhv.y() * q_rhv.z() - q_lhv.z() * q_rhv.y();
    q_res.y_ = q_lhv.w() * q_rhv.y() - q_lhv.x() * q_rhv.z() + q_lhv.y() * q_rhv.w() + q_lhv.z() * q_rhv.x();
    q_res.z_ = q_lhv.w() * q_rhv.z() + q_lhv.x() * q_rhv.y() - q_lhv.y() * q_rhv.x() + q_lhv.z() * q_rhv.w();

    return normalise(q_res);
}

template<typename T>
constexpr inline quaternion<T> operator/(quaternion<T> q_lhv, double rhv) {

    q_lhv.w_ /= rhv; 
    q_lhv.x_ /= rhv; 
    q_lhv.y_ /= rhv; 
    q_lhv.z_ /= rhv; 

    return normalise<false>(q_lhv);
}



template<typename T, typename U>
inline constexpr quaternion<T> conjugate(U&& q) {
    quaternion<T> q_res(std::forward(q));

    return q_res.conjugate();
}


template<typename T>
inline constexpr quaternion<T> inverse(quaternion<T> q_in) {
    // Check if a non-zero quaternion
    quaternion<T> q_conj = q_in.conjugate();

    return q_conj / pow(q_in.norm(), 2);
}


template<typename T>
class quaternion<T, typename std::enable_if_t<std::is_same<T, float>::value || std::is_same<T, double>::value>>{

    private:
        using qT = quaternion<T>;    
    
    public:
        using value_type = T;
    
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
        
        constexpr bool empty() const{
            return (w_ == 0 && x_ == 0 && y_ == 0 && z_ == 0);
        }
        
        constexpr inline T norm() const noexcept{
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

        inline constexpr qT normalise() {

            
            if(auto d = norm(); d != 0.0) {
                w_ /= d;
                x_ /= d;
                y_ /= d;
                z_ /= d;
            }

            return *this;
        }

        inline constexpr qT conjugate() {

            qT q_res(*this);

            q_res.x_ *= -1;
            q_res.y_ *= -1;
            q_res.z_ *= -1;

            return q_res;
        }


        // qT interpolation(const qT& q_start, const qT& q_end, double t, InterpType interp_type = InterpType::LERP){
        //     switch (interp_type)
        //     {
        //     case InterpType::LERP:
        //         return q_start * (1.0 - t) + q_end * t;
        //         break;
        //     default:
        //         return q_start;
        //     }

        // }

        // qT exp(const qT& q, double t){
            
        //     qT q_conj = q.conjugate();

        //     qT u = (q - q_conj ) / 2;
            
        //     return q_conj;
            
        // }


        T value;

        T w_;
        T x_;
        T y_;
        T z_;  

};
#endif