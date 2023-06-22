#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>

enum class InterpType {SLERP, LERP};
template<typename T, bool _unitquat = false>
class quaternion{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>, "This class only supports floating point types");
    private:
        using qT = quaternion<T, _unitquat>;    

        T w_;
        T x_;
        T y_;
        T z_;  
    
    public:
        using value_type = T;
    
        quaternion(): w_(1), x_(0), y_(0), z_(0) {}
        quaternion(T w, T x, T y, T z): w_(w), x_(x), y_(y), z_(z) {

            if constexpr (_unitquat){
                normalise();
            }
        }
        
        quaternion(const qT& q_in) = default;
        constexpr qT& operator=(const qT& q_in) = default;
        
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

};

using quaternionf = quaternion<float>;
using quaterniond = quaternion<double>;
using quaternionUf = quaternion<float, true>;
using quaternionUd = quaternion<double, true>;

#endif