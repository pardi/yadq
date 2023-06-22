#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>

namespace yadq{

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

            protected:

            inline constexpr void normalise() {

                if(auto d = norm(); d != 0.0) {
                    w_ /= d;
                    x_ /= d;
                    y_ /= d;
                    z_ /= d;
                }
            }

            inline constexpr void conjugate() {

                x_ *= -1;
                y_ *= -1;
                z_ *= -1;

            }

    };

    template<bool _activate_flag = true>
    inline auto normalise(auto q){

        if constexpr (_activate_flag){
            if(auto d = q.norm(); d != 0.0) {
                q.w_ /= d;
                q.x_ /= d;
                q.y_ /= d;
                q.z_ /= d;
            }
        }

        return q;
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
    quaternion<T> mod(quaternion<T> q){
        
        q.w_ /= fabs(q.w());
        q.x_ /= fabs(q.x());
        q.y_ /= fabs(q.y());
        q.z_ /= fabs(q.z());

        return q;
    }


    using quaternionf = quaternion<float>;
    using quaterniond = quaternion<double>;
    using quaternionUf = quaternion<float, true>;
    using quaternionUd = quaternion<double, true>;
}

#endif