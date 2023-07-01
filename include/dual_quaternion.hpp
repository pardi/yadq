#ifndef DUAL_QUATERNION_HPP
#define DUAL_QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>

namespace yadq{

    template<typename _T>
    class dualquaternion{
        static_assert(std::is_same_v<_T, float> || std::is_same_v<_T, double>, "This class only supports floating point types");
        private:
            using dqT = dualquaternion<_T>;    

        public:
            quaternionU<_T> qr_;
            quaternion<_T> qd_;

            dualquaternion() = default;

            dualquaternion(const dqT& qr, const dqT& qd){
                qr_ = qr;
                qd_ = qd;
            }

            dualquaternion(const quaternionU<_T>& r, const std::array<_T, 3> t){
                qr_ = r;
                quaternion<_T> q_t(0, t[0], t[1], t[2]);
                qd_ = 0.5 * q_t * r;
            }
    };


    template<   typename T, 
                typename U>
    constexpr dualquaternion<T> operator*(dualquaternion<T> dq_lhv, U rhv){
        
        dq_lhv.qr_ *= rhv;
        dq_lhv.qd_ *= rhv; 

        return dq_lhv;
    }

    template<   typename T, 
                typename U>
    constexpr dualquaternion<T> operator*(U lhv, dualquaternion<T> dq_rhv){
        
        return dq_rhv * lhv;
    }

    template<typename T>
    constexpr dualquaternion<T> operator*(dualquaternion<T> dq_lhv, const dualquaternion<T>& dq_rhv){
        
        dq_lhv.qr_ *= dq_rhv.qr_;
        dq_lhv.qd_ *= dq_rhv.qd_;

        return dq_lhv;
    }

    template<typename T>
    constexpr dualquaternion<T> operator+(dualquaternion<T> dq_lhv, const dualquaternion<T>& dq_rhv){
        
        dq_lhv.qr_ += dq_rhv.qr_;
        dq_lhv.qd_ += dq_rhv.qd_;

        return dq_lhv;
    }
    
    template<typename T>
    constexpr dualquaternion<T> conjugate(dualquaternion<T> dq_lhv){
        
        dq_lhv.qr_.conjugate();
        dq_lhv.qd_.conjugate();
        
        return dq_lhv;
    } 

    template<typename T>
    constexpr dualquaternion<T> norm(const dualquaternion<T>& dq_lhv){
        
        return dq_lhv * conjugate(dq_lhv);
    } 
}

#endif