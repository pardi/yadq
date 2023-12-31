#ifndef DUAL_QUATERNION_HPP
#define DUAL_QUATERNION_HPP

// Version
#define DUAL_QUATERNION_VERSION_MAJOR 1
#define DUAL_QUATERNION_VERSION_MINOR 0
#define DUAL_QUATERNION_VERSION_PATCH 0

#define DUAL_QUATERNION_VERSION_CODE (DUAL_QUATERNION_VERSION_MAJOR * 10000 + \
                                 DUAL_QUATERNION_VERSION_MINOR * 100 + \
                                 DUAL_QUATERNION_VERSION_PATCH)

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>
#include <array>
#include <yadq/quaternion.hpp>

namespace yadq{

    /**
    * \class dualquaternion
    * \brief A class describing dual quaternions. Operations and data combination are automatically handled by the object itself.
    */
    template<typename _T>
    class dualquaternion{
        static_assert(std::is_same_v<_T, float> || std::is_same_v<_T, double>, "This class only supports floating point types");
        private:
            using dqT = dualquaternion<_T>;    

        public:
            quaternionU<_T> qr_;
            quaternion<_T> qd_;

            /**
             * \brief Empty constructor
             */
            dualquaternion() = default;
            /**
             * \brief Constructor from single components
             * \param qr unitary quaternion component for the rotation
             * \param qd quaternion component for the translation
             */
            dualquaternion(const quaternionU<_T>& qr, const quaternion<_T>& qd){
                qr_ = qr;
                qd_ = qd;
            }
           /**
             * \brief Constructor from rotation and translation
             * \param qr unitary quaternion representing the rotation of the dual quaternion
             * \param t vector representing the translation of the dual quaternion
             */
            dualquaternion(const quaternionU<_T>& r, const std::array<_T, 3> t){
                qr_ = r;
                quaternion<_T> q_t(0, t[0], t[1], t[2]);
                qd_ = 0.5 * q_t * r;
            }
           /**
             * \brief Rotate the dual quaternion given a rotation as input
             * \param q_rhv rotation represented as quaternion
             */
            template< typename T>
            constexpr dqT operator*(quaternionU<T> q_rhv) const noexcept{
                return dqT( qr_ * q_rhv,
                            qd_ * q_rhv);
            }
    };

    template<typename T>
    std::ostream& operator<<(std::ostream &os, const dualquaternion<T>& dq_in) noexcept{ 
        return os <<    "q [w: " << dq_in.qr_.w() << " x: " << dq_in.qr_.x() << " y: " << dq_in.qr_.y() << " z: " << dq_in.qr_.z() << "]" << std::endl <<
                        "t [w: " << dq_in.qd_.w() << " x: " << dq_in.qd_.x() << " y: " << dq_in.qd_.y() << " z: " << dq_in.qd_.z() << "]" ;
    }


    template< typename T>
    constexpr dualquaternion<T> operator*(quaternionU<T> lhv, dualquaternion<T> dq_rhv){
        return dq_rhv * lhv;
    }

    template<typename T>
    constexpr dualquaternion<T> operator*(dualquaternion<T> dq_lhv, const dualquaternion<T>& dq_rhv){
        
        dq_lhv.qr_ *= dq_rhv.qr_;
        dq_lhv.qd_ *= dq_rhv.qd_;

        return dq_lhv;
    }

    // TODO: [#1] Validate functions
    template<typename T>
    constexpr std::array<T, 3> operator*(const dualquaternion<T>& dq_lhv, const std::array<T, 3>& p_rhv){
        
        std::array<T, 3> p_new;
        dualquaternion<T> dq_t (quaternionU<T>(1, 0, 0, 0), p_rhv);

        dualquaternion<T> dq = dq_lhv * dq_t * conjugate(dq_lhv);

        return p_new;
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
