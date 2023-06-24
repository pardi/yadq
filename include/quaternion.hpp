#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>

namespace yadq{

    enum class InterpType {SLERP, LERP};

    template<typename T>
    class quaternion{
        static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>, "This class only supports floating point types");
        private:
            using qT = quaternion<T>;    

            T w_;
            T x_;
            T y_;
            T z_;  
        
        public:
            using value_type = T;
        
            quaternion(): w_(1), x_(0), y_(0), z_(0) {}
            quaternion(T w, T x, T y, T z): w_(w), x_(x), y_(y), z_(z) {}
            
            quaternion(const qT& q_in) = default;
            constexpr qT& operator=(const qT& q_in) = default;
            constexpr qT& operator+=(const qT& q_in) {
                w_ += q_in.w_;
                x_ += q_in.x_;
                y_ += q_in.y_;
                z_ += q_in.z_;

                return *this;
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

    template<typename T>
    class quaternionU : public quaternion<T>{
        public:
            quaternionU(): quaternion<T>(){}
            quaternionU(T w, T x, T y, T z): quaternion<T>(w, x, y, z) {
                this->normalise();
            }
            
            quaternionU(const quaternionU<T>& q_in): quaternion<T>(q_in) {}
            constexpr quaternionU<T>& operator=(const quaternionU<T>& q_in) = default;

            constexpr quaternionU<T>& operator+=(const quaternionU<T>& q_in){
                *this = static_cast<quaternion<T>>(*this) + static_cast<quaternion<T>>(q_in);
                
                this->normalise();
                return *this;
            }
    };


    template<   bool _activate_flag = true,
                template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    inline Base<T> normalise(Base<T> q){

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

    template<   template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto hamilton_prod(const Base<T>& q_lhv, const Base<T>& q_rhv) {

        Base<T> q_res(  q_lhv.w() * q_rhv.w() - q_lhv.x() * q_rhv.x() - q_lhv.y() * q_rhv.y() - q_lhv.z() * q_rhv.z(),
                        q_lhv.w() * q_rhv.x() + q_lhv.x() * q_rhv.w() + q_lhv.y() * q_rhv.z() - q_lhv.z() * q_rhv.y(),
                        q_lhv.w() * q_rhv.y() - q_lhv.x() * q_rhv.z() + q_lhv.y() * q_rhv.w() + q_lhv.z() * q_rhv.x(),
                        q_lhv.w() * q_rhv.z() + q_lhv.x() * q_rhv.y() - q_lhv.y() * q_rhv.x() + q_lhv.z() * q_rhv.w());

        return q_res;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto conjugate(Base<T> q) {
        q.conjugate();
        return q;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto inverse(const Base<T>& q_in) {

        if(!q_in.empty()){
            Base<T> q_conj = q_in;
            q_conj.conjugate();

            return q_conj / pow(q_in.norm(), 2);;
        }
        else{
            return Base<T>(0, 0, 0, 0);
        }
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    inline constexpr Base<T> conjugate(const Base<T>& q) {
        Base<T> q_res(q);
        q_res.conjugate();

        return q_res;
    }

    /*
        ------------------------------ Operators definition ------------------------------
    */ 

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator/(const Base<T>& q_lhv, double rhv) {

        Base<T> q_res(  q_lhv.w() / rhv,
                        q_lhv.x() / rhv,
                        q_lhv.y() / rhv,
                        q_lhv.z() / rhv);

        return normalise<false>(q_lhv);
    }


    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator-(const Base<T>& q_lhv, const Base<T>& q_rhv){

        Base<T> q_res = q_lhv;

        q_res.w_ -= q_rhv.w();
        q_res.x_ -= q_rhv.x();
        q_res.y_ -= q_rhv.y();
        q_res.z_ -= q_rhv.z();

        if constexpr (std::is_same_v<Base<T>, quaternionU<T>>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }


using quaternionf = quaternion<float>;
using quaterniond = quaternion<double>;
using quaternionUf = quaternionU<float>;
using quaternionUd = quaternionU<double>;

}

#endif