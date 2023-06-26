#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>

namespace yadq{

    enum class InterpType {SLERP, LERP};

    template<typename _T>
    class quaternion{
        static_assert(std::is_same_v<_T, float> || std::is_same_v<_T, double>, "This class only supports floating point types");
        private:
            using qT = quaternion<_T>;    

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

                return *this;
            }
            constexpr qT& operator/=(const _T rhv) {
                w_ /= rhv;
                x_ /= rhv;
                y_ /= rhv;
                z_ /= rhv;

                return *this;
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
    constexpr Base<T> normalise(const Base<T>& q_in);

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

        Base<T> q_res(  q_lhv.w() - q_rhv.w(),
                        q_lhv.x() - q_rhv.x(),
                        q_lhv.y() - q_rhv.y(),
                        q_lhv.z() - q_rhv.z());

        if constexpr (std::is_same_v<Base<T>, quaternionU<T>>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator+(const Base<T>& q_lhv, const Base<T>& q_rhv){

        Base<T> q_res(  q_lhv.w() + q_rhv.w(),
                        q_lhv.x() + q_rhv.x(),
                        q_lhv.y() + q_rhv.y(),
                        q_lhv.z() + q_rhv.z());

        if constexpr (std::is_same_v<Base<T>, quaternionU<T>>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }    

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator+(const Base<T>& q_lhv, double rhv){

        Base<T> q_res(  q_lhv.w() + rhv,
                        q_lhv.x() + rhv,
                        q_lhv.y() + rhv,
                        q_lhv.z() + rhv);

        if constexpr (std::is_same_v<Base<T>, quaternionU<T>>){
            return normalise(q_res);
        }else{
            return q_res;
        }
    }  

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator+(double lhv, const Base<T>& q_rhv){
        return q_rhv + lhv;
    }  

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(double val, const Base<T>& q_rhv){ 

        Base<T> q_res(  q_rhv.w() * val,
                        q_rhv.x() + val,
                        q_rhv.y() + val,
                        q_rhv.z() + val);

        return normalise(q_res);
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(const Base<T>& q_lhv, double val){ 
        return val * q_lhv;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(const Base<T>& q_lhv, const Base<T>& q_rhv) {
        return hamilton_prod(q_lhv, q_rhv);
    }

    /*
        ------------------------------ Fcn definition ------------------------------
    */

    template<   bool _activate_flag = true,
                template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr Base<T> normalise(const Base<T>& q_in){

        if constexpr (_activate_flag){
            
            if(auto d = q_in.norm(); d != 0.0) {

                Base<T> q_res(  q_in.w() / d, 
                                q_in.x() / d, 
                                q_in.y() / d, 
                                q_in.z() / d);
                return q_res;
            }
        }

        return q_in;

    }

    template<typename T>
    std::ostream& operator<<(std::ostream &os, const quaternion<T>& q_in) { 
        return os << "w: " << q_in.w() << " x: " << q_in.x() << " y: " << q_in.y() << " z: " << q_in.z();
    }

    template<   template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto mod(const Base<T>& q_in){
        
        Base<T> q_res(  q_in.w() / fabs(q_in.w()),
                        q_in.x() / fabs(q_in.x()),
                        q_in.y() / fabs(q_in.y()),
                        q_in.z() / fabs(q_in.z()));
        
        return q_res;
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

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    inline constexpr Base<T> exp(const Base<T>& q_in) {
        Base<T> q_conj(q_in);
        q_conj.conjugate();

        auto u = (q_in - q_conj) / 2.0;
        auto u_norm = u.norm();

        return std::exp(q_in.w()) + (std::cos(u_norm) + (u / u_norm) * std::sin(u_norm));;
        
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr std::optional<Base<T>> acos(const Base<T>& q_in) {
        
        if constexpr (q_in.empty()){
            return std::nullopt;
        }

        return std::acos(q_in.w() / q_in.norm());
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr std::optional<Base<T>>log(const Base<T>& q_in) {
        
        Base<T> q_conj(q_in);
        q_conj.conjugate();

        auto u = (q_in - q_conj) / 2.0;
        auto u_norm = u.norm();

        auto angle = acos(q_in);

        if (angle == std::nullopt){
            return std::nullopt;
        }

        return std::log(u_norm) + (u / u_norm) * angle;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr Base<T> dot(const Base<T>& q_lhv, const Base<T>& q_rhv){
        return q_lhv.x() * q_rhv.x() + q_lhv.y() * q_rhv.y() + q_lhv.z() * q_rhv.z();
    }


    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr Base<T> interpolation(const Base<T>& q_start, const Base<T>& q_end, double t, InterpType interp_type = InterpType::LERP){
        switch (interp_type)
        {
        case InterpType::LERP:
            return q_start * (1.0 - t) + q_end * t;
            break;
        case InterpType::SLERP:
            auto omega = dot(q_start, q_end);
            return ((std::sin(1 - t) * omega) / std::sin(omega)) * q_start + (sin(t * omega) / sin(omega)) * q_end;
            break;
        default:
            return q_start;
        }
    }


    using quaternionf = quaternion<float>;
    using quaterniond = quaternion<double>;
    using quaternionUf = quaternionU<float>;
    using quaternionUd = quaternionU<double>;

}

#endif