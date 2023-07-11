#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <optional>
#include <array>

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
                return *this;
            }
    };

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

        return normalise(q_lhv);
    }


    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto operator-(const Base<T>& q_lhv, const Base<T>& q_rhv) noexcept{

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
    constexpr inline auto operator+(const Base<T>& q_lhv, const Base<T>& q_rhv) noexcept{

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
    constexpr inline auto operator+(const Base<T>& q_lhv, double rhv) noexcept{

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
    constexpr inline auto operator+(double lhv, const Base<T>& q_rhv) noexcept{
        return q_rhv + lhv;
    }  

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(double val, const Base<T>& q_rhv) noexcept{ 

        Base<T> q_res(  q_rhv.w() * val,
                        q_rhv.x() + val,
                        q_rhv.y() + val,
                        q_rhv.z() + val);

        return normalise(q_res);
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(const Base<T>& q_lhv, double val) noexcept{ 
        return val * q_lhv;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto operator*(const Base<T>& q_lhv, const Base<T>& q_rhv) noexcept {
        return hamilton_prod(q_lhv, q_rhv);
    }

    /*
        ------------------------------ Fcn definition ------------------------------
    */

    template<   template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr Base<T> normalise(const Base<T>& q_in) noexcept{

        if(auto d = q_in.norm(); d != 0.0) {

            Base<T> q_res(  q_in.w() / d, 
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

    template<   template <typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr inline auto hamilton_prod(const Base<T>& q_lhv, const Base<T>& q_rhv) noexcept{

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
    inline constexpr Base<T> conjugate(const Base<T>& q) noexcept{
        Base<T> q_res(q);
        q_res.conjugate();

        return q_res;
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    inline constexpr Base<T> exp(const Base<T>& q_in) noexcept{
        Base<T> q_conj(q_in);
        q_conj.conjugate();

        auto u = (q_in - q_conj) / 2.0;
        auto u_norm = u.norm();

        return std::exp(q_in.w()) + (std::cos(u_norm) + (u / u_norm) * std::sin(u_norm));;
        
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr std::optional<Base<T>> acos(const Base<T>& q_in) noexcept {
        
        if constexpr (q_in.empty()){
            return std::nullopt;
        }

        return std::acos(q_in.w() / q_in.norm());
    }

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr std::optional<Base<T>>log(const Base<T>& q_in)  noexcept{
        
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
    constexpr Base<T> dot(const Base<T>& q_lhv, const Base<T>& q_rhv) noexcept{
        return q_lhv.x() * q_rhv.x() + q_lhv.y() * q_rhv.y() + q_lhv.z() * q_rhv.z();
    }


    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr Base<T> interpolation(const Base<T>& q_start, const Base<T>& q_end, double t, InterpType interp_type = InterpType::LERP) noexcept{
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

    template<   template<typename> class Base, 
                typename T, 
                typename =  std::enable_if_t<std::is_base_of_v<quaternion<T>, Base<T>>>>
    constexpr auto quatToRotation(const Base<T>& q) noexcept{

        auto a0 = pow(q.w(), 2);
        auto a1 = pow(q.x(), 2);
        auto a2 = pow(q.y(), 2);
        auto a3 = pow(q.z(), 2);

        auto a4 = q.w() * q.x();
        auto a5 = q.w() * q.y();
        auto a6 = q.w() * q.z();

        auto a7 = q.x() * q.y();
        auto a8 = q.x() * q.z();
        
        auto a9 = q.y() * q.z();

        std::array<T, 9> R = {  2 * (a0 + a1) - 1, 2 * (a7 - a6), 2 * (a8 + a5),
                                2 * (a7 + a6), 2 * (a0 + a2) - 1, 2 * (a9 - a4),
                                2 * (a8 - a5), 2 * (a9 + a4), 2 * (a0 + a3) - 1};

        return R;
    }



    using quaternionf = quaternion<float>;
    using quaterniond = quaternion<double>;
    using quaternionUf = quaternionU<float>;
    using quaternionUd = quaternionU<double>;

}

#endif