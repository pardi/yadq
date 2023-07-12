#ifndef QUATERNION_TYPE_TRAITS_HPP
#define QUATERNION_TYPE_TRAITS_HPP

#include <quaternion.hpp>
#include <type_traits>

namespace yadq {

    template<typename _T>
    class quaternion;

    template<typename _T>
    class quaternionU;

    template<typename>
    struct is_base_of_quaternion : std::false_type {};

    template<   template<typename...> class Base, 
                typename... Ts>
    struct is_base_of_quaternion<Base<Ts...>> : std::is_base_of<quaternion<Ts...>, Base<Ts...>> {}; 

    template<typename T>
    constexpr bool is_base_of_quaternion_v = is_base_of_quaternion<T>::value;

    template<typename>
    struct is_quaternion : std::false_type {};

    template<typename T>
    struct is_quaternion<quaternion<T>> : std::true_type {};

    template<typename T>
    constexpr bool is_quaternion_v = is_quaternion<T>::value;

    template<typename>
    struct is_quaternionU : std::false_type {};

    template<typename T>
    struct is_quaternionU<quaternionU<T>> : std::true_type {};
    
    template<typename T>
    constexpr bool is_quaternionU_v = is_quaternionU<T>::value;
}


#endif