#ifndef QUATERNION_TYPE_TRAITS_HPP
#define QUATERNION_TYPE_TRAITS_HPP

#include <quaternion.hpp>
#include <type_traits>

namespace yadq {

    template<typename _T>
    class quaternion;

    template<typename _T>
    class quaternionU;
    
    template< template<typename> class C>
    std::false_type is_base_of_template_impl();

    template< template<typename> class C, typename T>
    std::true_type is_base_of_template_impl(const C<T>*);
    
    template<typename T, template <typename> class C>
    using is_base_of_template = decltype(is_base_of_template_impl<C>(std::declval<T*>()));

    template<typename T, template <typename> class C>
    constexpr bool is_base_of_template_v = is_base_of_template<T, C>::value;

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