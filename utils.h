template< class C, typename U = typename C::value_type>
inline U norm(const C& container) noexcept{
    
    U res = 0;

    for (auto& e: container){
        res += std::pow(e, 2);
    }

    return std::sqrt(res);
}