#include "quaternion.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <array>
#include <deque>
#include "utils.h"


// template<typename T>
// quaternion<T> quaternion<T>::operator+(quaternion<T>&& q_in){

//     // quaternion<T> q_tmp(std::forward<quaternion<T>>(q_in));
//     quaternion<T> q_tmp(std::move(q_in));

//     q_tmp.w_ += w_;
//     q_tmp.x_ += x_;
//     q_tmp.y_ += y_;
//     q_tmp.z_ += z_;
    
//     normalise();

//     return q_tmp;
// }

int main(){

    int w = 34;

    quaternion<double> q(w, 1, 2, 3);
    quaternion<double> q1 = q;

    int N = 100000;
    auto start = std::chrono::steady_clock::now();    
    for (int i = 0; i < N; ++i){
        quaternion<double> q2(q);
        if (i == N - 1){
            std::cout << "#1 res: " <<  q2.w_ << std::endl;
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "#1 l-value copy-constr [ticks]: " << elapsed_seconds.count()/ N * 1e15 << "\n\n";

    start = std::chrono::steady_clock::now();    
    for (int i = 0; i < N; ++i){
        quaternion<double> q2(std::move(q));

        if (i == N - 1){
            std::cout << "#2 res: " << q2.w_ << std::endl;
        }
    }
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    std::cout << "#2 r-value move-constr [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";

    start = std::chrono::steady_clock::now();    
    for (int i = 0; i < N; ++i){
        quaternion<double> q2(quaternion<double>(w, 1, 2, 3));

        if (i == N - 1){
            std::cout << "#3 res: " << q2.w_ << std::endl;
        }
    }
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    std::cout << "#3 r-value? perfect_forwarding move-constr [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";

    start = std::chrono::steady_clock::now();    
    quaternion<double> q_tmp(1, 0, 0, 0);
    for (int i = 0; i < N; ++i){
        q = q + q_tmp;

        if (i == N - 1){
            std::cout << "#4 res: " <<  q.w_ << std::endl;
        }
    }
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    std::cout << "#4 l-value op+ [ticks]: " << elapsed_seconds.count() / N * 1e10 << "\n\n";

    q =  quaternion<double>(w, 1, 2, 3);

    start = std::chrono::steady_clock::now();    
    for (int i = 0; i < N; ++i){
        q = q + quaternion<double>(1, 0, 0, 0);

        if (i == N - 1){
            std::cout << "#5 res: " << q.w_ << std::endl;
        }
    }
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end - start;
    std::cout << "#5 r-value op+ [ticks]: " << elapsed_seconds.count() / N * 1e10 << "\n\n";

    std::cout << "------------------------NORM--------------------\n\n";

    {
        std::array<double, 2> v = {4, 3};
        std::cout << "#1 array<double, 2> : " << norm(v) << std::endl;
    }

    {
        std::array<float, 2> v = {4.1, 3.1};
        std::cout << "#2 array<float, 2> : " << norm(v) << std::endl;
    }

    {
        std::vector<float> v = {4.1, 3.1};
        std::cout << "#3 vector<float> : " << norm(v) << std::endl;
    }

    {
        std::vector<double> v = {4.1, 3.1};
        std::cout << "#4 vector<double> : " << norm(v) << std::endl;
    }

    {
        std::deque<double> v = {4.1, 3.1};
        std::cout << "#5 deque<double> : " << norm(v) << std::endl;
    }
    
    {
        std::vector<double> v = {4.1, 3.1};

        start = std::chrono::steady_clock::now();    
        for (int i = 0; i < N; ++i){
            norm(v);
        }
        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#6 norm [ticks]: " << elapsed_seconds.count() / N * 1e10 << "\n\n";
    }

    std::cout << "------------------------[+=]--------------------\n\n";

        
    {
        quaternion<double> q_tmp;
        quaternion<double> q_add;
        
        start = std::chrono::steady_clock::now();    
        
        for (int i = 0; i < N; ++i){
            q_tmp += q_add;
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#1 += (lvalue) [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";
    }

    {
        quaternion<double> q_tmp;
        
        start = std::chrono::steady_clock::now();    
        
        for (int i = 0; i < N; ++i){
            q_tmp += quaternion<double>();
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#2 += (rvalue) [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";
    }

    std::cout << "------------------------[*]--------------------\n\n";

    {
        quaternion<double> q_tmp(2, -1, 3, 1);
        quaternion<double> q_tmp1(5, -4, 0, 1);
        
        start = std::chrono::steady_clock::now();    
        quaternion<double> q;
        for (int i = 0; i < N; ++i){
            q = q_tmp * q_tmp1;
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#1 * (lvalue) [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";
        
        std::cout << "res: " << q.w() << " " << q.x() << " " << q.y() << " "<< q.z() << std::endl;
    }

    std::cout << "------------------------[*=]--------------------\n\n";

    {
        quaternion<double> q_tmp(2, -1, 3, 1);
        quaternion<double> q_tmp1(5, -4, 0, 1);
        
        start = std::chrono::steady_clock::now();    
        quaternion<double> q;
        for (int i = 0; i < N; ++i){
            q *= q_tmp1;
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#1 *= (lvalue) [ticks]: " << elapsed_seconds.count() / N * 1e15 << "\n\n";
        
    }

    std::cout << "------------------------Inverse--------------------\n\n";

    {
        quaternion<double> q_tmp(2, -1, 3, 1);
        
        start = std::chrono::steady_clock::now();    
        for (int i = 0; i < N; ++i){
            q_tmp.inverse();
        }

        end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;
        std::cout << "#1 inverse [ticks]: " << elapsed_seconds.count() / N * 1e10 << "\n\n";
        std::cout << "res: " << q_tmp.inverse() << "\n\n";
        
    }

    std::cout << "------------------------ref&--------------------\n\n";
    
    {
        quaternion<double> q_tmp(2, -1, 3, 1);
        quaternion<double> q_tmp1(2, -1, 3, 1);
        
        start = std::chrono::steady_clock::now();    

        q_tmp += q_tmp;
        
    }

    return 0;
}