#include "utils/numerics.h"

Eigen::Matrix3f hat(const Eigen::Vector3f &x)
{
    Eigen::Matrix3f x_hat;
    x_hat << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
    return x_hat;
}

Eigen::Vector3f unhat(const Eigen::Matrix3f &x_hat)
{
    Eigen::Vector3f x;
    x << x_hat(2, 1), x_hat(0, 2), x_hat(1, 0);
    return x;
}

Eigen::Matrix3f SO3(const Eigen::Vector3f &x)
{
    return Eigen::Matrix3f(hat(x).exp());
}

Eigen::Vector3f InvSO3(const Eigen::Matrix3f &x_hat)
{
    return Eigen::Vector3f(unhat((x_hat.log())));
}

Eigen::Vector3f SO3add(const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const bool &is_cyclic)
{ 
    if (is_cyclic && (x1 + x2).norm() > M_PI)
    {
        return x1 + x2;
    }
    else
    {
        return InvSO3(SO3(x1) * SO3(x2));
    }
}