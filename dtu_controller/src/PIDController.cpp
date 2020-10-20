#include "PIDController.h"
#include <iostream>

float PIDController::update(float r, float y)
{
    // std::cout << "Error = " << (r-y) << std::endl;
    // std::cout << integral_state << " | " << filtered_derivative_state << std::endl;
    // std::cout << "((" << param.c << "*" << r-y << ")*" << param.D << "-" << filtered_derivative_state << ")*" << param.N << std::endl;
    const float u_d = ((param.c * r - y) * param.D - filtered_derivative_state) * param.N;
    const float u_p = (param.b * r - y) * param.P;

    const float u = u_d + u_p + integral_state;

    if( use_int ) integral_state += Ts * (r - y) * param.I;

    filtered_derivative_state += Ts * u_d;

    // std::cout << u_d << " | " << u_p << " | " << u << " | " << integral_state << " | " << filtered_derivative_state << std::endl;

    return u;
}

void PIDController::reset()
{
    integral_state = 0.0;
    filtered_derivative_state = 0.0;
}

void PIDController::set_params(float p, float i, float d, float n, float b, float c)
{
	param.P = p;
	param.I = i;
	param.D = d;
	param.N = n;
	param.b = b;
	param.c = c;
	reset();

}