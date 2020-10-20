#ifndef PIDMATLAB_HPP_
#define PIDMATLAB_HPP_

class PIDController
{
  public:
    struct Param
    {
        float P;
        float I;
        float D;
        float N;
        float b;
        float c;
    };


    // struct Param param;

    // _Ts: Control loop period
    // _param: Controller parameter
    // PIDController(float _Ts, const Param &_param) : Ts(_Ts), param(_param)
    PIDController(float _Ts) : Ts(_Ts)
    {
        reset();
    }

    virtual ~PIDController() {}

    float update(float r, float y);
    // {
    //     const float u_d = ((param.c * r - y) * param.D - filtered_derivative_state) * param.N;
    //     const float u_p = (param.b * r - y) * param.P;

    //     const float u = u_d + u_p + integral_state;

    //     integral_state += Ts * (r - y) * param.I;
    //     filtered_derivative_state += Ts * u_d;

    //     return u;
    // }

    void reset();
    void set_params(float p, float i, float d, float n, float b, float c);

    bool use_int = 0;

    // {
    //     integral_state = 0.0;
    //     filtered_derivative_state = 0.0;
    // }

  private:
    const float Ts;
    Param param;

    float integral_state;
    float filtered_derivative_state;
};

#endif /* PIDCONTROLLER_H_ */