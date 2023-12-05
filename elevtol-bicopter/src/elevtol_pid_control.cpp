#include <elevtol_pid_control.h>

PID_Control::PID_Control() {
    _gains = new gain_constants;
    _gains->p_gain = 0;
    _gains->i_gain = 0;
    _gains->d_gain = 0;

    _terms = new pid_terms;
    _terms->p_term = 0;
    _terms->i_term = 0;
    _terms->d_term = 0;

    _desired_value = 0;
    _curr_error = 0;
    _prev_error = 0;
    _prev_time = 0;
    _time_elapsed = 0;
}

PID_Control::PID_Control(gain_constants* gains, float desired_value) {
    _gains = new gain_constants;
    _gains->p_gain = gains->p_gain;
    _gains->i_gain = gains->i_gain;
    _gains->d_gain = gains->d_gain;

    _terms = new pid_terms;
    _terms->p_term = 0;
    _terms->i_term = 0;
    _terms->d_term = 0;

    _desired_value = desired_value;
    _curr_error = 0;
    _prev_error = 0;
    _prev_time = 0;
    _time_elapsed = 0;
}

PID_Control::~PID_Control() {
    if (_gains) {
        delete _gains;
    }
    if (_terms) {
        delete _terms;
    }
}

double PID_Control::calculate_PID(double time, double measurement) {
    _time_elapsed = time - _prev_time;

    // P Term
    _curr_error = _desired_value - measurement;
    _terms->p_term = _gains->p_gain * _curr_error;

    // I Term
    double curr_error_sum = (_prev_error + _curr_error) * _time_elapsed / 2;
    curr_error_sum *= _gains->i_gain;
    _terms->i_term += curr_error_sum;

    // D Term
    double error_diff = _curr_error - _prev_error;
    _terms->d_term = _gains->d_gain * error_diff;

    _prev_time = time;
    _prev_error = _curr_error;
}

void PID_Control::setDesiredValue(float desired_value) {
    _desired_value = desired_value;
}

float PID_Control::getDesiredValue() {
    return _desired_value;
}

void PID_Control::setPGain(float new_p_gain) {
    _gains->p_gain = new_p_gain;
}

float PID_Control::getPGain() {
    return _gains->p_gain;
}

void PID_Control::setIGain(float new_i_gain) {
    _gains->i_gain = new_i_gain;
}

float PID_Control::getIGain() {
    return _gains->i_gain;
}

void PID_Control::setDGain(float new_d_gain) {
    _gains->d_gain = new_d_gain;
}

float PID_Control::getDGain() {
    return _gains->d_gain;
}

double PID_Control::getPTerm() {
    return _terms->p_term;
}

double PID_Control::getITerm() {
    return _terms->i_term;
}

double PID_Control::getDTerm() {
    return _terms->d_term;
}
