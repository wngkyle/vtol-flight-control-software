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
    curr_error = 0;
    prev_error = 0;
    prev_time = 0;
    time_elapsed = 0;
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
    curr_error = 0;
    prev_error = 0;
    prev_time = 0;
    time_elapsed = 0;
}

PID_Control::~PID_Control() {
    if (_gains) {
        delete _gains;
    }
    if (_terms) {
        delete _terms;
    }
}

float PID_Control::calculate_PID(float measurement) {
    // P Term
    curr_error = _desired_value - measurement;
    _terms->p_term = _gains->p_gain * curr_error;

    // I Term
    if (-3 < curr_error && curr_error < 3) {
        float curr_error_sum = (prev_error + curr_error) * time_elapsed / 2;
        curr_error_sum *= _gains->i_gain;
        _terms->i_term += curr_error_sum;
    }

    // D Term
    float error_diff = curr_error - prev_error;
    _terms->d_term = _gains->d_gain * (error_diff / time_elapsed);

    prev_error = curr_error;

    return _terms->p_term + _terms->i_term + _terms->d_term;
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

float PID_Control::getPTerm() {
    return _terms->p_term;
}

float PID_Control::getITerm() {
    return _terms->i_term;
}

float PID_Control::getDTerm() {
    return _terms->d_term;
}

void PID_Control::setPTerm(float new_p_term) {
    _terms->p_term = new_p_term;
}

void PID_Control::setITerm(float new_i_term) {
    _terms->i_term = new_i_term;
}

void PID_Control::setDTerm(float new_d_term) {
    _terms->d_term = new_d_term;
}
