#ifndef ELEVTOL_PID_CONTROL_H
#define ELEVTOL_PID_CONTROL_H

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
} gain_constants;

typedef struct {
    double p_term;
    double i_term;
    double d_term;
} pid_terms;

class PID_Control {
    public:
        PID_Control();
        PID_Control(gain_constants* gains, float desired_value);
        ~PID_Control();
        double calculate_PID(double curr_time, double curr_value);

        void setDesiredValue(float desired_value);
        float getDesiredValue();

        void setPGain(float new_p_gain);
        float getPGain();
        void setIGain(float new_i_gain);
        float getIGain();
        void setDGain(float new_d_gain);
        float getDGain();
        
        double getPTerm();
        double getITerm();
        double getDTerm();
    private:
        float _desired_value;
        double _curr_error;
        double _prev_error;
        double _prev_time;
        double _time_elapsed;
        gain_constants* _gains;
        pid_terms* _terms;
};

#endif