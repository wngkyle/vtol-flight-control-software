#ifndef ELEVTOL_PID_CONTROL_H
#define ELEVTOL_PID_CONTROL_H

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
} gain_constants;

typedef struct {
    float p_term;
    float i_term;
    float d_term;
} pid_terms;

class PID_Control {
    public:
        PID_Control();
        PID_Control(gain_constants* gains, float desired_value);
        ~PID_Control();
        float calculate_PID(float curr_value);

        void setDesiredValue(float desired_value);
        float getDesiredValue();

        void setPGain(float new_p_gain);
        float getPGain();
        void setIGain(float new_i_gain);
        float getIGain();
        void setDGain(float new_d_gain);
        float getDGain();
        
        void setPTerm(float new_p_term);
        float getPTerm();
        void setITerm(float new_i_term);
        float getITerm();
        void setDTerm(float new_d_term);
        float getDTerm();

        float curr_error;
        float prev_error;
        float curr_time;
        float prev_time;
        float time_elapsed;
    private:
        float _desired_value;
        gain_constants* _gains;
        pid_terms* _terms;
};

#endif