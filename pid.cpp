#include "pid.h"

Pid::Pid() {
    this->kp = 0;
    this->ki = 0;
    this->kd = 0;
    this->integral = 0;
    this->prev_error = 0;
}

void Pid::setGain(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void Pid::setGoal(float goal) {
    this->goal = goal;
}

void Pid::update(float current) {
    this->current = current;
}

float Pid::calc() {
    this->error = this->goal - this->current;
    this->integral += this->error;
    float derivative = this->error - this->prev_error;
    this->prev_error = this->error;
    return this->kp * this->error + this->ki * this->integral + this->kd * derivative;
}
