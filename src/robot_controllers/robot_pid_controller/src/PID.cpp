#include <PID.h>

PID_controller::PID_controller(){
    ROS_INFO("initialize the PID controller with default parameters");
}

PID_controller::PID_controller(double kP, double kI, double kD, 
                                double dt, double max, double min){
    PID_controller::kP = kP;
    PID_controller::kI = kI;
    PID_controller::kD = kD;
    PID_controller::dt = dt;
    PID_controller::max = max;
    PID_controller::min = min;
    ROS_INFO("initialize the PID controller with parameters");
}

void PID_controller::setParameters(double kP, double kI, double kD, double max, double min){
    PID_controller::kP = kP;
    PID_controller::kI = kI;
    PID_controller::kD = kD;
    PID_controller::max = max;
    PID_controller::min = min;

    std::cout << "Kp, Ki, Kd: " << PID_controller::kP << " "
                                << PID_controller::kI << " "
                                << PID_controller::kD << std::endl;
}

double PID_controller::PIDcalculate(double state, double reference){
    error = reference - state;

    pPart = kP*error;
    integral += error*dt;
    iPart = kI*integral;
    dPart = kD*(error - lastError)/dt;

    PIDout = pPart + iPart + dPart;
    if(PIDout >= max) {PIDout = max;}
    if(PIDout <= min) {PIDout = min;}

    lastError = error;
    return PIDout;

}