#include <ros/ros.h>

class PID_controller {
public:
    double PIDout{0.0};
    double error{0.0}, lastError{0.0};
    PID_controller();
    PID_controller(double kP, double kI, double kD, 
                    double dt, double max, double min);
    void setParameters(double kP, double kI, double kD, double max, double min);
    double PIDcalculate(double state, double reference);
    double kP{1.2}, kI{0.8}, kD{0.00}, dt{0.02}, max{5.0}, min{-5.0};
private:
    
    double pPart{0.0}, iPart{0.0}, integral{0.0}, dPart{0.0};
    
    
};