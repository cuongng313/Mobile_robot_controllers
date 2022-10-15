#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>  

class RBFNN {
    public: 
        long int n_{1};             // number neural network
        int i_size_{1};                  // size of esimated vector
        Eigen::MatrixXf rbfcenter_; 
        Eigen::RowVectorXf rbfwidth_;
        Eigen::VectorXf input_;
        Eigen::VectorXf output_;
        RBFNN(int sizeOfVector, int number_neural,
                    double centerRange,
                    double rbfwidth);
        void calculate(Eigen::VectorXf input);
        

};