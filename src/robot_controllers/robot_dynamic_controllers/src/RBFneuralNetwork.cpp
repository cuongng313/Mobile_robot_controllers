#include <ros/ros.h>
#include "RBFneuralNetwork.h"

RBFNN::RBFNN(int sizeOfVector, int number_neural, 
                            double centerRange,
                            double rbfwidth) {

    this->i_size_ = sizeOfVector;
    this->n_ = number_neural;
    this->rbfcenter_.resize(sizeOfVector, number_neural);
    this->rbfwidth_.resize(number_neural);
    this->output_.resize(number_neural);
    this->input_.resize(sizeOfVector);

    // insert value for center and width of the RBF
    Eigen::RowVectorXf tempRow(number_neural);  tempRow.setZero();
    double pointSpace = (2*centerRange)/(number_neural-1);

    for (int i = 0; i < number_neural; i++) {
        if (i == 0) { tempRow(0) = -centerRange;}
        else { tempRow(i) = tempRow(i-1) + pointSpace;}
        this->rbfwidth_(i) = rbfwidth;
    }

    for (int i = 0; i < sizeOfVector; i++) {
        this->rbfcenter_.row(i) = tempRow;
        if ( i == 2) {this->rbfcenter_.row(i) = 0.1*tempRow; }    // fix for the small value of radian
    }

    // std::cout << "size of center: " << this->rbfcenter_.rows() << "x"
    //                                 << this->rbfcenter_.cols()
    //                                 << std::endl;
    std::cout << "******* RBFNN structure: " << std::endl;
    std::cout << " + number neural: " << this->n_ << std::endl;
    std::cout << " + size of input : " << this->i_size_ << std::endl;
    std::cout << " + center: " << std::endl << this->rbfcenter_ << std::endl;
    std::cout << " + width: " << std::endl << this->rbfwidth_ << std::endl;
    std::cout << "***************************** " << std::endl << std::endl;
}

void RBFNN::calculate(Eigen::VectorXf input) {
    if(input.size() != this->input_.size()) {
        std::cout << "not the same size of vector" << std::endl;
    }
    else {
        for (int i = 0; i < this->n_; i++) {
           Eigen::VectorXf dif(this->i_size_);      dif.setZero();
           dif = input - this->rbfcenter_.col(i);
           double b_i = this->rbfwidth_(i);
           this->output_(i) = exp(-(dif.squaredNorm()*dif.squaredNorm())/(2*b_i*b_i));  
        //    this->output_(i) = -(dif.squaredNorm()*dif.squaredNorm())/(2*b_i*b_i);  
        }
    }
}