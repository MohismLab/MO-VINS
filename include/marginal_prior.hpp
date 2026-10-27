// SPDX-License-Identifier: GPL-3.0-only
// Schur complement and quaternion delta conventions adapted from VINS-Fusion
// marginalization_factor.cpp. Ownership, finite checks and accumulation rewritten.
#pragma once
#include <ceres/sized_cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <set>
#include <vector>
namespace mohism {
struct FactorBlock {
 std::unique_ptr<ceres::CostFunction> cost;
 std::vector<double*> blocks;
 bool robust=false,huber=false;
 FactorBlock(ceres::CostFunction* c,std::vector<double*> b,bool r=false,bool h=false):cost(c),blocks(std::move(b)),robust(r),huber(h){}
 FactorBlock(FactorBlock&&)=default;FactorBlock& operator=(FactorBlock&&)=default;
};
class MarginalPrior final:public ceres::CostFunction {
 Eigen::MatrixXd A_;Eigen::VectorXd b_;
 std::vector<Eigen::VectorXd> origins_;std::vector<int> offsets_;
public:
 std::vector<double*> blocks;
 static std::unique_ptr<MarginalPrior> build(const std::vector<FactorBlock>&,const std::set<double*>& drop);
 bool Evaluate(double const*const* parameters,double* residuals,double** jacobians)const override;
 const Eigen::MatrixXd& squareRoot()const{return A_;}
 const Eigen::VectorXd& residualOrigin()const{return b_;}
 const std::vector<Eigen::VectorXd>& origins()const{return origins_;}
 static std::unique_ptr<MarginalPrior> restore(const Eigen::MatrixXd&,const Eigen::VectorXd&,const std::vector<double*>&,const std::vector<Eigen::VectorXd>&);
};
}
