// SPDX-License-Identifier: GPL-3.0-only
#include "marginal_prior.hpp"
#include <map>
#include <stdexcept>
namespace mohism {
static int local(int n){return n==7?6:n;}
std::unique_ptr<MarginalPrior> MarginalPrior::build(const std::vector<FactorBlock>& factors,const std::set<double*>& drop){
 std::map<double*,int,std::less<double*>> sizes,indices;
 for(const auto& f:factors)for(size_t i=0;i<f.blocks.size();++i){int size=f.cost->parameter_block_sizes()[i];auto inserted=sizes.emplace(f.blocks[i],size);if(!inserted.second&&inserted.first->second!=size)throw std::runtime_error("prior block size mismatch");}
 int m=0,n=0;for(auto [p,size]:sizes)if(drop.count(p)){indices[p]=m;m+=local(size);}
 n=m;for(auto [p,size]:sizes)if(!drop.count(p)){indices[p]=n;n+=local(size);}
 if(m==0||n==m)throw std::runtime_error("empty marginalization partition");
 Eigen::MatrixXd H=Eigen::MatrixXd::Zero(n,n);Eigen::VectorXd g=Eigen::VectorXd::Zero(n);
 for(const auto& f:factors){
  int rows=f.cost->num_residuals();Eigen::VectorXd r(rows);
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> jac;
  std::vector<double*> raw;for(int size:f.cost->parameter_block_sizes()){jac.emplace_back(rows,size);raw.push_back(jac.back().data());}
  if(!f.cost->Evaluate(f.blocks.data(),r.data(),raw.data())||!r.allFinite())throw std::runtime_error("invalid marginal residual");
  double squared=r.squaredNorm();double weight=f.robust?(f.huber?(squared>1?std::pow(squared,-.25):1):1/std::sqrt(1+squared)):1;
  r*=weight;
  for(auto& J:jac){if(!J.allFinite())throw std::runtime_error("invalid marginal Jacobian");J*=weight;}
  for(size_t i=0;i<jac.size();++i){int ii=indices.at(f.blocks[i]),si=local(sizes.at(f.blocks[i]));auto Ji=jac[i].leftCols(si);g.segment(ii,si)+=Ji.transpose()*r;
   for(size_t j=0;j<jac.size();++j){int jj=indices.at(f.blocks[j]),sj=local(sizes.at(f.blocks[j]));H.block(ii,jj,si,sj)+=Ji.transpose()*jac[j].leftCols(sj);}
  }
 }
 H=(H+H.transpose()).eval()*.5;
 Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eliminated(H.topLeftCorner(m,m));
 if(eliminated.info()!=Eigen::Success)throw std::runtime_error("marginal eigensolve failed");
 const double eps=1e-8;auto values=eliminated.eigenvalues();
 Eigen::VectorXd inv=values.unaryExpr([&](double x){return x>eps?1/x:0;});
 Eigen::MatrixXd inverse=eliminated.eigenvectors()*inv.asDiagonal()*eliminated.eigenvectors().transpose();
 Eigen::MatrixXd cross=H.bottomLeftCorner(n-m,m);
 Eigen::MatrixXd reduced=H.bottomRightCorner(n-m,n-m)-cross*inverse*cross.transpose();
 Eigen::VectorXd reduced_g=g.tail(n-m)-cross*inverse*g.head(m);
 reduced=(reduced+reduced.transpose()).eval()*.5;
 Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(reduced);
 if(eig.info()!=Eigen::Success||!eig.eigenvalues().allFinite()||!reduced_g.allFinite())throw std::runtime_error("invalid marginal prior");
 auto prior=std::make_unique<MarginalPrior>();Eigen::VectorXd roots=eig.eigenvalues().unaryExpr([&](double x){return x>eps?std::sqrt(x):0;});
 Eigen::VectorXd inverse_roots=roots.unaryExpr([](double x){return x>0?1/x:0;});
 prior->A_=roots.asDiagonal()*eig.eigenvectors().transpose();prior->b_=inverse_roots.asDiagonal()*eig.eigenvectors().transpose()*reduced_g;
 for(auto [p,size]:sizes)if(!drop.count(p)){prior->blocks.push_back(p);prior->origins_.emplace_back(Eigen::Map<const Eigen::VectorXd>(p,size));prior->offsets_.push_back(indices.at(p)-m);prior->mutable_parameter_block_sizes()->push_back(size);}
 prior->set_num_residuals(n-m);return prior;
}
std::unique_ptr<MarginalPrior> MarginalPrior::restore(const Eigen::MatrixXd& A,const Eigen::VectorXd& b,const std::vector<double*>& blocks,const std::vector<Eigen::VectorXd>& origins){
 if(blocks.size()!=origins.size()||blocks.empty()||!A.allFinite()||!b.allFinite())throw std::runtime_error("invalid saved prior");
 auto prior=std::make_unique<MarginalPrior>();int offset=0;std::set<double*> seen;
 for(size_t i=0;i<blocks.size();++i){int size=int(origins[i].size());if(size<1||size>9||!blocks[i]||!seen.insert(blocks[i]).second||!origins[i].allFinite())throw std::runtime_error("invalid saved prior block");prior->offsets_.push_back(offset);offset+=local(size);prior->mutable_parameter_block_sizes()->push_back(size);}
 if(A.cols()!=offset||A.rows()!=offset||b.size()!=offset)throw std::runtime_error("saved prior dimensions disagree");
 prior->A_=A;prior->b_=b;prior->blocks=blocks;prior->origins_=origins;prior->set_num_residuals(offset);return prior;
}
bool MarginalPrior::Evaluate(double const*const* p,double* residuals,double** jacobians)const{
 Eigen::VectorXd delta=Eigen::VectorXd::Zero(A_.cols());
 for(size_t i=0;i<blocks.size();++i){int size=parameter_block_sizes()[i],offset=offsets_[i];auto x=Eigen::Map<const Eigen::VectorXd>(p[i],size);if(!x.allFinite())return false;
  if(size==7){delta.segment<3>(offset)=x.head<3>()-origins_[i].head<3>();Eigen::Quaterniond q=Eigen::Map<const Eigen::Quaterniond>(origins_[i].data()+3).inverse()*Eigen::Map<const Eigen::Quaterniond>(p[i]+3);delta.segment<3>(offset+3)=2*(q.w()<0?-1:1)*q.vec();}
  else delta.segment(offset,size)=x-origins_[i];
  if(jacobians&&jacobians[i]){Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> J(jacobians[i],num_residuals(),size);J.setZero();J.leftCols(local(size))=A_.middleCols(offset,local(size));}
 }
 Eigen::Map<Eigen::VectorXd>(residuals,num_residuals())=b_+A_*delta;return true;
}
}
