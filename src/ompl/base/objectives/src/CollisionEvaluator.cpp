/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Bryce Willey */

#include "ompl/base/objectives/CollisionEvaluator.h"
#include "ompl/trajopt/sco_common.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/expr_vec_ops.h"
#include "ompl/trajopt/utils.h"
#include "ompl/trajopt/eigen_conversions.h"

ompl::base::JacobianDiscreteCollisionEvaluator::JacobianDiscreteCollisionEvaluator(
        WorkspaceCollisionFn inCollision, StateSpacePtr ss, JacobianFn J, sco::VarVector vars) :
    inCollision_(inCollision), ss_(ss), J_(J), vars_(vars)
{}

std::vector<sco::AffExpr> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistanceExpressions(std::vector<double> x) {
    std::vector<double> x_0 = sco::getDblVec(x, vars_);
    std::vector<sco::AffExpr> exprs;

    // Fields to be filled in by inCollision.
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;
    if (inCollision_(x_0, signedDists, points, link_names, normals)) {
        // For each collision, make the affine expression,
        // sd(vars) = sd(x_0) + normal^T * J_pa(x_0) * vars - normal^T * J_pa(x_0) * x
        for (unsigned int i = 0; i < signedDists.size(); i++) {
            double signedDist = signedDists[i];
            Eigen::Vector3d normal = normals[i];
            Eigen::Vector3d point = points[i];
            printf("Collided point: %f, %f, %f\n", point[0], point[1], point[2]);
            std::string link_name = link_names[i];

            printf("A collision!: dist: %f\n\tlink_name: %s\n\tnormal: %f, %f, %f\n",
                   signedDist, link_name.c_str(), normal[0], normal[1], normal[2]);
            sco::AffExpr dist(signedDist);
            printf("Made an affine expression.\n");
            Eigen::MatrixXd j = J_(x_0, point, link_name);
            Eigen::VectorXd dist_gradient = normal.transpose() * j;
            sco::exprInc(dist, sco::varDot(dist_gradient, vars_));
            sco::exprInc(dist, -dist_gradient.dot(sco::toVectorXd(x_0)));
            exprs.push_back(dist);
            std::cout << "dist expression: " << dist << std::endl;
        }
    }
    return exprs;
}

std::vector<double> ompl::base::JacobianDiscreteCollisionEvaluator::calcDistances(std::vector<double> x) {
    std::vector<double> dofVals = sco::getDblVec(x, vars_);
    std::vector<double> distsAtSteps;

    // TODO: We only need the clearance, so maybe pass in another callback that just gives that.
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;
    bool collision = inCollision_(dofVals, signedDists, points, link_names, normals);
    if (collision) {
        std::cout << "dofVals: ";
        for (unsigned int i = 0; i < dofVals.size(); i++) {
            std::cout << dofVals[i] << ", ";
        }
        std::cout << std::endl;
        printf("dists: ");
        for (unsigned int i = 0; i < signedDists.size(); i++) {
            printf("%f, ", signedDists[i]);
        }
        printf("\n\tlink_name: %s\n\tnormal: %f, %f, %f",
            link_names[0].c_str(), normals[0][0], normals[0][1], normals[0][2]);
        distsAtSteps.insert(distsAtSteps.begin(), signedDists.begin(), signedDists.end());
    }
    printf("\n");
    return distsAtSteps;
}

sco::VarVector ompl::base::JacobianDiscreteCollisionEvaluator::getVars() {
    return vars_;
}

ompl::base::JacobianContinuousCollisionEvaluator::JacobianContinuousCollisionEvaluator(
        WorkspaceContinuousCollisionFn inCollision, StateSpacePtr ss, JacobianFn J, sco::VarVector vars0, sco::VarVector vars1) :
    inCollision_(inCollision), ss_(ss), J_(J), vars0_(vars0), vars1_(vars1)
{}

std::vector<sco::AffExpr> ompl::base::JacobianContinuousCollisionEvaluator::calcDistanceExpressions(std::vector<double> x) {
    std::vector<double> x_0 = sco::getDblVec(x, vars0_); // configuration at first time
    std::vector<double> x_1 = sco::getDblVec(x, vars1_); // configuration at the second time
    std::vector<sco::AffExpr> exprs;

    // Fields to be filled in by inCollision.
    std::vector<Eigen::Vector3d> points_swept;
    std::vector<Eigen::Vector3d> points0;
    std::vector<Eigen::Vector3d> points1;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;
    if (inCollision_(x_0, x_1, signedDists, points_swept, points0, points1, link_names, normals)) {
        for (unsigned int i = 0; i < signedDists.size(); i++) {
            // sd(var0, var1) = sd(x_0, x_1) + alpha * normal^T * J_p0(x_0) * (vars0 - x_0) ...
            //                  + (1 - alpha) * normal^T * J_p1(x_1) * (vars1 - x_1)
            double signedDist = signedDists[i];
            printf("SignedDist %d: %f\n", i, signedDist);
            Eigen::Vector3d normal = normals[i];
            Eigen::Vector3d point_swept = points_swept[i];
            Eigen::Vector3d point0 = points0[i];
            Eigen::Vector3d point1 = points1[i];
            std::string link_name = link_names[i];
            sco::AffExpr dist(signedDist);
            Eigen::MatrixXd j0 = J_(x_0, point0, link_name);
            Eigen::MatrixXd j1 = J_(x_1, point1, link_name);
            double p1ps = (point1 - point_swept).norm();
            double alpha = p1ps / (p1ps + (point0 -point_swept).norm());

            Eigen::VectorXd dist_gradient0 = alpha * normal.transpose() * j0;
            Eigen::VectorXd dist_gradient1 = (1 - alpha) * normal.transpose() * j1;
            sco::exprInc(dist, sco::varDot(dist_gradient0, vars0_));
            sco::exprInc(dist, -dist_gradient0.dot(sco::toVectorXd(x_0)));
            sco::exprInc(dist, sco::varDot(dist_gradient1, vars1_));
            sco::exprInc(dist, -dist_gradient1.dot(sco::toVectorXd(x_1)));
            exprs.push_back(dist);
        }
    }
    return exprs;
}

std::vector<double> ompl::base::JacobianContinuousCollisionEvaluator::calcDistances(std::vector<double> x) {
    std::vector<double> dof0 = sco::getDblVec(x, vars0_);
    std::vector<double> dof1 = sco::getDblVec(x, vars1_);
    std::vector<double> distsAtSteps;

    std::vector<Eigen::Vector3d> points_swept;
    std::vector<Eigen::Vector3d> points0;
    std::vector<Eigen::Vector3d> points1;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> signedDists;
    std::vector<std::string> link_names;

    bool collision = inCollision_(dof0, dof1, signedDists, points_swept, points0, points1, link_names, normals);
    if (collision) {
        for (unsigned int i = 0; i < signedDists.size(); i++) {
            printf("SignedDist (calc) %d: %f\n", i, signedDists[i]);
        }
        distsAtSteps.insert(distsAtSteps.begin(), signedDists.begin(), signedDists.end());
    }
    return distsAtSteps;
}

sco::VarVector ompl::base::JacobianContinuousCollisionEvaluator::getVars() {
    // What a waste of memory, find a better solution for this.
    std::vector<sco::Var> allVars;
    allVars.insert(allVars.begin(), vars1_.begin(), vars1_.begin());
    allVars.insert(allVars.begin(), vars0_.begin(), vars0_.end());
    return allVars;
}
