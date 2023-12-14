/**
* DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
*
* This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
* Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
* the Under Secretary of Defense for Research and Engineering.
*
* © 2023 Massachusetts Institute of Technology.
*
* The software/firmware is provided to you on an As-Is basis
*
* Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
* Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
* Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
*/


#ifndef CBF_CLASSES_HEADER
#define CBF_CLASSES_HEADER

#include "State_Container.h"
// #include "OsqpEigen/OsqpEigen.h"
#include <math.h>

#include <eigen3/Eigen/Dense>

// DEBUG
#include <iostream>
using std::cerr, std::cout, std::endl;
// END DEBUG

class CBF_Parent {
   public:
    CBF_Parent() {};

    // functions
    //  returns a,b as in a^T u <= b
    virtual std::pair<Eigen::RowVectorXd, double> get_constraint(std::array<double, 2> u2) const = 0;
    virtual std::pair<Eigen::RowVectorXd, double> get_constraint() const = 0;

    // DEBUG
    virtual Eigen::Matrix2d T(double theta) const = 0;
    // END DEBUG

//    virtual std::pair<Eigen::RowVectorXd, double> get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2, std::array<double, 2> u2) const = 0;
//    virtual std::pair<Eigen::RowVectorXd, double> get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2) const = 0;

    virtual double h() const = 0;

   private:
    // Should this go in the child classes? This might be implementation
    // specific. For example, some CBFs may consider agents + obstacles, other
    // implementations will consider agents + agents

    // std::unordered_map<std::string, std::array<double,4>> *AgentStateInfo; //
    // Pointer to std::map with agents' state information
    // std::vector<std::string> ParticipatingAgents;
};



#endif