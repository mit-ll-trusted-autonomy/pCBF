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

//
// Created by ja29747 on 9/20/22.
//

#ifndef IVP_EXTEND_CBF_SINGLE_INTEGRATOR_H
#define IVP_EXTEND_CBF_SINGLE_INTEGRATOR_H

#include "CBF_Classes.h"

using Eigen::VectorXd, Eigen::RowVectorXd, Eigen::Map;
using constraint = std::pair<RowVectorXd, double>;

class CBF_Single_Integrator final : public CBF_Parent {
    /*
     * Simple single integrator CBF class
     */
public:
    CBF_Single_Integrator(const StateContainer<double,2> &_stateContainer1,
                          const StateContainer<double,2> &_stateContainer2,
                          double _delta,
                          double _radius) : AgentOneState{&_stateContainer1},
                          AgentTwoState{&_stateContainer2},
                          delta{_delta},
                          R{_radius} {};

    CBF_Single_Integrator(const StateContainer<double,2>* const _stateContainer1,
                          const StateContainer<double,2>* const _stateContainer2,
                          double _delta,
                          double _radius) : AgentOneState{_stateContainer1},
                            AgentTwoState{_stateContainer2},
                            delta{_delta},
                            R{_radius} {};

    double h() const override {
        // Returns R^2 - norm([x1,y1] - [x2,y2],2)^2

        const double x1{(*AgentOneState)[0]};
        const double y1{(*AgentOneState)[1]};
        const double x2{(*AgentTwoState)[0]};
        const double y2{(*AgentTwoState)[1]};

        return R * R - ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    constraint get_constraint(std::array<double,2> u2) const override {
        Eigen::RowVectorXd a = get_avector();
        double b = get_bscalar(u2);

        return constraint{a, b};
    }

    constraint get_constraint() const override {
        return get_constraint(std::array<double,2>{0,0});
    }

    // For explicitly provided states
    constraint get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2, std::array<double,2> u2) const override {
        Eigen::RowVectorXd a = get_avector(state1, state2);
        double b = get_bscalar(state1, state2, u2);

        return constraint{a, b};
    }

    constraint get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2, std::array<double,2> u2) const override {
        Eigen::RowVectorXd a = get_avector(state1, state2);
        double b = get_bscalar(state1, state2, u2);

        return constraint{a, b};
    }


    double R; // Safety radius

private:

    const StateContainer<double, 2>* const AgentOneState;
    const StateContainer<double, 2>* const AgentTwoState;

    double delta;

    Eigen::VectorXd get_avector() const {
        // Returns -2*(state1 - state2)
        RowVectorXd result = RowVectorXd::Zero(2);
        result[0] = AgentOneState->state[0] - AgentTwoState->state[0];
        result[1] = AgentOneState->state[1] - AgentTwoState->state[1];
        return -2*result;
    }

    double get_bscalar(std::array<double,2> &u2) const {
        VectorXd u2vec = Map<VectorXd>(u2.data(),2);

        RowVectorXd dh_dstate2 = RowVectorXd::Zero(2);
        dh_dstate2[0] = AgentTwoState->state[0] - AgentOneState->state[0];
        dh_dstate2[1] = AgentTwoState->state[1] - AgentOneState->state[1];
        dh_dstate2 *= 2;

        // DEBUG
        cerr << "Distance of state1 and state2: " << sqrt(pow(AgentTwoState->state[0]-AgentOneState->state[0],2) + pow(AgentTwoState->state[1] - AgentOneState->state[1],2)) << endl;
        cerr << "Value of -delta*h(): " << (-delta*h()) << endl;
        cerr << "Value of dh_dstate2*u2vec: " << (dh_dstate2*u2vec) << endl;
        // END DEBUG
        return -delta*h() + dh_dstate2*u2vec;
    }

    // Functions for getting the avector/bvector, but with explicit inputs.
    // Useful when treating Unicycles as single integrators.

    Eigen::VectorXd get_avector(const std::array<double,2> &state1, const std::array<double,2> &state2) const {
        RowVectorXd result = RowVectorXd::Zero(2);
        result[0] = state1[0] - state2[0];
        result[1] = state1[1] - state2[1];
        return -2*result;
    }

    double get_bscalar(const std::array<double,2> &state1, const std::array<double,2> &state2, std::array<double,2> &u2) const {
        VectorXd u2vec = Map<VectorXd>(u2.data(),2);

        RowVectorXd dh_dstate2 = RowVectorXd::Zero(2);
        dh_dstate2[0] = state2[0] - state1[0];
        dh_dstate2[1] = state2[1] - state1[1];
        dh_dstate2 *= 2;

        return -delta*h() + dh_dstate2*u2vec;
    }

};


#endif //IVP_EXTEND_CBF_SINGLE_INTEGRATOR_H
