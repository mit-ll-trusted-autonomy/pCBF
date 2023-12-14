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

#ifndef IVP_EXTEND_CBF_SINGLE_INTEGRATOR_UNICYCLE_H
#define IVP_EXTEND_CBF_SINGLE_INTEGRATOR_UNICYCLE_H

#include "CBF_Classes.h"



class CBF_SI_Unicycle final : public CBF_Parent {
    /*
     * Simple single integrator CBF class
     */

    typedef Eigen::VectorXd VectorXd;
    typedef Eigen::RowVectorXd RowVectorXd;
    typedef std::pair<RowVectorXd, double> constraint;

public:
    CBF_SI_Unicycle(const VectorContainer<double, 3> &_stateContainer1,
                    const VectorContainer<double, 3> &_stateContainer2,
                    double _delta,
                    double _radius) : AgentOneState{&_stateContainer1},
                                      AgentTwoState{&_stateContainer2},
                                      AgentOneInput{nullptr},
                                      AgentTwoInput{nullptr},
                                      delta{_delta},
                                      R{_radius},
                                      gamma{0.5} {};

    CBF_SI_Unicycle(const VectorContainer<double, 3> &_stateContainer1,
                    const VectorContainer<double, 3> &_stateContainer2,
                    const VectorContainer<double, 2> &_controlInputContainer1,
                    const VectorContainer<double, 2> &_controlInputContainer2,
                    double _delta,
                    double _radius) : AgentOneState{&_stateContainer1},
                                      AgentTwoState{&_stateContainer2},
                                      AgentOneInput{&_controlInputContainer1},
                                      AgentTwoInput{&_controlInputContainer2},
                                      delta{_delta},
                                      R{_radius},
                                      gamma{0.5} {};

    CBF_SI_Unicycle(const VectorContainer<double, 3> &_stateContainer1,
                    const VectorContainer<double, 3> &_stateContainer2,
                    const VectorContainer<double, 2> &_controlInputContainer1,
                    const VectorContainer<double, 2> &_controlInputContainer2,
                    double _delta,
                    double _radius,
                    double _gamma) : AgentOneState{&_stateContainer1},
                                      AgentTwoState{&_stateContainer2},
                                      AgentOneInput{&_controlInputContainer1},
                                      AgentTwoInput{&_controlInputContainer2},
                                      delta{_delta},
                                      R{_radius},
                                      gamma{_gamma} {};

//    CBF_SI_Unicycle(const VectorContainer<double, 3> &_stateContainer1,
//                    const VectorContainer<double, 3> &_stateContainer2,
//                    double _delta,
//                    double _radius,
//                    double _gamma) : AgentOneState{&_stateContainer1},
//                                     AgentTwoState{&_stateContainer2},
//                                     delta{_delta},
//                                     R{_radius},
//                                     gamma{_gamma} {};

//    CBF_SI_Unicycle(const VectorContainer<double, 3> *const _stateContainer1,
//                    const VectorContainer<double, 3> *const _stateContainer2,
//                    double _delta,
//                    double _radius) : AgentOneState{_stateContainer1},
//                                      AgentTwoState{_stateContainer2},
//                                      delta{_delta},
//                                      R{_radius},
//                                      gamma{0.5} {};

    double h() const override {
        // Returns R^2 - norm([x1,y1] - [x2,y2],2)^2

        const double x1{(*AgentOneState)[0]};
        const double y1{(*AgentOneState)[1]};
        const double x2{(*AgentTwoState)[0]};
        const double y2{(*AgentTwoState)[1]};

        return R * R - ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    constraint get_constraint(std::array<double, 2> u2) const override {
        Eigen::RowVectorXd a = get_avector();
        double b = get_bscalar(u2);

        return constraint{a, b};
    }

    constraint get_constraint() const override {
        /*
         * If there is no pointer to AgentTwoInput, then assume that second agent's control input is zero.
         *
         * Otherwise, use second agent's control input.
         */

        if (AgentTwoInput != nullptr){
            // DEBUG
//            cerr << "Values in AgentTwoInput: " << endl;
//            cerr << "[" << AgentTwoInput->values.at(0) << ", " << AgentTwoInput->values.at(1) << "]" << endl;
//            cerr << "Address of AgentTwoInput->values: " << &(AgentTwoInput->values) << endl;
//            cerr << "Address of AgentTwoState->values: " << &(AgentTwoState->values) << endl;
            // END DEBUG
            return get_constraint(AgentTwoInput->values);
        } else {
            return get_constraint(std::array<double, 2>{0.0,0.0});
        }
    }

//    constraint get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2, std::array<double, 2> u2) const override{
//
//    };

//    constraint get_constraint(const std::array<double,2> &state1, const std::array<double,2> &state2) const override{
//
//  };

    double R; // Safety radius
    const double gamma; // Linearization radius

private:

    const VectorContainer<double, 3> *const AgentOneState;
    const VectorContainer<double, 3> *const AgentTwoState;

    const VectorContainer<double, 2> *const AgentOneInput;
    const VectorContainer<double, 2> *const AgentTwoInput;

    double delta; // Defines alpha function: alpha(z) = delta*z.

    Eigen::RowVectorXd get_avector() const {
        /*
         * Returns -2*(state1 - state2), where state1 and state2 are in linearized form
         */
        Eigen::Vector2d state1 = linearize_state(AgentOneState->values);
        Eigen::Vector2d state2 = linearize_state(AgentTwoState->values);

        // Linearized state is [x,y]. No C matrix is needed.
        Eigen::RowVectorXd A_linearized = -2*(state1 - state2);

        double theta = AgentOneState->values[2];

        // DEBUG
        cerr << "state1:\n" << state1 << endl;
        cerr << "state2:\n" << state2 << endl;
        cerr << "A_linearized:\n" << A_linearized << endl;
        // END DEBUG

        return A_linearized*T(theta);

    }

    double get_bscalar(std::array<double, 2> &u2) const {
        /*
         * state1 and state2 are in linearized form
         */

        // DEBUG
        cerr << "Distance between state1 and state2: " << sqrt(pow(AgentOneState->values[0] - AgentTwoState->values[0],2) + pow(AgentOneState->values[1] - AgentTwoState->values[1],2)) << endl;
        cerr << "Value of h(): " << h() << endl;
        cerr << "Value of delta: " << delta << endl;
        cerr << "Value of -delta*h(): " << -delta*h() << endl;
        // END DEBUG

        Eigen::RowVector2d state1 = linearize_state(AgentOneState->values);
        Eigen::RowVector2d state2 = linearize_state(AgentTwoState->values);

        double theta2 = AgentTwoState->values[2];

        VectorXd u2vec = T(theta2)*Eigen::Map<VectorXd>(u2.data(), 2);

        RowVectorXd dh_dstate2 = -2*(state2 - state1);

        double agent_two_term = dh_dstate2*u2vec;
        agent_two_term = (0 < agent_two_term) ? agent_two_term : 0; // Return maximum of agent_two_term and zero
        return -delta * h() - agent_two_term;
    };

    Eigen::Matrix2d T(double theta) const {
        /*
         * Returns the input / output linearization transformation matrix T.
         * Transforms linear and angular control input commands [v;w] into
         * linearized single integrator commands [u_x; uy]
         */

        Eigen::Matrix2d result;
        result << cos(theta), -gamma * sin(theta),
                  sin(theta), gamma * cos(theta);

        return result;
    }

    Eigen::Matrix2d Tinv(double theta) const {
        /*
         * Returns the inverse of T.
         * Transforms single integrator commands [u_x; u_y] to the linear/angular
         * unicycle commands [v;w]
         */
        Eigen::Matrix2d result_inv;
        result_inv << cos(theta), sin(theta),
                      -sin(theta) / gamma, cos(theta) / gamma;

        return result_inv;
    }

    Eigen::Vector2d linearize_state(const std::array<double,3> &state) const {
        double x = state[0];
        double y = state[1];
        double theta = state[2];

        Eigen::Vector2d output;
        output <<   x + gamma*cos(theta),
                    y + gamma*sin(theta);

        return output;
    }

};


#endif //IVP_EXTEND_CBF_SINGLE_INTEGRATOR_UNICYCLE_H
