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

#ifndef IVP_EXTEND_CBF_LIN_DI_UNI_2AGENT_H
#define IVP_EXTEND_CBF_LIN_DI_UNI_2AGENT_H

#include "CBF_Classes.h"

using SC4 = const StateContainer<double, 4>*;
using constraint = std::pair<Eigen::RowVectorXd, double>;

// template <typename AlphaFunction>
class CBF_Lin_DI_Uni_2Agent final : public CBF_Parent {
    /*
    CBF involving two agents with linearized double integrator unicycle
    dynamics.
    */
public:
    double R; // Safety radius

    CBF_Lin_DI_Uni_2Agent(SC4 const _stateContainer1,
                          SC4 const _stateContainer2, double _delta, double _r)
            : AgentOneStateContainer{_stateContainer1},
              AgentTwoStateContainer{_stateContainer2},
              delta{_delta},
              R{_r}{};

    // CBF_Lin_DI_Uni_2Agent(SC4* const _StateContainer1,
    //   SC4* const _StateContainer2, AlphaFunction _alpha)
    // : AgentOneStateContainer{_StateContainer1},
    //   AgentTwoStateContainer{_StateContainer2},
    //   alpha{_alpha} {};

    constraint get_constraint(std::array<double, 2> u2) const override {
        // Compute a = dh/dp1,
        Eigen::RowVectorXd a = get_avector();
        double b = get_bscalar(u2);

        constraint return_val{a, b};
        return return_val;
    };

    constraint get_constraint() const override {
        /*
        Currently this assumes u2 (the second agent's
        control input) is simply zero.

        TODO: Create a more general scheme that handles
        when you don't know the control input.
        */

        return get_constraint(std::array<double, 2>{0, 0});
    };

    // Return the current h() value
    double h() const override{
        // Returns R^2 - norm([x1,y1] - [x2,y2],2)^2

        const double x1{(*AgentOneStateContainer)[0]};
        const double y1{(*AgentOneStateContainer)[1]};
        const double x2{(*AgentTwoStateContainer)[0]};
        const double y2{(*AgentTwoStateContainer)[1]};

        return R * R - ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

private:
    // Private Variables

    const StateContainer<double, 4>* const AgentOneStateContainer;
    const StateContainer<double, 4>* const AgentTwoStateContainer;

    double delta;  // alpha(z) = delta*z, delta > 0

    // Private Functions
    // AlphaFunction alpha;


    Eigen::RowVectorXd get_avector_old() const {
        /*
        NOTES:
                * Assumes that alpha(x) = delta*x, delta > 0
        */
        const double* const in1 = AgentOneStateContainer->state.data();
        const double* const in2 = AgentTwoStateContainer->state.data();

        double t13;
        double t14;
        double t16;
        double t17;
        double t18;
        double t2;
        double t3;
        //     This function was generated by the Symbolic Math Toolbox
        //     version 9.0. 02-Sep-2022 15:45:46
        t2 = std::cos(in1[3]);
        t3 = std::sin(in1[3]);
        t13 = in1[0] * 2.0 + -(in2[0] * 2.0);
        t14 = in1[1] * 2.0 + -(in2[1] * 2.0);
        t16 = delta * (in1[2] * 2.0 + -(in2[2] * 2.0));
        t17 = t2 * t14;
        t18 = t3 * t13;

        Eigen::RowVectorXd avec(2);

        avec[0] = (-t16 - t17) - t18;
        avec[1] =
                ((t16 + t17) + t18) -
                in1[2] *
                ((delta * (in1[3] * 2.0 - in2[3] * 2.0) + t2 * t13 * in1[2]) -
                 t3 * t14 * in1[2]);

        return avec;
    }


    Eigen::RowVectorXd get_avector() const {
        const double* const in1 = AgentOneStateContainer->state.data();
        const double* const in2 = AgentTwoStateContainer->state.data();

        double t13;
        double t14;
        double t16;
        double t17;
        double t18;
        double t2;
        double t3;
        //     This function was generated by the Symbolic Math Toolbox version 9.0.
        //     14-Sep-2022 17:05:08
        t2 = std::cos(in1[3]);
        t3 = std::sin(in1[3]);
        t13 = in1[0] * 2.0 + -(in2[0] * 2.0);
        t14 = in1[1] * 2.0 + -(in2[1] * 2.0);
        t16 = delta * (in1[2] * 2.0 + -(in2[2] * 2.0));
        t17 = t2 * t14;
        t18 = t3 * t13;

        Eigen::RowVectorXd avec(2);

        avec[0] = (-t16 - t17) - t18;
        avec[1] =
                ((t16 + t17) + t18) -
                in1[2] * ((delta * (in1[3] * 2.0 - in2[3] * 2.0) + t2 * t13 * in1[2]) -
                          t3 * t14 * in1[2]);

        return avec;
    }


    double get_bscalar_old(std::array<double, 2>& u2) const {
        const double* const in1 = AgentOneStateContainer->state.data();
        const double* const in2 = AgentTwoStateContainer->state.data();
        const double* const in4 = u2.data();

        double a;
        double b_a;
        double c_a;
        double d_a;
        double t12;
        double t13;
        //     This function was generated by the Symbolic Math Toolbox
        //     version 9.0. 02-Sep-2022 15:45:47
        t12 = in1[0] * 2.0 + -(in2[0] * 2.0);
        t13 = in1[1] * 2.0 + -(in2[1] * 2.0);
        a = in1[0] - in2[0];
        b_a = in1[1] - in2[1];
        c_a = in1[2] - in2[2];
        d_a = in1[3] - in2[3];
        t12 = ((std::cos(in1[3]) * t13 * in1[2] +
                std::sin(in1[3]) * t12 * in1[2]) +
               -(std::cos(in2[3]) * t13 * in2[2])) +
              -(std::sin(in2[3]) * t12 * in2[2]);
        return ((t12 - (in1[2] * 2.0 - in2[2] * 2.0) * (in4[0] - in4[1])) +
                delta * (t12 + delta * ((((a * a + b_a * b_a) + c_a * c_a) +
                                         d_a * d_a) -
                                        R * R))) -
               in4[1] * in2[2] * (in1[3] * 2.0 - in2[3] * 2.0);
    }

    double get_bscalar(std::array<double,2>& u2) const {
        const double* const in1 = AgentOneStateContainer->state.data();
        const double* const in2 = AgentOneStateContainer->state.data();
        const double* const in4 = u2.data();

        double a;
        double b_a;
        double b_bval_tmp;
        double bval_tmp;
        double c_a;
        double d_a;
        double t18;
        double t19;
        double t2;
        double t22;
        double t23;
        double t3;
        double t4;
        double t5;
        //     This function was generated by the Symbolic Math Toolbox version 9.0.
        //     14-Sep-2022 17:05:09
        t2 = std::cos(in1[3]);
        t3 = std::cos(in2[3]);
        t4 = std::sin(in1[3]);
        t5 = std::sin(in2[3]);
        t18 = in1[0] * 2.0 + -(in2[0] * 2.0);
        t19 = in1[1] * 2.0 + -(in2[1] * 2.0);
        t22 = (t2 * in1[2] * 2.0 + -(t3 * in2[2] * 2.0)) + delta * t19;
        t23 = (t4 * in1[2] * 2.0 + -(t5 * in2[2] * 2.0)) + delta * t18;
        a = in1[0] - in2[0];
        b_a = in1[1] - in2[1];
        c_a = in1[2] - in2[2];
        d_a = in1[3] - in2[3];
        bval_tmp = t3 * t19;
        b_bval_tmp = t5 * t18;
        return (((((delta *
                    ((((delta * ((((a * a + b_a * b_a) + c_a * c_a) + d_a * d_a) -
                                 R * R) +
                        t2 * t19 * in1[2]) +
                       t4 * t18 * in1[2]) -
                      bval_tmp * in2[2]) -
                     b_bval_tmp * in2[2]) -
                    (in4[0] - in4[1]) * ((bval_tmp + b_bval_tmp) +
                                         delta * (in1[2] * 2.0 - in2[2] * 2.0))) -
                   in4[1] * in2[2] *
                   ((delta * (in1[3] * 2.0 - in2[3] * 2.0) + t3 * t18 * in2[2]) -
                    t5 * t19 * in2[2])) +
                  t2 * t22 * in1[2]) +
                 t4 * t23 * in1[2]) -
                t3 * t22 * in2[2]) -
               t5 * t23 * in2[2];
    }

};


#endif //IVP_EXTEND_CBF_LIN_DI_UNI_2AGENT_H
