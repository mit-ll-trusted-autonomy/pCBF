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

/************************************************************/
/*    NAME: James Usevitch                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CBFCore.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef CBFCore_HEADER
#define CBFCore_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <array>
#include <vector>
#include <unordered_map>
#include <map>
//#include <eigen3/Eigen/Dense>
#include "Eigen/Dense" // osqp-cpp uses Eigen 3.3.7
#include <regex>
#include <iterator>
#include <string>
#include <memory>
#include "osqp++.h"

// DEBUG
#include <iostream>
#include <stdexcept>
// END DEBUG

#include "State_Container.h"
#include "CBF_Classes.h"
//#include "CBF_Single_Integrator.h"
#include "CBF_SI_Unicycle.h"

using std::vector, std::string, std::unordered_map, std::map, std::array;
using std::unique_ptr, std::pair;

class CBFCore : public AppCastingMOOSApp {
   public:
    CBFCore();
    // ~CBFCore(); // Follow Rule of Zero.

   protected:  // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

   protected:  // Standard AppCastingMOOSApp function to overload
    bool buildReport();

   protected:
    void registerVariables();

   private:
	// Problem parameters
	int num_CBFs;
	int control_input_size;
    Eigen::MatrixXd A_input;
    Eigen::VectorXd lower_bound_input;
    Eigen::VectorXd upper_bound_input;
    double R_safety;
    double delta;

	// Logistical variables
	vector<string> self_control_input_names;
    vector<string> self_control_input_names_precbf;
	string AgentPrefix;
	string SelfName;
	double RegisterFrequency;
	vector<string> AgentsToAvoid;
	// TODO: Change this later so that StateContainer can be
	// initialized at runtime
	unordered_map<string, VectorContainer<double, 3>> AgentStates;
    unordered_map<string, VectorContainer<double, 2>> AgentInputs;
	vector<unique_ptr<CBF_Parent>> CBFVector;


    // OSQP Variables
    const double kInfinity = std::numeric_limits<double>::infinity();
    Eigen::SparseMatrix<double> P{2,2};
    Eigen::VectorXd lower_bound;
    osqp::OsqpSettings osqp_settings;
//    osqp::OsqpSolver osqp_solver;

    // OSQP Backup problem (if original problem is infeasible)
    Eigen::SparseMatrix<double> P_backup;
    Eigen::VectorXd backup_lower_bound;


	// Functions
	vector<string> parseStringList(string &StringList);
	void updateState(const string &key, double value);
    void updateInputs(const string &key, const double value);
    pair<string, string> getAgentAndVariableName(const string &key);
	void CreateCBFVector();
    void runQP(const array<double,2> &u_nominal_array);
    void parseInputConstraints(string &StringList);
    void NotifyControlInputs(const Eigen::VectorXd &u);
    void SetupBackupProb();
    void SetupBackupProb(int n_slack_variables);

	// Other
	bool debug_verbose;
};

#endif
