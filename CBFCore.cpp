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
/*    FILE: CBFCore.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include "CBFCore.h"

#include "ACTable.h"
#include "MBUtils.h"

// DEBUG
#include <functional>
// END DEBUg

using namespace std;

constexpr double pi = 3.141592653589793238462643383279502884197169399375; // Probably overkill, but ¯\_(ツ)_/¯

/*
Utility function for checking if element is in std::vector
NOTE: Profile this function later to see if it affects performance.
Maybe try inlining it.
*/
template<typename T>
bool is_in(const T &element, const vector<T> &vec){
    if (find(vec.begin(), vec.end(), element) != vec.end()) {
        return true;
    }
    else {
        return false;
    }
    return false;
}

/*
 * Utility function for cout'ing an array
 */

template<typename T, size_t N>
ostream& operator<<(ostream &os, const array<T,N> &_array){
    os << "[";
    for (const auto &item : _array) {
        os << item << ", ";
    }
    os << "]";
}



//---------------------------------------------------------
// Constructor

CBFCore::CBFCore() {
    num_CBFs = 0;
    control_input_size = 0;

    RegisterFrequency = 0;

    AgentStates["SELF"] =
        VectorContainer<double, 3>{array<double, 3>{0, 0}};

    // Set OSQP objective matrix to identity
    P.setIdentity();

    // Set default CBF parameters
    R_safety = 10.0; // Safety radius
    delta = 1; // Multiplication constant \alpha(x) = -\delta * x

    debug_verbose = false;
}

//---------------------------------------------------------
// Destructor
/*
NOTE: Do not declare the destructor unless you're actually
going to use it. Follow the rule of zero:
https://www.fluentcpp.com/2019/04/23/the-rule-of-zero-zero-constructor-zero-calorie/
*/

// CBFCore::~CBFCore()
// {
// }

//---------------------------------------------------------
// Procedure: OnNewMail

bool CBFCore::OnNewMail(MOOSMSG_LIST &NewMail) {
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for (p = NewMail.begin(); p != NewMail.end(); p++) {
        CMOOSMsg &msg = *p;
        string key = msg.GetKey();

        if (key == "NAV_X" || key == "NAV_Y" || key == "NAV_SPEED" ||
            key == "NAV_HEADING") {
            // Update own state
            updateState(key, msg.GetDouble());
        } else if (std::regex_match(
                       key,
                       std::regex(
                           AgentPrefix +
                           "[0-9]+(_NAV_X|_NAV_Y|_NAV_SPEED|_NAV_HEADING)"))) {
            /*
            Update information about other agents' states.
            TODO: 	This blindly collects information for all agents. If we
                    want to restrict this in the future to only certain agents, we'll
                    need to alter this.
            */

            // Update the agent's state in States_AgentAvoid;
            updateState(key, msg.GetDouble());
        } else if(is_in(key, self_control_input_names_precbf)) {
            updateInputs(key, msg.GetDouble());
        } else if(std::regex_match(
                key,
                std::regex(AgentPrefix + "[0-9]+(_DESIRED_THRUST|_DESIRED_RUDDER)"))) {
            updateInputs(key, msg.GetDouble());
        } else if (key != "APPCAST_REQ") {  // handled by AppCastingMOOSApp
            reportRunWarning("Unhandled Mail: " + key);
        }
    }

    return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool CBFCore::OnConnectToServer() {
    // registerVariables();
    return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool CBFCore::Iterate() {
    AppCastingMOOSApp::Iterate();
    // Do your thing here!

    runQP(AgentInputs[SelfName].values);
    AppCastingMOOSApp::PostReport();
    return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CBFCore::OnStartUp() {
    AppCastingMOOSApp::OnStartUp();

    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
        reportConfigWarning("No config block found for " + GetAppName());

    // Logistical variables
    string CONTROL_CONSTRAINTS;

    STRING_LIST::iterator p;
    for (p = sParams.begin(); p != sParams.end(); p++) {
        string orig = *p;
        string line = *p;
        string param = toupper(biteStringX(line, '='));
        string value = line;

        bool handled = false;
        if (param == "SELF_NAME") {
            SelfName = value;
            handled = true;
        } else if (param == "CONTROL_INPUTS") {
            /*
            TODO: This currently assumes all agents have DESIRED_THRUST /
            DESIRED_RUDDER as control inputs. Change to be more general.
            */
            self_control_input_names = parseStringList(value);
            control_input_size = self_control_input_names.size();

            // Set the pre-CBF control input variable names. These are the variables this app
            // will listen for and modify.
            self_control_input_names_precbf.resize(control_input_size);
            for (std::size_t ii=0; ii < self_control_input_names.size(); ii++){
                self_control_input_names_precbf[ii] = self_control_input_names[ii] + "_PRECBF";
            }
            handled = true;
        } else if(param == "CONTROL_CONSTRAINTS") {
            CONTROL_CONSTRAINTS = value;
//            parseInputConstraints(value);
            handled = true;
        } else if (param == "AGENT_PREFIX") {
            AgentPrefix = value;
            handled = true;
        } else if (param == "AGENTS_TO_AVOID") {
            AgentsToAvoid = parseStringList(value);
            handled = true;
        } else if(param == "SAFETY_RADIUS") {
            R_safety = stod(value);
            handled = true;
        } else if (param == "DELTA") {
            delta = stod(value);
            handled = true;
        } else if (param == "REGISTER_FREQUENCY") {
            RegisterFrequency = stod(value);
            handled = true;
        } else if (param == "DEBUG_VERBOSE") {
            if (toupper(value).substr(0, 4) == "TRUE") {
                debug_verbose = true;
            } else {
                debug_verbose = false;
            }
            handled = true;
        } else if (param == "OSQP_VERBOSE") {
            osqp_settings.verbose = (tolower(value) == "true");
            handled = true;
        } else if (param == "OSQP_MAX_ITER") {
            osqp_settings.max_iter = stoi(value);
            handled = true;
        }

        if (!handled) reportUnhandledConfigWarning(orig);
    }

    CreateCBFVector();
    parseInputConstraints(CONTROL_CONSTRAINTS);
    // Create the objective P matrix for the backup optimization problem.
    // Number of slack variables is one less than number of CBFs. (First CBF
    // is always enforced)
    SetupBackupProb(num_CBFs-1);

    registerVariables();

    return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void CBFCore::registerVariables() {
    AppCastingMOOSApp::RegisterVariables();

    // Self variables
    Register("NAV_X", RegisterFrequency);
    Register("NAV_Y", RegisterFrequency);
    Register("NAV_SPEED", RegisterFrequency);
    Register("NAV_HEADING", RegisterFrequency);

    Register("DESIRED_THRUST_PRECBF", RegisterFrequency);
    Register("DESIRED_RUDDER_PRECBF", RegisterFrequency);

    // Register for agents to avoid
    for (auto agent_name : AgentsToAvoid) {
        if (agent_name != SelfName) {
            Register(agent_name + "_NAV_X", RegisterFrequency);
            Register(agent_name + "_NAV_Y", RegisterFrequency);
            Register(agent_name + "_NAV_SPEED", RegisterFrequency);
            Register(agent_name + "_NAV_HEADING", RegisterFrequency);

            Register(agent_name + "_DESIRED_THRUST", RegisterFrequency);
            Register(agent_name + "_DESIRED_RUDDER", RegisterFrequency);
        }
    }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool CBFCore::buildReport() {
    m_msgs << "============================================" << endl;
    m_msgs << "File:                                       " << endl;
    m_msgs << "============================================" << endl;

    ACTable actab(4);
    actab << "Alpha | Bravo | Charlie | Delta";
    actab.addHeaderLines();
    actab << "one"
          << "two"
          << "three"
          << "four";
    m_msgs << actab.getFormattedString();

//    m_msgs << "\nControl Input Names:" << endl;
//    for (const auto &name : self_control_input_names) {
//        m_msgs << name << endl;
//    }
//    m_msgs << "\n State Values:" << endl;
//    // DEBUG
//    m_msgs << "Size of AgentStates: " << AgentStates.size() << endl;
//    // END DEBUG
//    for (const auto &[key, value] : AgentStates) {
//        m_msgs << key << ": " << value << endl;
//    }

    m_msgs << "\nCBFVector size: " << CBFVector.size() << endl;
    m_msgs << "\nNumber of CBFs: " << num_CBFs << endl;
    m_msgs << "CBFVector values:" << endl;
    for (auto &cbf : CBFVector){
        m_msgs << cbf->h() << endl;
    }

    m_msgs << "\nAgentInput size: " << AgentInputs.size() << endl;
    m_msgs << "\nAgentInput values: " << endl;
    for (const auto &[key, value] : AgentInputs) {
        m_msgs << key << ": " << value << endl;
    }

    m_msgs << "\nAgentstoAvoid size: " << AgentsToAvoid.size() << endl;
    m_msgs << "\nAgentstoAvoid values:" << endl;
    for (const auto &name : AgentsToAvoid){
        m_msgs << name << endl;
    }

    m_msgs << "Safety Radius: " << R_safety << endl;


    return (true);
}

//------------------------------------------------------------
// Other Functions

// Parse a string list
vector<string> CBFCore::parseStringList(string &StringList) {
    vector<string> outList = {};

    while (!StringList.empty()) {
        outList.push_back(biteStringX(StringList, ','));
    }

    return outList;
}

// Update state information
void CBFCore::updateState(const string &key, double value) {
    /*
      Updates state information about other agents when new mail is received.

      An unordered_map maps agent names to a std::array with the state values.
    */

    // Get agent name and variable name

    string agent_name;
    string variable_name;
    tie(agent_name, variable_name) = getAgentAndVariableName(key);

    int column = -1;

    if (variable_name == "NAV_X") {
        column = 0;
    } else if (variable_name == "NAV_Y") {
        column = 1;
    } else if (variable_name == "NAV_HEADING") {
        // TODO: This is a specific hack for linearized single integrators
        //       Transform naval heading into "normal" heading
        //       Naval heading treats due North as zero and 90 degrees is due East
        //       "Normal" heading treats X-axis as zero and y-axis as pi/2

        // Transform value (naval degrees) into normal theta (radians)
        double normal_degrees = -value + 90;
        double normal_radians = normal_degrees*pi/180;
        value = normal_radians;
        column = 2;
    } else {
        // Nothing to be done; not a variable that needs updating.
        return;
    }


    // Store the appropriate value
    // Ordering of values in array:
    //  [NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING]
//    if (result != AgentStates.end()) {
//        // Double check that entry doesn't exist
//        for (auto iter = AgentStates.begin(); iter != AgentStates.end(); ++iter){
//            if (agent_name ==iter->first){
//                result = iter;
//            }
//        }
//        result = AgentStates.find(agent_name);
//    }

    auto result = AgentStates.find(agent_name);
    if (result != AgentStates.end()) {
        // Key already exists in the hash table.
        AgentStates.at(agent_name)[column] = value;
    } else {
        // Key does not exist. Add the state array to the hash table.
        AgentStates.insert({agent_name, VectorContainer<double,3>{std::array<double,3>{0, 0}}});

        // Update the key
        AgentStates.at(agent_name)[column] = value;
    }
}



void CBFCore::updateInputs(const string &key, const double value) {

    string agent_name, variable_name;
    tie(agent_name, variable_name) = getAgentAndVariableName(key);

    int column = -1;

    if (variable_name == "DESIRED_THRUST" || variable_name == "DESIRED_THRUST_PRECBF"){
        column = 0;
    } else if (variable_name == "DESIRED_RUDDER" || variable_name == "DESIRED_RUDDER_PRECBF"){
        column = 1;
    }


    if (AgentInputs.find(agent_name) != AgentInputs.end()){
        // Update the control input values.
        AgentInputs[agent_name][column] = value;

        // DEBUG
//        cerr << "All values in AgentInputs: " << endl;
//        for(const auto &[key, value] : AgentInputs) {
//            cerr << key << ": " << value << endl;
//            cerr << "Address location for " << key << ": " << &AgentInputs[key] << endl;
//            cerr << "State address location for " << key << ": " << &AgentStates[key] << endl;
//        }
        // END DEBUG
    } else {
        // Key does not exist. Add it to hash table.
        AgentInputs[agent_name] = std::array<double, 2>{0.0,0.0};
        AgentInputs[agent_name][column] = value;
        // DEBUG
        if (debug_verbose){
            cerr << "\nupdateInputs: Key was NOT present in hash table." << endl;
        }
    }
}



pair<string, string> CBFCore::getAgentAndVariableName(const string &key) {
    /*
     * Splits a key string into an agent name and variable name.
     *
     * For example, splits AGENT_1_NAV_X into agent name (AGENT_1) and
     * variable name (NAV_X)
     */
    smatch agent_name_match;
    string agent_name;
    string variable_name;

    if (regex_search(key, agent_name_match, regex(AgentPrefix + "[0-9]+"))) {
        agent_name = agent_name_match.str(0);
        variable_name = key.substr(agent_name_match.str(0).size() + 1);
    } else {
        // No prefix means that the information refers to itself.
        agent_name = SelfName;
        variable_name = key.substr(agent_name_match.str(0).size());
    };

    return std::pair<std::string, std::string>{agent_name, variable_name};
}



void CBFCore::CreateCBFVector() {
    /*
    Creates all CBF objects.

    TODO: Currently only considers double integrator unicycles.
    Change this to consider multiple types of CBFs in the future.
    */

    // Other Agents
    for (auto other_agent : AgentsToAvoid) {
        if (other_agent != SelfName) {
            CBFVector.emplace_back(
				make_unique<CBF_SI_Unicycle>(
					AgentStates[SelfName],
					AgentStates[other_agent],
                    AgentInputs[SelfName],
                    AgentInputs[other_agent],
					delta,
                    R_safety
				)
			);
            num_CBFs++;
        }
    }

    // Keep-Out Regions
    // TODO

    // Stay-in Regions
    // TODO


    // Set the lower_bound vector to negative infinity
    lower_bound = Eigen::VectorXd::Constant(num_CBFs, -kInfinity);

}


void CBFCore::runQP(const array<double,2> &u_nominal_array) {
    /*
     * Minimally modifies the control input via a QP.
     *
     * Solver is OSQP.
     */

    Eigen::VectorXd u_nominal = Eigen::Map<const Eigen::VectorXd>(u_nominal_array.data(), 2);

    // Collect all CBF constraints into Matrix
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_CBFs, control_input_size);
    Eigen::VectorXd b_vec(num_CBFs);

    Eigen::RowVectorXd a;
    double b_val;
    for (std::size_t idx=0; idx < CBFVector.size(); idx++) {
        std::tie(a,b_val) = CBFVector[idx]->get_constraint();
        A.row(idx) = a;
        b_vec[idx] = b_val;
    }

    // Add control constraints
    // TODO: Make this more efficient
    Eigen::MatrixXd A_final(A.rows() + A_input.rows(), A.cols());
    Eigen::VectorXd b_final(b_vec.size()+upper_bound_input.size());
    Eigen::VectorXd lower_bound_final(b_final.size());
    A_final << A, A_input;
    b_final << b_vec, upper_bound_input;
    lower_bound_final << lower_bound, lower_bound_input;

    // DEBUG
//    cerr << "A_input:\n" << A_input << endl;
//    cerr << "lower_bound_final:\n" << lower_bound_final << endl;
    cerr << "A_final:\n" << A_final << endl;
    cerr << "b_final:\n" << b_final << endl;
//    cerr << "lower_bound_final:\n" << lower_bound_final << endl;
    // END DEBUG

    /*
     * uSimMarine uses a naval coordinate system where due north is 0 degrees
     * and East is 90 degrees. This means that the angular velocity command
     * will be the negative of a normal coordinate frame where 0 degrees is
     * the x axis and 90 degrees is the y axis.
     *
     * We convert u_nominal to a normal coordinate frame for the CBF
     * optimization, then convert the solution back to the naval frame.
     *
     * In the conversion,
     *      * Linear velocity v (entry 0) is the same
     *      * Angular velocity w (entry 1) reverses sign.
     */
    Eigen::VectorXd normal_frame_u_nominal = u_nominal;
    normal_frame_u_nominal[1] = -u_nominal[1];


    // Check feasibility of u_nominal.
    // If u_nominal is feasible, return it without running QP
    if (((A*normal_frame_u_nominal - b_vec).array() <= 0).all()) {
        // DEBUG
        if (debug_verbose){
            cerr << "\nQP skipped; u_nominal feasible.\n" << endl;
        }
        // END DEBUG
        NotifyControlInputs(u_nominal);
    }

    // Set up QP
    osqp::OsqpInstance problem;
    problem.objective_matrix = P;
    problem.objective_vector = -normal_frame_u_nominal; // TODO: Check if this needs to be multiplied by 2
    problem.constraint_matrix = A_final.sparseView();
    problem.lower_bounds = lower_bound_final;
    problem.upper_bounds = b_final;


    // Solve QP
    // TODO: Benchmark how fast it is to set up a new solver each time?
    //      With multi-agent CBFs, the number of constraints may change as
    //      more agents enter/exit the picture.
    osqp::OsqpSolver solver;
    auto status = solver.Init(problem, osqp_settings);

    if (!status.ok()){
        // Display error message and use zero control
        cerr << "\nERROR: status is NOT ok after init." << endl;
        cerr << "Status: " << status << endl;
        cerr << "\nDefaulting to zero control.\n" << endl;
        NotifyControlInputs(Eigen::VectorXd::Zero(u_nominal.size()));
        return;
    }

    osqp::OsqpExitCode exit_code = solver.Solve();

    if (exit_code == osqp::OsqpExitCode::kOptimal || exit_code == osqp::OsqpExitCode::kOptimalInaccurate) {

        // Change the angular velocity from the normal frame to the naval frame
        Eigen::VectorXd final_u = solver.primal_solution();
        final_u[1] = -final_u[1];



        // Condition forward velocity to be strictly nonnegative
        if (final_u[0] < 0) {
            final_u[0] = 0;
            // DEBUG
            cerr << "THE CONTROL INPUT WAS MODIFIED TO HAVE NONNEGATIVE FORWARD VELOCITY" << endl;
            // END DEBUG
        }

        // DEBUG
        cerr << "Original control input:\n" << u_nominal << endl;
        cerr << "Modified control input (naval):\n" << final_u << endl;
        Eigen::VectorXd orig_final_u = solver.primal_solution();
        if (orig_final_u[0] < 0) {
            orig_final_u[0] = 0;
        }
        cerr << "Modified control input (normal):\n" << orig_final_u << endl;
        cerr << "A_final*orig_final_u - b_final:\n" << A_final*orig_final_u - b_final << endl;
        cerr << "T(theta):\n" << CBFVector[0]->T(AgentStates[SelfName][2]) << endl;
        cerr << "Value of T(theta)*orig_final_u:\n" << CBFVector[0]->T(AgentStates[SelfName][2])*orig_final_u << endl;
        cerr << "Delta: " << delta << endl;
        // END DEBUG

        NotifyControlInputs(final_u);
        return;
    } else {
        /*
         * Infeasible QP. Compute the "best effort" control input.
         *
         * The best effort control input finds the closest feasible control input
         * to the safe set of inputs. Roughly speaking, we solve
         *
         * Problem:
         *      min ||u - z||^2
         *      s.t.
         *          A*T(theta)*z <= b_vec
         *          A_input*u <= upper_bound_input
         *
         * However, if A is infeasible then the problem still fails. We add slack
         * variables d_i to all CBF constraints except for the first one. In
         * essence this makes the first constraint "hard" and the rest "soft".
         * The first CBF will always be satisfied, and the rest will be satisfied
         * if possible.
         *
         * Problem with slack variables:
         *      min_{u,z,d_i} ||u - z||^2 + d_2^2 + d_3^2 + ...
         *      s.t.
         *          a_1*z <= b_vec
         *          a_i*z + d_i <= b_vec, i=2,...
         *          A_input*u <= upper_bound_input
         *
         * TODO: Is it better to make d_i non-negative with constraints and have linear sum in objective?
         */
        cerr << "\nERROR: Optimal solution not found. Exit code: " << ToString(exit_code) << "\n" << endl;
        cerr << "Defaulting to best-effort controller" << endl;


        Eigen::MatrixXd A_backup(A.rows() + A_input.rows(), A.cols() + A_input.cols() + num_CBFs - 1);
        A_backup.setZero();

        int startRow {0};
        int startCol {0};

        // A_input*u <= upper_bound_input
        A_backup.block(startRow,startCol, A_input.rows(), A_input.cols()) = A_input;

        startRow += A_input.rows();
        startCol += A_input.cols();

        // A*z + slack_vec <= bvec
        A_backup.block(startRow, startCol, A.rows(), A.cols()) = A;

        startRow += 1; // We don't add a slack variable to the first CBF constraint
        startCol += A.cols();

        A_backup.block(startRow, startCol, num_CBFs-1, num_CBFs-1).setIdentity();


        // Set the b vector
        Eigen::VectorXd b_backup(A_input.rows() + A.rows());
        b_backup << upper_bound_input, b_vec;




        osqp::OsqpInstance backup_problem;
        backup_problem.objective_matrix = P_backup;
        backup_problem.objective_vector = Eigen::VectorXd::Zero(P_backup.cols());
        backup_problem.constraint_matrix = A_backup.sparseView();
        backup_problem.lower_bounds = backup_lower_bound;
        backup_problem.upper_bounds = b_backup;

        // DEBUG
        cerr << "\nP_backup:\n" << Eigen::MatrixXd(P_backup) << endl;
        cerr << "A_backup:\n" << Eigen::MatrixXd(A_backup.sparseView()) << endl;
        cerr << "backup_lower_bound:\n" << backup_lower_bound << endl;
        cerr << "b_backup:\n" << b_backup << endl;
        // END DEBUG

        osqp::OsqpSolver backup_solver;
        auto backup_status = backup_solver.Init(backup_problem, osqp_settings);

        if (!backup_status.ok()) {
            // Display error message and use zero control
            cerr << "\nERROR: backup QP status is NOT ok after init." << endl;
            cerr << "Status: " << backup_status << endl;
            cerr << "\nDefaulting to zero control.\n" << endl;
            NotifyControlInputs(Eigen::VectorXd::Zero(u_nominal.size()));
            return;
        }

        osqp::OsqpExitCode backup_exit_code = backup_solver.Solve();

        if (backup_exit_code == osqp::OsqpExitCode::kOptimal || backup_exit_code == osqp::OsqpExitCode::kOptimalInaccurate) {
            // DEBUG
            cerr << "Backup control input: " << backup_solver.primal_solution() << endl;
            // END DEBUG


            // Change the angular velocity from the normal frame to the naval frame
            // TODO: Make final_u part of the class so we're not allocating / reallocating every function call
            Eigen::VectorXd primal_sol = backup_solver.primal_solution();
            Eigen::VectorXd final_u = Eigen::VectorXd::Zero(2);
            final_u[0] = primal_sol[0];
            final_u[1] = -primal_sol[1];
//            final_u[1] = -final_u[1];

            // Condition forward velocity to be strictly nonnegative
            if (final_u[0] < 0) {
                final_u[0] = 0;
            }

            // DEBUG
            cerr << "******************\n" << endl;
            cerr << "final_u: " << final_u << endl;
            cerr << "primal_sol: " << primal_sol << endl;
            cerr << "backup_solver.primal_solution(): " << backup_solver.primal_solution() << endl;
            cerr << "\n******************" << endl;
            // END DEBUG

            NotifyControlInputs(final_u);
            return;

        } else {
            // Something went horribly wrong.
            // Default to zero control

            // DEBUG
            cerr << "\n!!! BACKUP QP FAILED. Something went horribly wrong. !!!" << endl;
            cerr << "Backup exit code: " << osqp::ToString(backup_exit_code) << endl;
            cerr << "Defaulting to zero control.\n" << endl;
            // END DEBUG

            // DEBUG
            cerr << "Feasible point times A_backup:\n" << A_backup*Eigen::Vector4d{0,0,-1000*A_backup(2,2),-1000*A_backup(2,3)} << endl;
            // END DEBUG

            NotifyControlInputs(Eigen::VectorXd::Zero(u_nominal.size()));
            return;
        }

        // If all else fails, default to a zero control.
        NotifyControlInputs(Eigen::VectorXd::Zero(u_nominal.size()));
        return;
    }

    // MONKEY_PATCH: Return the original control for now
    cerr << "\nWARNING: Something unexpected happened. Defaulting to original (unsafe) control input.\n" << endl;
    // DEBUG
    cerr << u_nominal;
    // END DEBUG
    NotifyControlInputs(u_nominal);
}

void CBFCore::parseInputConstraints(string &StringList) {
    /*
     * Creates control input constraints from the CONTROL_CONSTRAINTS parameter in the .moos file.
     *
     * Each control input is a scalar (MOOSDB variables can only hold scalars)
     */
    vector<tuple<int, double, double>> index_lower_upper_bounds; // Control input index, lower bound, upper bound;
    string control_name;
    string lowerbound_string;
    string upperbound_string;
    double lowerbound;
    double upperbound;


    // Parse control input bounds
    for (size_t idx=0; idx < self_control_input_names.size(); idx++) {
        control_name = self_control_input_names[idx];
        string values = tokStringParse(toupper(StringList), toupper(control_name), ',', '=');

        if (!values.empty()) {
            // Extract the bounds from values
            // TODO: Make this more robust
            lowerbound_string = biteStringX(values,':');
            upperbound_string = values;

            if (toupper(lowerbound_string) == "-INF" || toupper(lowerbound_string).empty()){
                lowerbound = -kInfinity;
            } else {
                lowerbound = stod(lowerbound_string);
            }

            if (toupper(upperbound_string) == "INF" || toupper(upperbound_string).empty()){
                upperbound = kInfinity;
            } else {
                upperbound = stod(upperbound_string);
            }
            index_lower_upper_bounds.push_back(tuple<int,double,double>{idx, lowerbound, upperbound});
        }
    }

    // Create Eigen control input constraint matrix
    // TODO: Make this Matrix representation more efficient in the future

    size_t n_constraints = index_lower_upper_bounds.size();
    A_input = Eigen::MatrixXd::Zero(n_constraints, control_input_size);
    lower_bound_input = Eigen::VectorXd::Zero(n_constraints);
    upper_bound_input = Eigen::VectorXd::Zero(n_constraints);

    int col;
    for (size_t idx=0; idx < index_lower_upper_bounds.size(); idx++) {
        tie(col, lowerbound, upperbound) = index_lower_upper_bounds[idx];

        A_input(idx, col) = 1;
        lower_bound_input(idx) = lowerbound;
        upper_bound_input(idx) = upperbound;
    }
}

void CBFCore::NotifyControlInputs(const Eigen::VectorXd &u) {
    for (size_t ii=0; ii < self_control_input_names.size(); ii++){
        Notify(self_control_input_names[ii], u[ii]);
    }
}


void CBFCore::SetupBackupProb(int n_slack_variables) {
    /*
     * Populates the objective matrix and constraints for the CBF backup optimization problem.
     * The objective matrix has the form
     * [[I, -I]
     *  [-I, I]]
     *
     *  This is used for the optimization objective
     *      minimize_{[u;z]} ||u - z||^2 = (u-z)^T(u-z)
     *
     * The constraints have the form
     *      [0, A] [u;z] <= b
     *      [lower; -inf] <= [u;z] <= [upper; -inf] (box constraints)
     *
     * I put this in a separate function to keep the constructor relatively clean.
     */

    P_backup = Eigen::SparseMatrix<double>(4+n_slack_variables, 4+n_slack_variables);

    typedef Eigen::Triplet<double> T;
    vector<T> triplets;
    triplets.reserve(8 + n_slack_variables);

    // Identity matrices in top left corner
    for (int idx=0; idx < 4+n_slack_variables; idx++) {
        triplets.push_back(T(idx, idx, 1));
    }

    // Negative identity matrices in top left corner
    triplets.push_back(T(2,0,-1));
    triplets.push_back(T(3,1,-1));
    triplets.push_back(T(0,2,-1));
    triplets.push_back(T(1,3,-1));

    P_backup.setFromTriplets(triplets.begin(), triplets.end());

    // Set the backup lower bound vector
    backup_lower_bound = Eigen::VectorXd::Constant(A_input.rows() + num_CBFs, -kInfinity);
    for (size_t idx = 0; idx < lower_bound_input.size(); idx++) {
        backup_lower_bound[idx] = lower_bound_input[idx];
    }

    // DEBUG
    // Set a non-zero, positive lower bound on the forward velocity.
    // This ensures the vehicle can turn.
    if (backup_lower_bound[0] == 0) {
        backup_lower_bound[0] = 10;
    }
    // END DEBUG
}

void CBFCore::SetupBackupProb() {
    SetupBackupProb(0);
}



