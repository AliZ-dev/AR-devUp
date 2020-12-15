// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

#include <Eigen/LU>
// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/jntarrayacc.hpp>
#include <kdl/jntarray.hpp>
#include <math.h>
#include <numeric>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/framevel.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49

namespace arm_controllers
{
class DemoController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);
        seg_jac_1.resize(1);
        seg_jac_2.resize(2);
        seg_jac_3.resize(3);
        seg_jac_4.resize(4);
        seg_jac_5.resize(5);
        J_.resize(n_joints_);
        K_kine_.resize(n_joints_);
        K_kine_.data(0) = 1.0;
        K_kine_.data(1) = 1.0;
        K_kine_.data(2) = 1.0;
        K_kine_.data(3) = 1.0;
        K_kine_.data(4) = 1.0;
        K_kine_.data(5) = 1.0;

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/demo_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/demo_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/demo_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/demo_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        FKSolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        FKSolver_vel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        jnt_to_jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));

        kdl_tree_.getChain("world", "elfin_link1", kdl_chain_1);
        jac_solver_1.reset(new KDL::ChainJntToJacSolver(kdl_chain_1));
        fk_solver_1.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_1));

        kdl_tree_.getChain("world", "elfin_link2", kdl_chain_2);
        jac_solver_2.reset(new KDL::ChainJntToJacSolver(kdl_chain_2));
        fk_solver_2.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_2));

        kdl_tree_.getChain("world", "elfin_link3", kdl_chain_3);
        jac_solver_3.reset(new KDL::ChainJntToJacSolver(kdl_chain_3));
        fk_solver_3.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_3));

        kdl_tree_.getChain("world", "elfin_link4", kdl_chain_4);
        jac_solver_4.reset(new KDL::ChainJntToJacSolver(kdl_chain_4));
        fk_solver_4.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_4));

        kdl_tree_.getChain("world", "elfin_link5", kdl_chain_5);
        jac_solver_5.reset(new KDL::ChainJntToJacSolver(kdl_chain_5));
        fk_solver_5.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_5));




        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);
        xdot_.data = Eigen::VectorXd::Zero(n_joints_);
        ex_task_ = Eigen::VectorXd::Zero(n_joints_);
        jnt_limits_lower_.data = Eigen::VectorXd::Zero(n_joints_);
        jnt_limits_upper_.data = Eigen::VectorXd::Zero(n_joints_);
        diff_to_low_.data = Eigen::VectorXd::Zero(n_joints_);
        diff_to_low_prev_.data = Eigen::VectorXd::Zero(n_joints_);
        diff_to_upper_.data = Eigen::VectorXd::Zero(n_joints_);
        diff_to_upper_prev_.data = Eigen::VectorXd::Zero(n_joints_);
        ex_obstacle_prev_ = Eigen::VectorXd::Zero(n_joints_);
        qdot_vel_array_.q.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_vel_array_.qdot.data = Eigen::VectorXd::Zero(n_joints_);

        obstacle_.p(0) = -0.1;
        obstacle_.p(1) = -0.1;
        obstacle_.p(2) = 0.75;
        obstacle_.M.RPY(0,0,0);

        jnt_limits_lower_(0) = -0.6;
        jnt_limits_lower_(1) = -0.6;
        jnt_limits_lower_(2) = -0.6;
        jnt_limits_lower_(3) = -0.6;
        jnt_limits_lower_(4) = -0.6;
        jnt_limits_lower_(5) = -0.6;

        jnt_limits_upper_(0) = 0.6;
        jnt_limits_upper_(1) = 0.6;
        jnt_limits_upper_(2) = 0.6;
        jnt_limits_upper_(3) = 0.6;
        jnt_limits_upper_(4) = 0.6;
        jnt_limits_upper_(5) = 0.6;

        table_corner_1 = KDL::Vector(-0.2, -0.15, 0.3);
        table_corner_2 = KDL::Vector(-0.9, -0.15, 0.3);
        table_corner_3 = KDL::Vector(-0.9, 0.15, 0.3);
        table_corner_4 = KDL::Vector(-0.2, 0.15, 0.3);

        table2_corner_1 = KDL::Vector(-0.1, -0.45, 0.2);
        table2_corner_2 = KDL::Vector(-0.8, -0.45, 0.2);
        table2_corner_3 = KDL::Vector(-0.8, -0.15, 0.2);
        table2_corner_4 = KDL::Vector(-0.1, -0.15, 0.2);

        table3_corner_1 = KDL::Vector(-0.1, 0.15, 0.2);
        table3_corner_2 = KDL::Vector(-0.8, 0.15, 0.2);
        table3_corner_3 = KDL::Vector(-0.8, 0.45, 0.2);
        table3_corner_4 = KDL::Vector(-0.1, 0.45, 0.2);

        interpolated_table_surface_.push_back(table_corner_1);
        interpolated_table_surface_.push_back(table_corner_2);

        interpolated_table_surface2_.push_back(table2_corner_1);
        interpolated_table_surface2_.push_back(table2_corner_2);

        interpolated_table_surface3_.push_back(table3_corner_1);
        interpolated_table_surface3_.push_back(table3_corner_2);


        int split_num = 20;
        for(int i = 1; i < split_num; i++){
            double diff12 = (table_corner_2(0) - table_corner_1(0)) * i/split_num;
            interpolated_table_surface_.push_back(KDL::Vector(table_corner_1(0) + diff12, -0.15, 0.3));

            diff12 = (table2_corner_2(0) - table2_corner_1(0)) * i/split_num;
            interpolated_table_surface2_.push_back(KDL::Vector(table2_corner_1(0) + diff12, -0.45, 0.2));

            diff12 = (table3_corner_2(0) - table3_corner_1(0)) * i/split_num;
            interpolated_table_surface3_.push_back(KDL::Vector(table3_corner_1(0) + diff12, 0.15, 0.2));
        }

        std::vector<KDL::Vector> tmp_copy = interpolated_table_surface_;
        std::vector<KDL::Vector> tmp_copy2 = interpolated_table_surface2_;
        std::vector<KDL::Vector> tmp_copy3 = interpolated_table_surface3_;

        for(int i = 1; i < (int)tmp_copy.size(); i++){
            double curr_x = tmp_copy[i](0);
            double curr_x2 = tmp_copy2[i](0);
            double curr_x3 = tmp_copy3[i](0);
            for(int j = 0; j <= split_num; j++){
                double next_y = tmp_copy[i](1) + 0.3*j/split_num;
                double next_y2 = tmp_copy2[i](1) + 0.3*j/split_num;
                double next_y3 = tmp_copy3[i](1) + 0.3*j/split_num;
                interpolated_table_surface_.push_back(KDL::Vector(curr_x, next_y, 0.3));
                interpolated_table_surface_.push_back(KDL::Vector(curr_x2, next_y2, 0.2));
                interpolated_table_surface_.push_back(KDL::Vector(curr_x3, next_y3, 0.2));
            }
        }
        std::cout << "TABLE HAS: " << interpolated_table_surface_.size() << std::endl;


        for(int i = 1; i <= 5; i++){
            KDL::JntArray arr;
            arr.data = Eigen::VectorXd::Zero(i);
            arm_segment_states_.push_back(arr);
        }
        std::cout << "JntArrays done " << std::endl;

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
     


        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Reactive Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        int segment_length = 1;
        for(int i = 0; i < 5; i++){
            for(int j = 1; j <= segment_length; j++){
                arm_segment_states_[i](j) = q_(j);
            }
            segment_length++;
        }



        std::vector<KDL::Jacobian> jacobians_;
   //     jac_solver_1->JntToJac(arm_segment_states_[0], seg_jac_1);
   //     jacobians_.push_back(seg_jac_1);

   //     jac_solver_2->JntToJac(arm_segment_states_[1], seg_jac_2);
   //     jacobians_.push_back(seg_jac_2);

   //     jac_solver_3->JntToJac(arm_segment_states_[2], seg_jac_3);
   //     jacobians_.push_back(seg_jac_3);

   //     jac_solver_4->JntToJac(arm_segment_states_[3], seg_jac_4);
   //     jacobians_.push_back(seg_jac_4);

        jac_solver_5->JntToJac(arm_segment_states_[4], seg_jac_5);
        jacobians_.push_back(seg_jac_5);
        

        KDL::Frame xd_;
        FKSolver_->JntToCart(q_, x_);
        double Q_star_table_ = 0.15;
        double Q_star_table_min_ = 0.122;

        if( (t > 3 && t < 5) ){
            Q_star_table_ -= (t-3) * 0.016;
        }
        else if( (t > 5 && t < 11.9) ){
            Q_star_table_ = Q_star_table_min_;
        }
        else if( (t >= 11.9 && t < 12.5) ){
            Q_star_table_ -= (12.5-t) * 0.5;
        }
        else if( (t >= 15 && t < 16) ){
            Q_star_table_ -= (t-15) * 0.03;
        }
        else if( (t >= 16 && t < 21.9) ){
            Q_star_table_ = Q_star_table_min_;
        }
        else if( (t >= 21.9 && t < 23) ){
            Q_star_table_ -= (23-t) * 0.272;
        }


        if( Q_star_table_ < Q_star_table_min_){
            Q_star_table_ = Q_star_table_min_;
         }





        // ********* 1. Desired Trajecoty in Joint Space *********

        for (size_t i = 0; i < n_joints_; i++)
        {
       //     qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
       //     qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);
       //     qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t);
          //  continue;
         //  
            if( t < 1 ){
                qd_(i) = 0;
            }
           // qd_(i) = 0;
            qd_ddot_(i) = 0;
            qd_dot_(i) = 0;


            // Move to first table
            if(t > 1 && t < 6){ 
                qd_(0) = -1.0;
                qd_(1) = 0;
                qd_(2) = -1.75; 
                qd_(3) = 0.01;
                qd_(4) = -1.4;
                qd_(5) = 0;
            }


            // Perform maneuver
            if(t >= 6 && t < 12){ 
                if( !task_space_ ){
                    circleDrawerAngle_ = -M_PI/2;
                    FKSolver_->JntToCart(q_, x_);
                    circle_x = x_.p(0);
                    circle_y = x_.p(1) + 0.1;
                    Ki_(0) = 400.0;
                    Ki_(1) = 400.0;
                    Ki_(2) = 400.0;
                    Ki_(3) = 400.0;
                    Ki_(4) = 400.0;
                    Ki_(5) = 400.0;
                    Kd_(0) = 40.0;
                    Kd_(1) = 40.0;
                    Kd_(2) = 40.0;
                    Kd_(3) = 40.0;
                    Kd_(4) = 40;
                    Kd_(5) = 40;
                }
                drawing_circle_ = true;
                task_space_ = true;
                FKSolver_->JntToCart(q_, x_);
                circleDrawerAngle_ += 0.0004;
                xd_.p(0) = circle_x + 0.07 * cos(circleDrawerAngle_);
                xd_.p(1) = circle_y + 0.07 * sin(circleDrawerAngle_);
                xd_.p(2) = 0.3;
                xd_.M = KDL::Rotation(KDL::Rotation::RPY(M_PI/2, 0, 0));
              //  std::cout << circleDrawerAngle_ << "  " << xd_.p(0) << " " << xd_.p(1) << std::endl;
            }

            //Move to second table
            if(t >= 12 && t < 16){ 
                if( task_space_ ){
                    task_space_ = false;
                    drawing_circle_ = false;
                    Ki_(0) = 36.0;
                    Ki_(1) = 36.0;
                    Ki_(2) = 36.0;
                    Ki_(3) = 36.0;
                    Ki_(4) = 36.0;
                    Ki_(5) = 36.0;
                    Kd_(0) = 12.0;
                    Kd_(1) = 12.0;
                    Kd_(2) = 12.0;
                    Kd_(3) = 12.0;
                    Kd_(4) = 12;
                    Kd_(5) = 12;
                }
                if( q_(0) < 0.7){
                    qd_(0) = qd_(0) + 0.00015;
                }
                else{
                    qd_(0) = 1.0;
                }
                qd_(1) = 0;
                qd_(2) = -1.75; 
                qd_(3) = 0.01;
                qd_(4) = -1.4;
                qd_(5) = 0;
            }

            // Perform maneuver
            if(t >= 16 && t < 22){ 
                if( !task_space_ ){
                    circleDrawerAngle_ = M_PI/2;
                    FKSolver_->JntToCart(q_, x_);
                    circle_x = x_.p(0) - 0.05;
                    circle_y = x_.p(1) - 0.1;
                    Ki_(0) = 400.0;
                    Ki_(1) = 400.0;
                    Ki_(2) = 400.0;
                    Ki_(3) = 400.0;
                    Ki_(4) = 400.0;
                    Ki_(5) = 400.0;
                    Kd_(0) = 40.0;
                    Kd_(1) = 40.0;
                    Kd_(2) = 40.0;
                    Kd_(3) = 40.0;
                    Kd_(4) = 40;
                    Kd_(5) = 40;
                }
                drawing_circle_ = true;
                task_space_ = true;
                FKSolver_->JntToCart(q_, x_);
                circleDrawerAngle_ -= 0.0004;
                xd_.p(0) = circle_x - 0.07 * cos(circleDrawerAngle_);
                xd_.p(1) = circle_y - 0.07 * sin(circleDrawerAngle_);
                xd_.p(2) = 0.3;
                xd_.M = KDL::Rotation(KDL::Rotation::RPY(M_PI/2, 0, 0));
           //     std::cout << circleDrawerAngle_ << "  " << xd_.p(0) << " " << xd_.p(1) << std::endl;
            }

            if(t >= 22 && t < 25){ 
                if( task_space_ ){
                    task_space_ = false;
                    drawing_circle_ = false;
                    Ki_(0) = 36.0;
                    Ki_(1) = 36.0;
                    Ki_(2) = 36.0;
                    Ki_(3) = 36.0;
                    Ki_(4) = 36.0;
                    Ki_(5) = 36.0;
                    Kd_(0) = 12.0;
                    Kd_(1) = 12.0;
                    Kd_(2) = 12.0;
                    Kd_(3) = 12.0;
                    Kd_(4) = 12;
                    Kd_(5) = 12;
                }
                if( q_(0) > 0.05){
                    qd_(0) = qd_(0) - 0.00012;
                }
                else{
                    qd_(0) = 0;
                }
                qd_(1) = 0;
                qd_(2) = -1.75; 
                qd_(3) = 0.01;
                qd_(4) = -1.4;
                qd_(5) = 0;
            }
            if( t >= 25 && t < 27 ){
                qd_(0) = 0;
                qd_(1) = 0;
                qd_(2) = 0; 
                qd_(3) = 0;
                qd_(4) = 0;
                qd_(5) = 0;
            }
            if( t >= 27 ){
                t = 0;
            }
        }

        tau_rep_ = Eigen::VectorXd::Zero(n_joints_);

        // *** 2.2.1 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 


        // Apply Joint limits as repulsive forces/torques
        KDL::JntArray diff_q_;
        Subtract(q_, qd_, diff_q_);
        Subtract(q_, jnt_limits_lower_, diff_to_low_);
        Subtract(q_, jnt_limits_upper_, diff_to_upper_);
        


        KDL::Frame x_up_lim_;
        KDL::Frame x_low_lim_;



        jnt_to_jac_solver_->JntToJac(q_, J_);



        // Calculate workspace positions for joints and middle parts of links.
        std::vector<KDL::Frame> arm_segment_states_cart_;




        KDL::Frame seg_cart_;
 //       fk_solver_1->JntToCart(arm_segment_states_[0], seg_cart_);
 //       arm_segment_states_cart_.push_back(seg_cart_);
 //       fk_solver_2->JntToCart(arm_segment_states_[1], seg_cart_);
 //       arm_segment_states_cart_.push_back(seg_cart_);
 //       fk_solver_3->JntToCart(arm_segment_states_[2], seg_cart_);
 //       arm_segment_states_cart_.push_back(seg_cart_);
 //       fk_solver_4->JntToCart(arm_segment_states_[3], seg_cart_);
 //       arm_segment_states_cart_.push_back(seg_cart_);
        fk_solver_5->JntToCart(arm_segment_states_[4], seg_cart_);
        arm_segment_states_cart_.push_back(seg_cart_);




        double closest_dist_ = 999;
        int closest_idx_ = -2;
        Eigen::Matrix<double, 6, 1> ex_table_point_ = Eigen::VectorXd::Zero(6);
        Eigen::Matrix<double, 6, 1> ex_closest_table_point_ = Eigen::VectorXd::Zero(6);
        Eigen::Matrix<double, 6, 1> F_rep_table_total_ = Eigen::VectorXd::Zero(6);


        // Include repulsion field of the table WRT end effector only.
        for(int i = 0; i < (int)interpolated_table_surface_.size(); i++){
           // break;
            KDL::Vector curr_table_point_ = interpolated_table_surface_[i];
            ex_table_point_(0) = curr_table_point_(0) - x_.p(0);
            ex_table_point_(1) = curr_table_point_(1) - x_.p(1);
            ex_table_point_(2) = curr_table_point_(2) - x_.p(2);
            table_point_distances_[i] = -ex_table_point_;

            double dist = sqrt( ex_table_point_(0)*ex_table_point_(0) + ex_table_point_(1)*ex_table_point_(1) + ex_table_point_(2)*ex_table_point_(2));
            if( dist < Q_star_table_ ){
                closest_dist_ = dist;
                closest_idx_ = i;
                ex_closest_table_point_ = -ex_table_point_;
                
                Eigen::Matrix<double, 6, 1> grad_D_table_ = ex_table_point_ - table_point_distances_prev_[i];


              //  F_rep_table_total_(0) = -0.1 * (1 / ex_table_point_(0) - 1 / Q_star_table_) * (1 / (ex_table_point_(0) * ex_table_point_(0)) * -grad_D_table_(0));
             //   F_rep_table_total_(1) = -0.1 * (1 / ex_table_point_(1) - 1 / Q_star_table_) * (1 / (ex_table_point_(1) * ex_table_point_(1)) * -grad_D_table_(1));
                F_rep_table_total_(2) = -1 * (1 / ex_table_point_(2) - 1 / Q_star_table_) * (1 / (ex_table_point_(2) * ex_table_point_(2)) * -grad_D_table_(2));
                for(int f = 0; f < 3; f++){
                    if( F_rep_table_total_(f) > 90 ){
                        F_rep_table_total_(f) = 90;
                    }
                }
                
                // UNCOMMENT THIS to include the tables repulsion forces in the arm.
               tau_rep_ += J_.data.transpose() * F_rep_table_total_;
            }
        }
        tau_rep_ -= M_.data * qdot_.data;



       // ex_table_prev_ = ex_closest_table_point_;
        table_point_distances_prev_ = table_point_distances_;
        table_point_distances_arm_prev_ = table_point_distances_arm_;

        //Eigen::Matrix<double, 6, 1> F_rep_ = F_obstacle_;
        Eigen::Matrix<double, 6, 1> F_rep_ = F_rep_table_total_;
        F_rep_(3) = 0;
        F_rep_(4) = 0;
        F_rep_(5) = 0;
       

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)


        
        // *** 2.2.2 gravity + PD as in slides ***

        if( !task_space_){
        // This can be considered as u_att
            tau_PD_grav_.data = G_.data + Kp_.data.cwiseProduct(e_.data) - Kd_.data.cwiseProduct(qdot_.data);

            KDL::JntArray tau_total_;
            tau_total_.data = tau_PD_grav_.data + tau_rep_;

           for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(tau_total_(i));
            }
        }
        
        else if( task_space_ ){

           // FKSolver_->JntToCart(qd_, xd_);

            xd_dot_task_(0) = 0;
            xd_dot_task_(1) = 0;
            xd_dot_task_(2) = 0;
            xd_dot_task_(3) = 0;
            xd_dot_task_(4) = 0;
            xd_dot_task_(5) = 0;

            xd_ddot_task_(0) = 0;
            xd_ddot_task_(1) = 0;
            xd_ddot_task_(2) = 0;
            xd_ddot_task_(3) = 0;
            xd_ddot_task_(4) = 0;
            xd_ddot_task_(5) = 0;

            xdot_task_ = J_.data * qdot_.data;
            ex_dot_task_ = xd_dot_task_ - xdot_task_;

            ex_temp_ = diff(x_, xd_);

            ex_task_(0) = ex_temp_(0);
            ex_task_(1) = ex_temp_(1);
            ex_task_(2) = ex_temp_(2);
            ex_task_(3) = ex_temp_(3);
            ex_task_(4) = ex_temp_(4);
            ex_task_(5) = ex_temp_(5);

            qdot_vel_array_.q.data = q_.data;
            qdot_vel_array_.qdot.data = qdot_.data;
            jnt_to_jac_dot_solver_->JntToJacDot(qdot_vel_array_, Jdot_);
            jnt_to_jac_dot_solver_->JntToJacDot(qdot_vel_array_, J_dot_q_dot_);


            KDL::JntArray J_dot_q_dot_array_; J_dot_q_dot_array_.resize(6);
            J_dot_q_dot_array_(0) = J_dot_q_dot_(0);
            J_dot_q_dot_array_(1) = J_dot_q_dot_(1);
            J_dot_q_dot_array_(2) = J_dot_q_dot_(2);
            J_dot_q_dot_array_(3) = J_dot_q_dot_(3); 
            J_dot_q_dot_array_(4) = J_dot_q_dot_(4); 
            J_dot_q_dot_array_(5) = J_dot_q_dot_(5); 


            aux_d_.data = M_.data * J_.data.inverse() * (xd_ddot_task_ + Kp_.data.cwiseProduct(ex_task_) + Kd_.data.cwiseProduct(ex_dot_task_) - J_dot_q_dot_array_.data);
            comp_d_.data = C_.data + G_.data;
            tau_d_.data = aux_d_.data + comp_d_.data + tau_rep_;
           for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(tau_d_(i));
            }
        }


        // *** 2.3 Apply Torque Command to Actuator ***

        // Use 'e_dot_.data = qdot_c_.data - qdot_.data' for kinematic controller. 
        // Otherwise use 'e_dot_.data = qd_dot_.data - qdot_.data'.
        //e_dot_.data = qdot_c_.data - qdot_.data;
        //aux_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data));
        //comp_d_.data = C_.data + G_.data;
        //tau_d_.data = aux_d_.data + comp_d_.data;




        // ********* 3. data 저장 *********
        save_data();

        // ********* 4. state 출력 *********
        print_state();
    }


		double trajectory_generator_pos(double dStart, double dEnd, double dDuration, double time_)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
		}

		double trajectory_generator_vel(double dStart, double dEnd, double dDuration, double time_)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
		}

		double trajectory_generator_acc(double dStart, double dEnd, double dDuration, double time_)
		{
			double dA0 = dStart;
			double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
			double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
			double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

			return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
		}




    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        j1_avg.push_back(abs(R2D * e_(0)));
        j2_avg.push_back(abs(R2D * e_(1)));
        j3_avg.push_back(abs(R2D * e_(2)));
        j4_avg.push_back(abs(R2D * e_(3)));
        j5_avg.push_back(abs(R2D * e_(4)));
        j6_avg.push_back(abs(R2D * e_(5)));

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(3);
        SaveData_[40] = e_dot_(4);
        SaveData_[41] = e_dot_(5);
        SaveData_[42] = e_dot_(6);


        j1_vel_avg.push_back(abs(R2D * e_dot_(0)));
        j2_vel_avg.push_back(abs(R2D * e_dot_(1)));
        j3_vel_avg.push_back(abs(R2D * e_dot_(2)));
        j4_vel_avg.push_back(abs(R2D * e_dot_(3)));
        j5_vel_avg.push_back(abs(R2D * e_dot_(4)));
        j6_vel_avg.push_back(abs(R2D * e_dot_(5)));

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");


            printf("*** Joint Space Vel Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_dot_(0));
            printf("%f, ", R2D * e_dot_(1));
            printf("%f, ", R2D * e_dot_(2));
            printf("%f, ", R2D * e_dot_(3));
            printf("%f, ", R2D * e_dot_(4));
            printf("%f\n", R2D * e_dot_(5));
            printf("\n");

            int num_vals = j1_vel_avg.size(); 
            double average = 0.0;
            if ( num_vals != 0) {
                average = std::accumulate( j1_vel_avg.begin(), j1_vel_avg.end(), 0.0) / num_vals; 
                std::cout << "JOINT VEL ERROR AVERAGES: " << std::to_string(average) << " - ";
                average = 0.0;
                average = std::accumulate( j2_vel_avg.begin(), j2_vel_avg.end(), 0.0) / num_vals; 
                std::cout << std::to_string(average) << " - ";
                average = 0.0;
                average = std::accumulate( j3_vel_avg.begin(), j3_vel_avg.end(), 0.0) / num_vals; 
                std::cout << std::to_string(average) << " - ";
                average = 0.0;
                average = std::accumulate( j4_vel_avg.begin(), j4_vel_avg.end(), 0.0) / num_vals; 
                std::cout << std::to_string(average) << " - ";
                average = 0.0;
                average = std::accumulate( j5_vel_avg.begin(), j5_vel_avg.end(), 0.0) / num_vals; 
                std::cout << std::to_string(average) << " - ";
                average = 0.0;
                average = std::accumulate( j6_vel_avg.begin(), j6_vel_avg.end(), 0.0) / num_vals; 
                std::cout << std::to_string(average) << std::endl;
            }

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntSpaceInertiaMatrix M_inv_;
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_, qddot_;
    KDL::JntArray e_, e_dot_, e_int_, e_ddot_;
    KDL::Jacobian J_;
    KDL::Jacobian Jdot_;
    KDL::Twist J_dot_q_dot_;
    KDL::Jacobian seg_jac_1;
    KDL::Jacobian seg_jac_2;
    KDL::Jacobian seg_jac_3;
    KDL::Jacobian seg_jac_4;
    KDL::Jacobian seg_jac_5;
    boost::scoped_ptr<KDL::ChainFkSolverPos>    FKSolver_;
    boost::scoped_ptr<KDL::ChainFkSolverVel>  FKSolver_vel_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; 
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_1;  
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_2;  
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_3;  
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_4;  
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_5; 
    boost::scoped_ptr<KDL::ChainFkSolverPos>    fk_solver_1; 
    boost::scoped_ptr<KDL::ChainFkSolverPos>    fk_solver_2; 
    boost::scoped_ptr<KDL::ChainFkSolverPos>    fk_solver_3; 
    boost::scoped_ptr<KDL::ChainFkSolverPos>    fk_solver_4; 
    boost::scoped_ptr<KDL::ChainFkSolverPos>    fk_solver_5;                     
    KDL::JntArray  xdot_;
    

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray aux_d_Kpzero_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;
    KDL::JntArray tau_PD_grav_;
    KDL::JntArray vel_control_;
    KDL::JntArray kine_control_;
    KDL::JntArray qdot_c_;
    double rotX_pos, rotY_pos, rotZ_pos, rotW_pos, rotX_des, rotY_des, rotZ_des, rotW_des;


    // Joint limits
    KDL::JntArray jnt_limits_lower_;
    KDL::JntArray jnt_limits_upper_;
    KDL::JntArray diff_to_low_;
    KDL::JntArray diff_to_upper_;
    KDL::JntArray diff_to_low_prev_;
    KDL::JntArray diff_to_upper_prev_;
    KDL::Frame obstacle_;
    std::vector<KDL::JntArray> arm_segment_states_;
   

    Eigen::Matrix<double, 6, 1> ex_obstacle_prev_;
    Eigen::Matrix<double, 3, 1> ex_table_prev_;
    KDL::Vector table_corner_1;
    KDL::Vector table_corner_2;
    KDL::Vector table_corner_3;
    KDL::Vector table_corner_4;
    KDL::Vector table2_corner_1;
    KDL::Vector table2_corner_2;
    KDL::Vector table2_corner_3;
    KDL::Vector table2_corner_4;
    KDL::Vector table3_corner_1;
    KDL::Vector table3_corner_2;
    KDL::Vector table3_corner_3;
    KDL::Vector table3_corner_4;
    std::vector<KDL::Vector> interpolated_table_surface_;
    std::vector<KDL::Vector> interpolated_table_surface1_;
    std::vector<KDL::Vector> interpolated_table_surface2_;
    std::vector<KDL::Vector> interpolated_table_surface3_;
    std::map<int, Eigen::Matrix<double, 6, 1>> table_point_distances_;
    std::map<int, Eigen::Matrix<double, 6, 1>> table_point_distances_prev_;
    std::map<int, std::map<int, Eigen::Matrix<double, 6, 1>>> table_point_distances_arm_;
    std::map<int, std::map<int, Eigen::Matrix<double, 6, 1>>> table_point_distances_arm_prev_;
    Eigen::Matrix<double ,6, 1> tau_rep_;
    std::vector<boost::scoped_ptr<KDL::ChainJntToJacSolver>> jac_seg_solvers_;


    // gains
    KDL::JntArray Kp_, Ki_, Kd_, K_kine_;
    KDL::Frame fKine_pos_Frame_;
    KDL::Frame fKine_des_Frame_;
    KDL::Twist ex_temp_;
    KDL::Frame x_;
    KDL::FrameVel fKine_vel_Frame_;
    KDL::JntArrayVel qdot_vel_array_;


    KDL::Chain kdl_chain_1;
    KDL::Chain kdl_chain_2;
    KDL::Chain kdl_chain_3;
    KDL::Chain kdl_chain_4;
    KDL::Chain kdl_chain_5;

    double circleDrawerAngle_ = 0;
    double circle_x, circle_y;
    bool task_space_ = false;
    bool drawing_circle_ = false;
    bool drawing_line_ = false;
    Eigen::Matrix<double, 6, 1> ex_task_;
    Eigen::Matrix<double, 6, 1> xd_dot_task_, xd_ddot_task_;
    Eigen::Matrix<double, 6, 1> ex_dot_task_, xdot_task_;


    // save the data
    double SaveData_[SaveDataMax];
    std::vector<double> j1_avg;
    std::vector<double> j2_avg;
    std::vector<double> j3_avg;
    std::vector<double> j4_avg;
    std::vector<double> j5_avg;
    std::vector<double> j6_avg;

    std::vector<double> j1_vel_avg;
    std::vector<double> j2_vel_avg;
    std::vector<double> j3_vel_avg;
    std::vector<double> j4_vel_avg;
    std::vector<double> j5_vel_avg;
    std::vector<double> j6_vel_avg;

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::DemoController, controller_interface::ControllerBase)
