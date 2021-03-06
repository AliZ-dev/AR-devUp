// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>

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
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/framevel.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/opencv.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49

namespace arm_controllers
{
class Visual_Servo_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
        Jacobian_q_.resize(n_joints_);
        K_kine_.resize(n_joints_);
        K_kine_.data(0) = 1.0;
        K_kine_.data(1) = 1.0;
        K_kine_.data(2) = 1.0;
        K_kine_.data(3) = 1.0;
        K_kine_.data(4) = 1.0;
        K_kine_.data(5) = 1.0;

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_ -1; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/visual_servo_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/visual_servo_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/visual_servo_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/visual_servo_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
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

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);
        xdot_ = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

       
        aruco_pose_.p(0) = 0;
        aruco_pose_.p(1) = -1.0;
        aruco_pose_.p(2) = 0.7;
        aruco_pose_.M.RPY(1.57, -1.57, 3.14);



        cam_pose_.data = Eigen::VectorXd::Zero(6);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());



        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        sub_cam_ = n.subscribe<geometry_msgs::TransformStamped>("/aruco/transforms", 1, &Visual_Servo_Controller::updateCam, this);
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
        ROS_INFO("Starting Computed Torque Controller");
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

        // ********* 1. Desired Trajecoty in Joint Space *********

        //for (size_t i = 0; i < n_joints_; i++)
       // {
            //qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); 
            //qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t);          
        //    qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t);
         //   qd_ddot_(i) = 0;
       //     qd_dot_(i) = 0;
     //   }

        if(t < 4){
            qd_(0) = 1.55;
            qd_(1) = 0.15;
            qd_(2) = -1.0;
            qd_(3) = 0.0;
            qd_(4) = -2.05;
        }



        // Camera pos
        FKSolver_->JntToCart(q_, x_);

        // *** 2.1 computing Jacobian J(q) ***
        jnt_to_jac_solver_->JntToJac(q_, J_);

        // *** 2.2 computing Jacobian transpose/inversion ***
        J_transpose_ = J_.data.transpose();
        xdot_ = J_.data * qdot_.data;
       

        KDL::Frame obj_wrt_cam_, aruco_frame_, cam_to_obj_;

     //   cam_to_obj_.M.Quaternion(rvec[0], rvec[1], rvec[2], rvec[3]);
     //   cam_to_obj_.M.SetInverse();
     //   cam_to_obj_.p(0) = -tvec[0];
     //   cam_to_obj_.p(1) = -tvec[1];
     //   cam_to_obj_.p(2) = -tvec[2];

        obj_wrt_cam_.p(0) = tvec(0);
        obj_wrt_cam_.p(1) = tvec(1);
        obj_wrt_cam_.p(2) = tvec(2);
       // obj_wrt_cam_.M.Quaternion(rvec[0], rvec[1], rvec[2], rvec[3]);
        obj_wrt_cam_.M.RPY(rvec[0], rvec[1], rvec[2]);


        double rx, ry, rz;
        cam_to_obj_ = obj_wrt_cam_.Inverse();
        cam_to_obj_.M.GetRPY(rx, ry, rz);


        std::cout << " Obj wrt cam: " << obj_wrt_cam_.p[0] << "  "  << obj_wrt_cam_.p[1] << "  "  << obj_wrt_cam_.p[2] << "  "  << rvec[0] << "  "  << rvec[1] << "  "  << rvec[2] << std::endl;

        KDL::Twist tf_des_to_cam_twist_ = diff(xd_, obj_wrt_cam_);
        KDL::Frame tf_des_to_cam_;

        tf_des_to_cam_.p(0) = tf_des_to_cam_twist_.vel(0);
        tf_des_to_cam_.p(1) = tf_des_to_cam_twist_.vel(1);
        tf_des_to_cam_.p(2) = tf_des_to_cam_twist_.vel(2);
        tf_des_to_cam_.M.RPY(tf_des_to_cam_twist_.rot(0), tf_des_to_cam_twist_.rot(1), tf_des_to_cam_twist_.rot(2));

        // DESIRED POSE 
        if(aruco_detected){
            xd_.p(0) = 0.01;
            xd_.p(1) = 0.0;
            xd_.p(2) = 0.25;
            xd_.M = KDL::Rotation(KDL::Rotation::RPY(0,0,0));
        }


        tf_des_to_cam_ = xd_ * obj_wrt_cam_.Inverse();

        KDL::Frame world_to_desired_ = x_ * tf_des_to_cam_.Inverse();

       // double rx, ry, rz;
        ex_(0) = -tf_des_to_cam_.p(0);
        ex_(1) = -tf_des_to_cam_.p(1);
        ex_(2) = -tf_des_to_cam_.p(2);
        tf_des_to_cam_.M.GetRPY(rx, ry, rz);
        ex_(3) = -rx;
        ex_(4) = -ry;
        ex_(5) = -rz;

        KDL::Chain chain;
        kdl_tree_.getChain("world", "camera", chain);



        Eigen::Matrix<double, 6, 6> T_A;       
        Eigen::Matrix<double, 6, 6> rot_transposes;
        Eigen::Matrix<double, 3, 3> R_d;
        Eigen::Matrix<double, 3, 3> R_d_T;

        R_d.setZero();
        for(int row = 0; row < 3; row++){
            for(int col = 0; col < 3; col++){
               // R_d(row, col) = tf_des_to_cam_.M(row, col);
                R_d(row, col) = world_to_desired_.M(row, col);
            }
        }
        R_d_T = R_d.transpose();



        T_A.setZero();
        for(int row = 0; row < 3; row++){
            T_A(row, row) = 1.0;
            for(int col = 0; col < 3; col++){
                T_A(row + 3, col + 3) = tf_des_to_cam_.M(row, col);
            }
        }


        rot_transposes.setZero();
     //   KDL::Rotation tf_rot_transpose = tf_des_to_cam_.M.Inverse();
        for(int row = 0; row < 3; row++){
            for(int col = 0; col < 3; col++){
                rot_transposes(row, col) = R_d_T(row, col);
                rot_transposes(row + 3, col + 3) = R_d_T(row, col);
            }
        }



        KDL::Jacobian J_Ad;
        J_Ad.data = T_A.inverse() * rot_transposes * J_.data;
       // J_Ad.data = T_A.inverse() * J_.data;


        std::cout << " Des to cam: " << tf_des_to_cam_.p[0] << "  "  << tf_des_to_cam_.p[1] << "  "  << tf_des_to_cam_.p[2] << std::endl;
        std::cout << "        ex_: " << ex_(0) << "  "  << ex_(1) << "  "  << ex_(2) << std::endl;

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)
        
        // *** 2.2.1 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 





        // *** 2.3 Apply Torque Command to Actuator ***

        // Use 'e_dot_.data = qdot_c_.data - qdot_.data' for kinematic controller. 
        // Otherwise use 'e_dot_.data = qd_dot_.data - qdot_.data'.
     //   e_dot_.data = qdot_c_.data - qdot_.data;
        //aux_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data));
       // aux_d_.data = J_transpose_ * ex_ * (Kp_.data.cwiseProduct(ex_) - Kd_.data.cwiseProduct(J_.data * ex_) * qdot_.data);
        aux_d_.data = J_Ad.data.transpose() * (Kp_.data.cwiseProduct(ex_) - Kd_.data.cwiseProduct(J_Ad.data * qdot_.data));
        comp_d_.data =  G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        // *** 2.2.2 gravity + PD as in slides ***
        if(t < 4)
            tau_d_.data = G_.data + Kp_.data.cwiseProduct(e_.data) - Kd_.data.cwiseProduct(qdot_.data);

        for (int i = 0; i < n_joints_; i++)
        {
            //joints_[i].setCommand(qdot_c_(i));
            //joints_[i].setCommand(kine_control_(i));
            //joints_[i].setCommand(vel_control_(i));
            //joints_[i].setCommand(tau_PD_grav_(i));
            joints_[i].setCommand(tau_d_(i));
            // joints_[i].setCommand(0.0);
        }

        // ********* 3. data 저장 *********
        save_data();

        // ********* 4. state 출력 *********
        print_state();
    }

    void stopping(const ros::Time &time)
    {
    }

    void updateCam(const geometry_msgs::TransformStamped c_pose)
    {
      if( !aruco_detected){
        double Kp_pbvs = 60;
        double Kd_pbvs = 22;
        Kp_(0) = Kp_pbvs;
        Kp_(1) = Kp_pbvs;
        Kp_(2) = Kp_pbvs;
        Kp_(3) = Kp_pbvs;
        Kp_(4) = Kp_pbvs;
        Kp_(5) = Kp_pbvs;
        Kd_(0) = Kd_pbvs;
        Kd_(1) = Kd_pbvs;
        Kd_(2) = Kd_pbvs;
        Kd_(3) = Kd_pbvs;
        Kd_(4) = Kd_pbvs;
        Kd_(5) = Kd_pbvs;
      }

        aruco_detected = true;
        tvec[0] = c_pose.transform.translation.x;
        tvec[1] = c_pose.transform.translation.y;
        tvec[2] = c_pose.transform.translation.z;
        rvec[0] = c_pose.transform.rotation.x;
        rvec[1] = c_pose.transform.rotation.y;
        rvec[2] = c_pose.transform.rotation.z;
        rvec[3] = c_pose.transform.rotation.w;
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
    KDL::Jacobian Jacobian_q_;
    KDL::Jacobian J_;
    Eigen::Matrix<double, 6, 6> J_transpose_;
    boost::scoped_ptr<KDL::ChainFkSolverVel>  FKSolver_vel_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;                      
    KDL::JntArray cam_pose_;
    KDL::Frame aruco_pose_;
    cv::Vec3d rvec, tvec;
    cv::Mat R; //holds rotation matrix
    KDL::Frame frame_cam;


    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;
    Eigen::Matrix<double, 6, 1> ex_;
    Eigen::Matrix<double, 6, 1> xdot_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> FKSolver_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray aux_d_test_;
    KDL::JntArray aux_d_Kpzero_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;
    KDL::JntArray tau_PD_grav_;
    KDL::JntArray vel_control_;
    KDL::JntArray kine_control_;
    KDL::JntArray qdot_c_;
    double rotX_pos, rotY_pos, rotZ_pos, rotW_pos, rotX_des, rotY_des, rotZ_des, rotW_des;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_, K_kine_;
    KDL::Frame fKine_pos_Frame_;
    KDL::Frame fKine_des_Frame_;
    KDL::FrameVel fKine_vel_Frame_;
    KDL::JntArrayVel qdot_vel_array_;


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
    
    ros::Subscriber sub_cam_;

    bool aruco_detected = false;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::Visual_Servo_Controller, controller_interface::ControllerBase)
