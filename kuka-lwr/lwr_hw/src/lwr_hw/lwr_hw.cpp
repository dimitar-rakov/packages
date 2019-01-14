#include "lwr_hw/lwr_hw.h"


namespace lwr_hw
{
void LWRHW::create(std::string robot_namespace, std::string urdf_path)
{
    std::cout << "Creating a KUKA LWR 4 called: " << robot_namespace << std::endl;

    // SET NAME AND MODEL
    robot_namespace_ = robot_namespace;
    urdf_path_ = urdf_path;

    // ALLOCATE MEMORY

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    for (size_t j =0 ; j< join_names_sufixes_.size(); j++)
      joint_names_.push_back( robot_namespace_ + join_names_sufixes_[j] );


    // VARIABLES
    joint_msr_position_.resize(n_joints_);
    joint_msr_position_prev_.resize(n_joints_);
    joint_msr_velocity_.resize(n_joints_);
    joint_msr_velocity_prev_.resize(n_joints_);
    joint_msr_effort_.resize(n_joints_);
    joint_cmd_position_.resize(n_joints_);
    joint_cmd_velocity_.resize(n_joints_);
    joint_cmd_effort_.resize(n_joints_);
    joint_cmd_stiffness_.resize(n_joints_);
    joint_cmd_damping_.resize(n_joints_);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);
    joint_velocity_limits_.resize(n_joints_);
    joint_effort_limits_.resize(n_joints_);
    joint_acceleration_limits_.resize(n_joints_);
    joint_jerk_limits_.resize(n_joints_);


    // RESET VARIABLES
    reset();

    std::cout << "Parsing transmissions from the URDF... \n";

    // GET TRANSMISSIONS THAT BELONG TO THIS LWR 4 ARM
    if (!parseTransmissionsFromURDF(urdf_path_))
    {
        std::cout << robot_namespace_ <<": "<< "Error parsing URDF in lwr_hw.\n\n";
        return;
    }

    if (urdf_model_ptr_->initString(urdf_path_))
    {
        std::cout << robot_namespace_ <<": "<< "Error initialising model in lwr_hw.\n\n";
        return;
    }

    std::cout << "Registering interfaces...\n";
    registerInterfaces(urdf_model_ptr_.get(), *transmissions_);

    std::cout << "Initializing KDL variables...\n";

    // INIT KDL STUFF
    initKDLdescription(urdf_model_ptr_.get());

    std::cout << "Succesfully created an abstract LWR 4 ARM with interfaces to ROS control\n";
}

// reset values
void LWRHW::reset()
{
    for (size_t j = 0; j < n_joints_; ++j)
    {
        joint_msr_position_[j] = 0.0;
        joint_msr_position_prev_[j] = 0.0;
        joint_msr_velocity_[j] = 0.0;
        joint_msr_velocity_prev_[j] = 0.0;
        joint_msr_effort_[j] = 0.0;
        joint_cmd_position_[j] = 0.0;
        joint_cmd_velocity_[j] = 0.0;
        joint_cmd_effort_[j] = 0.0;
        joint_cmd_stiffness_[j] = 100.0;
        joint_cmd_damping_[j] = 0.0;
    }
    current_strategy_ = JOINT_POSITION;
}

void LWRHW::registerInterfaces(const urdf::Model *const urdf_model_ptr,
                               std::vector<transmission_interface::TransmissionInfo> transmissions)
{

    // Check that this transmission has one joint
    if( transmissions.empty())
    {
        std::cout << "lwr_hw: " << "There are no transmission in this robot, all are non-driven joints?\n";
        return;
    }

    // Initialize values
    for(size_t j=0; j < n_joints_; j++)
    {
        // Check that this transmission has one joint
        if(transmissions[j].joints_.size() == 0)
        {
            std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_ << " has no associated joints.\n";
            continue;
        }
        else if(transmissions[j].joints_.size() > 1)
        {
            std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_
                      << " has more than one joint, and they can't be controlled simultaneously.\n";
            continue;
        }

        std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

        if( joint_interfaces.empty() )
        {
            std::cout << "lwr_hw: " << "Joint " << transmissions[j].joints_[0].name_ <<
                         " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
                         "You need to, otherwise the joint can't be controlled.\n";
            continue;
        }

        const std::string& hardware_interface = joint_interfaces.front();

        // Debug
        std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j]
                     << "' of type '" << hardware_interface << "'" << "\x1B[0m\n";

        // Create joint state interface for all joints
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                                                joint_names_[j],
                                                &joint_msr_position_[j],
                                                &joint_msr_velocity_[j],
                                                &joint_msr_effort_[j]));

        // Prepare and register the Position command interface
        hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
                                                                  joint_state_interface_.getHandle(joint_names_[j]),
                                                                  &joint_cmd_position_[j]);
        position_interface_.registerHandle(joint_handle_position);

        // Prepare and register the Effort command interface
        hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
                                                                joint_state_interface_.getHandle(joint_names_[j]),
                                                                &joint_cmd_effort_[j]);
        effort_interface_.registerHandle(joint_handle_effort);

        // Prepare and register the Joint impedance command handle
        hardware_interface::JointImpedanceHandle joint_handle_impedance = hardware_interface::JointImpedanceHandle(
                                                                            joint_state_interface_.getHandle(joint_names_[j]),
                                                                            &joint_cmd_position_[j],
                                                                            &joint_cmd_stiffness_[j],
                                                                            &joint_cmd_damping_[j],
                                                                            &joint_cmd_effort_[j]);
        impedance_interface_.registerHandle(joint_handle_impedance);


        // velocity command handle, recall it is fake, there is no actual velocity interface
        hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
                                                                  joint_state_interface_.getHandle(joint_names_[j]),
                                                                  &joint_cmd_velocity_[j]);


        registerJointLimits( joint_names_[j], joint_handle_position, joint_handle_effort, joint_handle_velocity,
                             urdf_model_ptr, joint_lower_limits_[j], joint_upper_limits_[j], joint_velocity_limits_[j],
                             joint_acceleration_limits_[j], joint_effort_limits_[j], joint_jerk_limits_[j]);
    }

    // Register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_interface_);
    registerInterface(&impedance_interface_);
}

// Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
// retrieved from the urdf_model.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void LWRHW::registerJointLimits(const std::string& joint_name,
                                const hardware_interface::JointHandle& joint_handle_position,
                                const hardware_interface::JointHandle& joint_handle_effort,
                                const hardware_interface::JointHandle& joint_handle_velocity,
                                const urdf::Model *const urdf_model_ptr,
                                double &joint_lower_limit, double &joint_upper_limit,
                                double &velocity_limit, double &effort_limit,
                                double &acceleration_limit, double &jerk_limit)
{
    joint_lower_limit = -std::numeric_limits<double>::max();
    joint_upper_limit = std::numeric_limits<double>::max();
    velocity_limit = std::numeric_limits<double>::max();
    acceleration_limit = std::numeric_limits<double>::max();
    jerk_limit = std::numeric_limits<double>::max();
    effort_limit = std::numeric_limits<double>::max();
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;


    if (urdf_model_ptr == nullptr){
      std::cout << "lwr_hw: " << "Pointer to urdf model was a nullptr.\n";
      return;
    }

    if (urdf_model_ptr->getJoint(joint_name) == nullptr){
      std::cout << "lwr_hw: " << "Pointer to joint " << joint_name << " was a nullptr.\n";
      return;
    }

      // Get limits from the URDF file.
      bool has_limits = joint_limits_interface::getJointLimits(urdf_model_ptr->getJoint(joint_name), limits);
      bool has_soft_limits = joint_limits_interface::getSoftJointLimits(urdf_model_ptr->getJoint(joint_name), soft_limits);


    if (!has_limits)
        return;

    joint_lower_limit = (limits.has_position_limits)? limits.min_position : joint_lower_limit;
    joint_upper_limit = (limits.has_position_limits)? limits.max_position : joint_upper_limit;
    velocity_limit = (limits.has_velocity_limits)? limits.max_velocity : velocity_limit;
    acceleration_limit = (limits.has_acceleration_limits)? limits.max_acceleration : acceleration_limit;
    jerk_limit = (limits.has_jerk_limits)? limits.max_jerk : jerk_limit;
    effort_limit = (limits.has_effort_limits)? limits.max_effort : effort_limit;

    if (has_soft_limits)
    {
        const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle_effort(joint_handle_effort, limits, soft_limits);
        ej_limits_interface_.registerHandle(limits_handle_effort);
        const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle_position(joint_handle_position, limits, soft_limits);
        pj_limits_interface_.registerHandle(limits_handle_position);
        const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle_velocity(joint_handle_velocity, limits, soft_limits);
        vj_limits_interface_.registerHandle(limits_handle_velocity);

    }
    else
    {
        const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, limits);
        ej_sat_interface_.registerHandle(sat_handle_effort);
        const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, limits);
        pj_sat_interface_.registerHandle(sat_handle_position);
        const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, limits);
        vj_sat_interface_.registerHandle(sat_handle_velocity);
    }
}

void LWRHW::enforceLimits(const ros::Duration &period)
{
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
}

bool LWRHW::parseTransmissionsFromURDF(const std::string& urdf_path)
{
    std::vector<transmission_interface::TransmissionInfo> transmissions;

    // Only *standard* transmission_interface are parsed
    transmission_interface::TransmissionParser::parse(urdf_path, transmissions);

    // Now iterate and save only transmission from this robot
    for (size_t j = 0; j < n_joints_; ++j)
    {
        for(auto transmition : transmissions)
        {
            if (joint_names_[j].compare(transmition.joints_[0].name_) == 0)
                transmissions_->push_back(transmition);
        }
    }
    return !transmissions_->empty() ;
}

bool LWRHW::initKDLdescription(const urdf::Model *const urdf_model)
{
    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        std::cout <<"Failed to construct kdl tree.\n";
        return false;
    }

    std::cout << "LWR kinematic successfully parsed with " << kdl_tree.getNrOfJoints()
              << " joints, and " << kdl_tree.getNrOfJoints()<< " segments.\n";

    // Get the info from parameters
    std::string root_name;
    ros::param::get("/" + robot_namespace_ + "/root", root_name);
    if( root_name.empty() )
        root_name = kdl_tree.getRootSegment()->first; // default
    
    std::string tip_name;
    ros::param::get("/" + robot_namespace_ + "/tip", tip_name);
    if( root_name.empty() )
        tip_name = robot_namespace_ + join_names_sufixes_.back(); ; // default

    std::cout << "Using root: " << root_name << " and tip: " << tip_name << ".\n";

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    // d.r. in current ICS setup the vector is pointing up on negative x, however here as default is given on negative z direction
    gravity_vec_ = KDL::Vector(0, 0, -9.80665016);

    // take the value from the parameter server
    if (   ros::param::get("/" + robot_namespace_ + "/gravity_wrt_robot_base/x", gravity_vec_(0))
           && ros::param::get("/" + robot_namespace_ + "/gravity_wrt_robot_base/y", gravity_vec_(1))
           && ros::param::get("/" + robot_namespace_ + "/gravity_wrt_robot_base/z", gravity_vec_(2)))
    {
        std::cout<<"Gravity vector w.r.t robot base is: "
                   "x: "<<gravity_vec_(0)<< "y: "<<gravity_vec_(1)<< "z: "<<gravity_vec_(2)<<".\n";
    }
    else
    {
      std::cout <<"Default gravity vector w.r.t robot base is set: "
                  "x: "<<gravity_vec_(0)<< "y: "<<gravity_vec_(1)<< "z: "<<gravity_vec_(2)<<".\n";
    }
    // Extract the chain from the tree
    if(!kdl_tree.getChain(root_name, tip_name, *lwr_chain_))
    {
        std::cout << "Failed to get KDL chain from tree.\n";
        return false;
    }

    std::cout <<"Number of segments:" << lwr_chain_->getNrOfSegments()<<".\n";
    std::cout <<"Number of joints in chain: " << lwr_chain_->getNrOfJoints()<<".\n";

    f_dyn_solver_.reset(new KDL::ChainDynParam(*lwr_chain_,gravity_vec_));
    joint_position_kdl_ = KDL::JntArray(lwr_chain_->getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_->getNrOfJoints());
    return true;
}

bool LWRHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                          const std::list<hardware_interface::ControllerInfo> &stop_list) const
   {
       int counter_position = 0;
       int counter_effort = 0;
       int counter_joint_impedance = 0;
       int counter_cartesian = 0;

       for ( const auto &ctrl_info : start_list)
       {
           for(const auto & interface_resource : ctrl_info.claimed_resources)
           {
               // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
               if(interface_resource.hardware_interface.compare("hardware_interface::VelocityJointInterface") == 0)
               {
                   std::cout << "The given controllers to start work on a velocity joint interface, and this robot "
                                "does not have such an interface. "<< "The switch can't be done \n";
                   return false;
               }
               if( interface_resource.hardware_interface.compare("hardware_interface::PositionJointInterface") == 0)
                   counter_position = 1;
               else if( interface_resource.hardware_interface.compare("hardware_interface::EffortJointInterface") == 0)
                   counter_effort = 1;
               else if( interface_resource.hardware_interface.compare("hardware_interface::ImpedanceJointInterface") == 0)
                   counter_joint_impedance= 1;
               else if( interface_resource.hardware_interface.compare("hardware_interface::PositionCartesianInterface") == 0)
                   counter_cartesian = 1;
           }
       }

       if( (counter_position+counter_effort+counter_cartesian + counter_joint_impedance)>1)
       {
           std::cout << "OOPS! Currently we are using the JointCommandInterface to switch mode,"
                        "this is not strictly correct. This is temporary until a joint_mode_controller"
                        "is available (so you can have different interfaces available in different modes)"
                        "Having said this, we do not support more than one controller that ones "
                        "to act on any given JointCommandInterface and we can't switch.\n";
           return false;
       }

       return true;
}

void LWRHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                     const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    // at this point, we now that there is only one controller that ones to command joints
    ControlStrategy desired_strategy = getNewControlStrategy(start_list,stop_list,desired_strategy);

    for (size_t j = 0; j < n_joints_; ++j)
    {
      ///semantic Zero
      joint_cmd_position_[j] = joint_msr_position_[j];
      joint_cmd_effort_[j] = 0.0;

      ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
      try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_cmd_position_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}
      try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_cmd_effort_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}

      ///reset joint_limit_interfaces
      pj_sat_interface_.reset();
      pj_limits_interface_.reset();
    }

    if(desired_strategy == getControlStrategy())
      std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << "\n";
    else
    {
      setControlStrategy(desired_strategy);
      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << "\n";

    }
  }

LWRHW::ControlStrategy LWRHW::getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list,
                                                    const std::list< hardware_interface::ControllerInfo >& stop_list,
                                                    LWRHW::ControlStrategy default_control_strategy)
    {
        ControlStrategy desired_strategy = default_control_strategy;
        // NOTE that this allows to switch only based on the strategy of the first controller, but ROS kinetic would allow multiple interfaces and more
        bool strategy_found = false;

        for ( const auto &ctrl_info : start_list)
        {
            for( const auto & interface_resource : ctrl_info.claimed_resources)
            {
                if( interface_resource.hardware_interface.compare("hardware_interface::PositionJointInterface") == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION).\n";
                    desired_strategy = JOINT_POSITION;
                    strategy_found = true;
                    break;
                }
                else if( interface_resource.hardware_interface.compare("hardware_interface::ImpedanceJointInterface") == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::ImpedanceJointInterface (JOINT_IMPEDANCE).\n";
                    desired_strategy = JOINT_IMPEDANCE;
                    strategy_found = true;
                    break;
                }
                else if( interface_resource.hardware_interface.compare("hardware_interface::EffortJointInterface") == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::EffortJointInterface (JOINT_EFFORT).\n";
                    desired_strategy = JOINT_EFFORT;
                    strategy_found = true;
                    break;
                }
                else if( interface_resource.hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::PositionCartesianInterface (CARTESIAN_IMPEDANCE).\n";
                    desired_strategy = CARTESIAN_IMPEDANCE;
                    strategy_found = true;
                    break;
                }
            }
            if(strategy_found)
                break;
        }

        return desired_strategy;
    }
}
