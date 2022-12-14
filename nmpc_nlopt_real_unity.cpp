#include <nmpc_nlopt_sc_franka.h>
#include <chrono>

NmpcNlopt::NmpcNlopt(ros::NodeHandle nh):MoveitTool(nh)
{
 // string node_name = ros::this_node::getName();
  bool param_ok=true;
  double Q1v,Q2v,Pf1v,Pf2v,Rv;
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Q1v",Q1v))
  {
    ROS_ERROR("error loading  Q1v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Q2v",Q2v))
  {
    ROS_ERROR("error loading  Q2v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Pf1v",Pf1v))
  {
    ROS_ERROR("error loading  Pf1v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Pf2v",Pf2v))
  {
    ROS_ERROR("error loading  Pf2v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Rv",Rv))
  {
    ROS_ERROR("error loading  Rv param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/ph",ph))
  {
    ROS_ERROR("error loading  prediction horizon param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/ch",ch))
  {
    ROS_ERROR("error loading  control horizon param");
     param_ok = false;
  }

  int maxeval;
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/max_eval_num",maxeval))
  {
    ROS_ERROR("error loading  max_eval_num");
     param_ok = false;
  }

  // outout filename parameter
//  string filename1;
//  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/filename1",filename1))
//  {
//    ROS_ERROR("error loading  filename1");
//     param_ok = false;
//  }


  // parameter: number of threads
  if(!nh.getParam("nlopt_mpc_main_node/multi_threading/num_cost_threads",num_cost_threads))
  {
    ROS_ERROR("error loading  num_cost_threads");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/multi_threading/num_cnt_threads",num_cnt_threads))
  {
    ROS_ERROR("error loading  num_cnt_threads");
     param_ok = false;
  }


  if(!nh.getParam("nlopt_mpc_main_node/plot_obstacle",plot_obstacle))
  {
    ROS_ERROR("error loading  plot_obstacle");
     param_ok = false;
  } 
  if(!param_ok)
  {
    ROS_ERROR("Param not ok , ROS Shutdown ***********");
    ros::shutdown();
  }
  
  Vector6d diag1;
  diag1 << Q1v,Q1v,Q1v,Q2v,Q2v,Q2v;
  Q = diag1.asDiagonal();
  diag1 << Pf1v,Pf1v,Pf1v,Pf2v,Pf2v,Pf2v;
  Pf = diag1.asDiagonal();
  Vector7d diag2;
  diag2 << Rv,Rv,Rv,Rv,Rv,Rv,0.1*Rv;
  R = diag2.asDiagonal();
  
  curr_joint_values.resize(joint_num);
  last_joint_values.resize(joint_num);
  curr_joint_vels.resize(joint_num);

  ROS_WARN("Unity Sim Param Done");
  //ROS_WARN_STREAM("Path " << "/movo/"+arm_name_space+"_arm_controller/state");

  joint_state_sub = nh.subscribe("/franka_ros_interface/custom_franka_state_controller/joint_states", 10, &NmpcNlopt::jointStateSubCB_sim,this); 
  //movo2kinova_offset = {0,3.1416,0,3.1416,0,3.1416,0};
  //ROS_WARN("-------------GET STARTED");
  state_sub_started = false;
  goal_sub_started = false;
 
  obstacleTable_shape = std::make_shared<shapes::Box>(0.375,0.375,1);

  goal_sub = nh.subscribe("/nmpc_controller/in/goal",1,&NmpcNlopt::goalSubCBFromOut,this);

  velocity_pub = nh.advertise<franka_core_msgs::JointCommand>("/franka_ros_interface/motion_controller/arm/joint_commands",1);

  //string this_arm_base = arm_name_space;
  //obstacle_marker.header.frame_id = this_arm_base;
  
  goal_msg.names = {"panda_link1","panda_link2","panda_link3","panda_link4",
                         "panda_link5","panda_link6","panda_hand"};
  cylinders_added = false;
  
  self_cylinders_pub = nh.advertise<perception_msgs::Cylinders>("/nmpc_controller/out/cylinder_poses",1);

  eef_pose_pub = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/out/eef_pose",1);

  // server for toggle tracking mode
  //track_toggle_server = nh.advertiseService("/panda_simulator/track_toggle",&NmpcNlopt::track_toggle_cb,this);
  track_mode = true;

  
  Ts = 0.2;
  interval = ros::Duration(Ts);

  joint_velocities_mutex = PTHREAD_MUTEX_INITIALIZER;
  position_goal_mutex = PTHREAD_MUTEX_INITIALIZER;

  // initialize optmizer
  optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);
  

// add upper and lower bounds;
  lb.resize(ch*7);
  ub.resize(ch*7);
   // set objective cost function to minimize
  optimizer.set_min_objective(costFuncMultiThread,this);
  optimizer.set_ftol_rel(1e-6);
  optimizer.set_maxeval(maxeval);


  u.resize(ch*7);
  std::fill(u.begin(),u.end(),0);

  // set joint_velocities[0] to -200 to represent that controller has not generate velocity command and
  // velocity sending thread should not send command to robot.
  joint_velocities.resize(7);
  joint_velocities[0] = -200;

  position_goal.resize(joint_num);
//  std::fill(joint_velocities.begin(),joint_velocities.end(),0);
  goal_position << 0.4,0,0.55,0,1,0,0;
   // set exitFlag to false first
  exitFlag=false;
  new_goal_got = false;
  
  // start the thread
  //thread1 = std::thread(velocitiesSend_thread_sim,&curr_joint_values, velocity_pub, &exitFlag, &joint_velocities, &goal_msg, &joint_velocities_mutex, &position_goal_mutex,&new_goal_got, &position_goal);
  thread1 = std::thread(velocitiesSend_thread_real, velocity_pub, &exitFlag, &joint_velocities,  &goal_msg, &joint_velocities_mutex,&new_goal_got);

  num_cost_loops = ph/num_cost_threads;
  last_num_cost_loops = num_cost_loops+ph%num_cost_threads;
  num_cnt_loops = ph/num_cnt_threads;
  last_num_cnt_loops = num_cnt_loops+ph%num_cnt_threads;

  // threads input
  u_ptr = new double[ch*7];
  jnt_vals_ptr = new Vector7d[ph];

  // cost calculation threads preparetion
  costCal_jnt_vals_ptr = new Vector7d[ph];
  partial_costs_ptr = new double[num_cost_threads];
  partial_grads_ptr = new double*[num_cost_threads];
  cost_activated = false;
  cost_finished_ptr = new std::atomic_bool[num_cost_threads];
  cost_calculate_grad = false;
  cost_threads_ptr = new std::thread[num_cost_threads];

  for(int i=0;i<num_cost_threads;i++)
  {
 	partial_grads_ptr[i] = new double[ch*7];
 	cost_finished_ptr[i] = true;
 	cost_threads_ptr[i] = std::thread(costCalThread, i, this, u_ptr);
 	ros::Duration(0.1).sleep();
  }


  // constraint calculation threads preparetion
  activated = false;
  calculate_grad = false;
  finished_ptr = new std::atomic_bool[num_cnt_threads];
  threads_ptr = new std::thread[num_cnt_threads];
  for(int i=0;i<num_cnt_threads;i++)
  {
    finished_ptr[i] = true;
    threads_ptr[i] = std::thread(cntCalThread, i, this);
    ros::Duration(0.1).sleep();
  }
  
  // shared in-output constainer for constraint cal thread INITIALIZATION
  partial_cnt_val.resize(num_cnt_threads);
  partial_cnt_grads.resize(num_cnt_threads);
  MoveitTools_array.resize(num_cnt_threads);
  for(int i=0;i<num_cnt_threads;i++)
    MoveitTools_array[i] = new MoveitTool(nh);
  //ROS_WARN("--------------");
	
  cal_cost_count = 0;
	time_avg = 0;

  // initialize marker for visualizing predicted trajectory
 
  
}


NmpcNlopt::~NmpcNlopt()
{
  // stop the velocities sending thread
  exitFlag = true;
  thread1.join();
 for(int i=0;i<num_cost_threads;i++)
 {
 	cost_threads_ptr[i].join();
 	delete []partial_grads_ptr[i];
 }
 for(int i=0;i<num_cnt_threads;i++)
 {
 	threads_ptr[i].join();
 	delete MoveitTools_array[i];
 }
 delete []partial_grads_ptr;
 
 delete []threads_ptr;
 delete []cost_threads_ptr;

  // delete arraies
  delete []u_ptr;
  delete []jnt_vals_ptr;
  delete []costCal_jnt_vals_ptr;
  delete []partial_costs_ptr;
  delete []finished_ptr;
  delete []cost_finished_ptr;
  
  // Eigen::Vector3d diff_position = goal_position.head<3>() - prev_point_x_position;
  // double w1=curr_point_quat(0),w2=goal_position(3);
  // Eigen::Vector3d q1=curr_point_quat.tail<3>(),q2=goal_position.tail<3>();
  // Eigen::Matrix3d skew_q2;
  // skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
  // Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
  // std::cout << "squared position error: " << diff_position.squaredNorm() << std::endl;
  // std::cout << "squared quaternion error: " << diff_orientation.squaredNorm() << std::endl;

  // end the data file and close the handle;
 
}
void NmpcNlopt::jointStateSubCB_sim(const sensor_msgs::JointState::ConstPtr& msg)
{
  double uplimit[7] = {2.87, 1.74, 2.87, -0.06, 2.87, 3.6, 2.87};
  double lowlimit[7] = {-2.87, -1.74, -2.87, -3.05, -2.87, -0.014, -2.87};
  //u_up=(up/lowlimit-msg->position)/interval.toSec()
  //std::fill(lb.begin(),lb.end(),-0.34);
  //std::fill(ub.begin(),ub.end(),0.34);
  //for(int i=0;i<ch;i++)
  //{
   // lb[7*i+6] = -0.8;
    //ub[7*i+6] = 0.8;
 // }
  //std::cout << msg<<std::endl;
  for (int i=0;i<ch;i++){
    for (int j=0;j<7;j++){
      //std::cout << msg->position[j] << std::endl;
      if ((uplimit[j]-msg->position[j])/0.2>0.34){
        ub[7*i+j] = 0.34;
        }
      else {
        ub[7*i+j] = (uplimit[j]-msg->position[j])/0.2;
      }
      if ((lowlimit[j]-msg->position[j])/0.2<-0.34){
        lb[7*i+j] = -0.34;
      }
      else {
        lb[7*i+j] = (lowlimit[j]-msg->position[j])/0.2;
      }
      if (u[7*i+j] > ub[7*i+j] || u[7*i+j] < lb[7*i+j]){
        u[7*i+j] = (ub[7*i+j]+lb[7*i+j])/2;
      }
    }
  }
  
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);
  //simple_optimizer.set_lower_bounds(lb);
  //simple_optimizer.set_upper_bounds(ub);
  if(!state_sub_started)
  {
    state_sub_started = true;
    for(int i=0;i<joint_num;i++)
    {
      //ROS_INFO("-------------------Joint state received------------------");
      //std::cout << msg->position[i]<<std::endl;
      curr_joint_values[i] = msg->position[i];
      last_joint_values[i] = curr_joint_values[i];
      curr_joint_vels[i] = 0;
    }
    
                  

  }
  else
  {
    for(int i=0;i<joint_num;i++)
    {
      last_joint_values[i] = curr_joint_values[i];
      curr_joint_values[i] = msg->position[i];
      curr_joint_vels[i] = (curr_joint_values[i] - last_joint_values[i])/interval.toSec();//calculate joint velocity
    }
  }
}

void NmpcNlopt::goalSubCBFromOut(const geometry_msgs::PoseConstPtr& msg)
{
  goal_position << msg->position.x,msg->position.y,msg->position.z,
                   msg->orientation.w,msg->orientation.x,
                   msg->orientation.y,msg->orientation.z;

  transformStamped_goal.header.stamp = ros::Time::now();
  transformStamped_goal.transform.translation.x = msg->position.x;
  transformStamped_goal.transform.translation.y = msg->position.y;
  transformStamped_goal.transform.translation.z = msg->position.z;

  transformStamped_goal.transform.rotation.x = msg->orientation.x;
  transformStamped_goal.transform.rotation.y = msg->orientation.y;
  transformStamped_goal.transform.rotation.z = msg->orientation.z;
  transformStamped_goal.transform.rotation.w = msg->orientation.w;
  br.sendTransform(transformStamped_goal);
  std::cout << goal_position<<std::endl;
  ROS_WARN("Goal received");
  if(!goal_sub_started)
    goal_sub_started = true;
}



void NmpcNlopt::octomap_sub_cb(const octomap_msgs::OctomapConstPtr octo_msg)
{
  ROS_INFO("octomap received.");
  const auto tree_ptr = octomap_msgs::binaryMsgToMap(*octo_msg);
  octree_shape = std::make_shared<shapes::OcTree>(std::shared_ptr<const octomap::OcTree>(
                                                    dynamic_cast<octomap::OcTree*>(tree_ptr)
                                                    ));
  Eigen::Affine3d octree_pose;
  octree_pose.setIdentity();
  addObstacle("shelf",octree_shape,octree_pose);
}



void NmpcNlopt::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform,const bool plot)
{
  // rviz marker doesn't support octree, do not visualize octree objects
  if (obj_shape->type==shapes::OCTREE)
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform);
  else
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform,true);
 for(int i=0;i<num_cnt_threads;i++)
   MoveitTools_array[i]->addObstacle(obj_name,obj_shape,obj_transform);

  int m = ph*(3*getObstacleNum()+1);
//  int m = ph*(3*getObstacleNum());
  std::vector<double> tol(m,1e-2);
  optimizer.remove_inequality_constraints();
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);
  
}

void NmpcNlopt::updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform, const bool plot)
{
  MoveitTool::updateObstacle(obj_name,obj_transform,true);
  for(int i=0;i<num_cnt_threads;i++)
   MoveitTools_array[i]->updateObstacle(obj_name,obj_transform);
}

void NmpcNlopt::costCalThread(const int thread_index, NmpcNlopt *class_ptr, const double* u_ptr)
{
  std::cout << "Cost cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->cost_activated && !(class_ptr->cost_finished_ptr[thread_index]))
    {
//      std::cout << "thread[" << thread_index << "] cal started." << std::endl;
      // auto t1 = std::chrono::high_resolution_clock::now();
      //cost and grad calculation
      const Matrix6d& Q = class_ptr->Q;
      const Matrix6d& Pf = class_ptr->Pf;
      const Matrix7d& R = class_ptr->R;
      const double& Ts = class_ptr->Ts;
      const int& ph = class_ptr->ph;
      const int& ch = class_ptr->ch;

      // output partial cost value container
      double& cost_running = class_ptr->partial_costs_ptr[thread_index];
      // output partial cost gradient container
      double* grad = class_ptr->partial_grads_ptr[thread_index];

      cost_running = 0;
      if(class_ptr->cost_calculate_grad)
      {
        for(int i=0;i<ch*7;i++)
          grad[i]=0;
      }
      Vector7d eef_position, u_eigen;
      Vector6d diff_running;
      const Vector7d& goal_position = class_ptr->goal_position;
      std::vector<double> curr_x(7);
      Eigen::Matrix3d eef_rotm;
      Eigen::Vector3d eef_translation;
      Eigen::Vector4d eef_quat;

      int local_num_loops = (thread_index==(class_ptr->num_cost_threads-1))?class_ptr->last_num_cost_loops:class_ptr->num_cost_loops;
      for(int i=0;i<local_num_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_cost_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->costCal_jnt_vals_ptr[global_i][j];
//        class_ptr->jaco_hess_cal.getPosition(curr_x, eef_position);
//        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_position.head<3>();
//        double w1=eef_position(3),w2=goal_position(3);
//        Eigen::Vector3d q1=eef_position.tail<3>(),q2=goal_position.tail<3>();
//        Eigen::Matrix3d skew_q2;
//        skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
//        Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
//        diff_running << diff_position,diff_orientation;

        class_ptr->jaco_hess_cal.getEEFTransform(curr_x, eef_rotm, eef_translation);
        class_ptr->jaco_hess_cal.rotm2quat(eef_rotm,eef_quat);
        //eef_translation[2]-=0.1;
        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_translation;
        double w1=eef_quat(0),w2=goal_position(3);
        Eigen::Vector3d q1=eef_quat.tail<3>(),q2=goal_position.tail<3>();
        Eigen::Matrix3d skew_q2;
        skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
        Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
        //Eigen::Vector3d diff_orientation = w2*q1-w1*q2-skew_q2*q1;  // accoring to paper, this one is correct
        diff_running << diff_position,diff_orientation;

        
        Eigen::Matrix<double,1,6> diffTQPf;
        Matrix6d QPf;
        if(global_i==ph-1)
        {
          diffTQPf = diff_running.transpose()*Pf;
          QPf = Pf;
        }
        else
        {
          diffTQPf = diff_running.transpose()*Q;
          QPf = Q;
        }
        double cost_running_temp = diffTQPf*diff_running;

        int u_index = std::min(global_i,ch-1);
        u_eigen << u_ptr[7*u_index],u_ptr[7*u_index+1],u_ptr[7*u_index+2],
                   u_ptr[7*u_index+3],u_ptr[7*u_index+4],u_ptr[7*u_index+5],
                   u_ptr[7*u_index+6];

        Eigen::Matrix<double,1,7> uTR = u_eigen.transpose()*R;
        double temp = uTR*u_eigen;
        cost_running = cost_running + cost_running_temp + temp;

        if(!class_ptr->cost_calculate_grad)
          continue;
        Eigen::Matrix<double,7,7> Ji_quat;

        // Eigen::Matrix<double,6,7> Ji = local_moveit->getJacobian(false);
        // Eigen::Matrix<double,4,3> quaternion_update_matrix;
        // quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
        // Ji_quat.topRows<3>() = Ji.topRows<3>();
        // Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();
        // Ji_quat.block<3,1>(0,6) << 0,0,0;
        // Ji_quat(2,0) = 0;

//        class_ptr->jaco_hess_cal.getAnalyticalJacobian(curr_x,Ji_quat);
        Eigen::Matrix<double,6,7> Ji;
        class_ptr->jaco_hess_cal.getAnalyticalJacobianOmega(curr_x, Ji);
        double w =eef_quat(0),x=eef_quat(1),y=eef_quat(2),z=eef_quat(3);
        Eigen::Matrix<double,4,3> quaternion_update_matrix;
        quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
        // quaternion_update_matrix << -x,-y,-z,w,-z,y,z,w,-x,-y,x,w; // this shoud be right one according to paper
        Ji_quat.topRows<3>() = Ji.topRows<3>();
        Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();

        Eigen::Matrix<double,1,7> dfidqi_pos = diffTQPf.leftCols<3>()*-2*Ji_quat.topRows<3>();
        Eigen::Matrix<double,3,4> quat_update;
        quat_update.col(0) << q2;
        quat_update.rightCols<3>() << -w2*Eigen::MatrixXd::Identity(3,3)-skew_q2;
        // quat_update.col(0) << -q2;
        // quat_update.rightCols<3>() << w2*Eigen::MatrixXd::Identity(3,3)-skew_q2; //this shoud be right one according to paper

        Eigen::Matrix<double,1,7> dfidqi_quat = diffTQPf.rightCols<3>()*2*quat_update*Ji_quat.bottomRows<4>();
        Eigen::Matrix<double,1,7> dfidqi = dfidqi_pos+dfidqi_quat;

        Eigen::Matrix<double,1,7> dfiduj_past = dfidqi*Ts;
        Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;

        for(int j=0;j<std::min(ch-1,global_i+1);j++)
        {
          if(j<global_i)
          {
            for(int k=0;k<7;k++)
              grad[j*7+k] += dfiduj_past(k);
          }
          else
          {
            for(int k=0;k<7;k++)
              grad[j*7+k] += dfiduj_now(k);
          }
        }
        if(global_i>=ch-1)
        {
          Eigen::Matrix<double,1,7> dfiduj = 2*uTR + dfidqi*Ts*(global_i-ch+2);
          for(int k=0;k<7;k++)
            grad[(ch-1)*7+k] += dfiduj(k);
        }

      }
      // auto t2 = std::chrono::high_resolution_clock::now();
      // class_ptr->time_consumed[thread_index] = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 );
//      std::cout << "thread["<<thread_index<<"] done." << std::endl;
      class_ptr->cost_finished_ptr[thread_index] = true;
    }
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  if(thread_index==0)
    std::cout << "Cost cal threads ended." << std::endl;
}

double NmpcNlopt::costFuncMultiThread(unsigned n, const double *u, double *grad, void *data)
{
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;
// int& cal_cost_count = class_ptr->cal_cost_count;
// double& time_avg = class_ptr->time_avg;
//multi threading preparation
//  class_ptr->time_consumed.resize(class_ptr->num_cost_threads);

  if(grad)
    class_ptr->cost_calculate_grad = true;
  else
    class_ptr->cost_calculate_grad = false;

  for(int j=0;j<7;j++)
  {
    class_ptr->u_ptr[j] = u[j];
    class_ptr->costCal_jnt_vals_ptr[0][j] =  class_ptr->temp_joint_values[j]+u[j]*Ts;
  }

  for(int i=1;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
    {
      double u_temp = u[u_index*7+j];
      class_ptr->u_ptr[u_index*7+j] = u_temp;
      class_ptr->costCal_jnt_vals_ptr[i][j] = class_ptr->costCal_jnt_vals_ptr[i-1][j]+u_temp*Ts;
    }
  }
  for(int i=0;i<class_ptr->num_cost_threads;i++)
    class_ptr->cost_finished_ptr[i]=false;
  class_ptr->cost_activated = true;


  // wait until all threads calculation finished
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_cost_threads;i++)
    {
      all_threads_done = all_threads_done&&class_ptr->cost_finished_ptr[i];
    }
    if(all_threads_done)
      break;
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
  }
  // deactivate cost cal in threads

  class_ptr->cost_activated = false;

  double multi_cost = 0;
  for(int i=0;i<class_ptr->num_cost_threads;i++)
    multi_cost += class_ptr->partial_costs_ptr[i];
  if(grad)
  {
    for(int j=0;j<ch*7;j++)
    {
      grad[j] = 0;
      for(int i=0;i<class_ptr->num_cost_threads;i++)
        grad[j] += class_ptr->partial_grads_ptr[i][j];
    }
  }
 // average time calculation
 //   time_avg = (duration_total+time_avg*cal_cost_count)/(cal_cost_count+1);
 //   cal_cost_count++;
 //   std::cout << time_avg << " | ";
 // }


//  std::cout << "----" << std::endl;
//  if(class_ptr->cal_cost_count++>1)
//  {
//    ros::shutdown();
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    exit(0);
//  }

  return multi_cost;
}



void NmpcNlopt::cntCalThread(const int thread_index, NmpcNlopt *class_ptr)
{
  std::cout << "Constraint cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->activated && !(class_ptr->finished_ptr[thread_index]))
    {
      // auto t1 = std::chrono::high_resolution_clock::now();
      //cost and grad calculation
      const double& Ts = class_ptr->Ts;
      const int& ch = class_ptr->ch;
      const double safe_distance = 0.02;
      const int& num_obstacles = class_ptr->getObstacleNum();
      const int num_links = 3;
      const int num_dists = num_links*num_obstacles+1;
      const int n = 7*ch;

      // output partial cost value container
      std::vector<double>& cnt_vals = class_ptr->partial_cnt_val[thread_index];
      // output partial cost gradient container
      std::vector<double>& cnt_grads = class_ptr->partial_cnt_grads[thread_index];

      std::vector<double> curr_x(7);
      MoveitTool* local_moveit = class_ptr->MoveitTools_array[thread_index];

      int local_num_cnt_loops = (thread_index==(class_ptr->num_cnt_threads-1))?class_ptr->last_num_cnt_loops:class_ptr->num_cnt_loops;
      for(int i=0;i<local_num_cnt_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_cnt_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->jnt_vals_ptr[global_i][j];
        local_moveit->updateJointState(curr_x, true);
        std::vector< std::vector<double> > dists;
        std::vector<std::vector<Eigen::Vector3d> > points1, points2;
        local_moveit->getDistsAndPoints(dists,points1,points2);
        for(int j=0;j<num_obstacles;j++)
        {
          for(int k=0;k<num_links;k++)
          {
            cnt_vals[global_i*num_dists+j*num_links+k] = -dists[j][k] + safe_distance;
            if(class_ptr->calculate_grad)
            {
              Eigen::Matrix<double,3,7> J_link;
              //Eigen::MatrixXd J_link;
              //class_ptr->getjacobian(k,J_link,points1[j][k]);
              class_ptr->jaco_hess_cal.getAnalyJaco4Ref(k,curr_x, J_link, points1[j][k]);
              Eigen::Vector3d normal = points1[j][k] - points2[j][k];
              normal = normal/normal.norm();
              if(dists[j][k]<0)
                normal = -normal;
              // normal = normal/dists[j][k] 
              Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link;
              //Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link.topRows<3>();
              Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;
              for(int l=0;l<std::min(ch-1,global_i+1);l++)
              {
                for(int m1=0;m1<7;m1++)
                {
                  cnt_grads[global_i*num_dists*n+j*num_links*n+k*n+l*7+m1]=dc_ijk_du_n(m1);
                }
              }
              if(global_i>=ch-1)
              {
                Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(global_i-ch+2);
                for(int m1=0;m1<7;m1++)
                {
                 cnt_grads[global_i*num_dists*n+j*num_links*n+k*n+(ch-1)*7+m1]=dc_ijk_du_last(m1);
                }
              }
            }
          }
        }
        // distance between cylinder 1 and 3
        Eigen::Vector3d cylinder1_point, cylinder3_point;
        double dist13 = local_moveit->selfCollisionDistAndPoints(cylinder1_point,cylinder3_point);
        cnt_vals[global_i*num_dists+num_obstacles*num_links] = -dist13 + safe_distance;

        if(class_ptr->calculate_grad)
        {
          Eigen::Matrix<double,3,7> J_link1,J_link3;
          //Eigen::MatrixXd J_link1,J_link3;
          //class_ptr->getjacobian(0,J_link1,cylinder1_point);
          //class_ptr->getjacobian(2,J_link3,cylinder1_point);
          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(0,curr_x, J_link1, cylinder1_point);
          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(2,curr_x, J_link3, cylinder1_point);
          Eigen::Vector3d normal = cylinder1_point - cylinder3_point;
          normal = normal/normal.norm();
          if(dist13<0)
            normal = -normal;
          Eigen::Matrix<double,1,7> dc_ijk_dq_i = normal.transpose() * (J_link3-J_link1);
          //Eigen::Matrix<double,1,7> dc_ijk_dq_i = normal.transpose() * (J_link3.topRows<3>()-J_link1.topRows<3>());
          Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;

          for(int l=0;l<std::min(ch-1,global_i+1);l++)
          {
            for(int m1=0;m1<7;m1++)
            {
              cnt_grads[global_i*num_dists*n+num_obstacles*num_links*n+l*7+m1]=dc_ijk_du_n(m1);
            }
          }
          if(global_i>=ch-1)
          {
            Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(global_i-ch+2);
            for(int m1=0;m1<7;m1++)
            {
             cnt_grads[global_i*num_dists*n+num_obstacles*num_links*n+(ch-1)*7+m1]=dc_ijk_du_last(m1);
            }
          }
        }

      }

      class_ptr->finished_ptr[thread_index] = true;
    }
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  if(thread_index==0)
    std::cout << "Constraint cal threads ended." << std::endl;
}

void NmpcNlopt::obstacleConstraintsMultiThread(unsigned m, double *result, unsigned n, const double *u, double *grad, void *data)
{
  // auto t1 = std::chrono::high_resolution_clock::now();
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;
  int& cal_cost_count = class_ptr->cal_cost_count;
  double& time_avg = class_ptr->time_avg;


  if(grad)
  {
    for(int i=0;i<m*n;i++)
      grad[i]=0;
    class_ptr->calculate_grad = true;
    for(int i=0;i<class_ptr->num_cnt_threads;i++)
      class_ptr->partial_cnt_grads[i].resize(m*n);
  }
  else
    class_ptr->calculate_grad = false;
  for(int i=0;i<class_ptr->num_cnt_threads;i++)
    class_ptr->partial_cnt_val[i].resize(m);
  for(int j=0;j<7;j++)
    class_ptr->jnt_vals_ptr[0][j] =  class_ptr->temp_joint_values[j]+u[j]*Ts;
  for(int i=1;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
    {
      double u_temp = u[u_index*7+j];
      class_ptr->jnt_vals_ptr[i][j] = class_ptr->jnt_vals_ptr[i-1][j]+u_temp*Ts;
    }
  }
  class_ptr->activated = true;
  for(int i=0;i<class_ptr->num_cnt_threads;i++)
    class_ptr->finished_ptr[i]=false;
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_cnt_threads;i++)
      all_threads_done = all_threads_done&&class_ptr->finished_ptr[i];

    if(all_threads_done)
      break;
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  class_ptr->activated = false;

  for(int i=0;i<m;i++)
  {
    result[i] = 0;
    for(int j=0;j<class_ptr->num_cnt_threads;j++)
      result[i]+=class_ptr->partial_cnt_val[j][i];
  }
  if(grad)
  {
    for(int i=0;i<m*n;i++)
    {
      grad[i]=0;
      for(int j=0;j<class_ptr->num_cnt_threads;j++)
      {
        grad[i] += class_ptr->partial_cnt_grads[j][i];
      }
    }
  }
  return;
}


void NmpcNlopt::optimize(std::vector<double>& u, double mincost)
{
  auto t1 = std::chrono::high_resolution_clock::now();
  try{
      nlopt::result result = optimizer.optimize(u,mincost);
  }
  catch(std::exception &e){
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  //std::cout << duration << std::endl;

  // discard mvs in [1,ch-1]-th time step
  // only send the mv in the frist time step: 0th time step
  pthread_mutex_lock(&joint_velocities_mutex);
  for(int i=0;i<7;i++)
  {
    joint_velocities[i] = u[i];
  }
  pthread_mutex_unlock(&joint_velocities_mutex);
  new_goal_got = true;

  
   

  std::vector<double> traj_point_q(7);
  line_strip.color.g = 0.0;
  line_strip.color.r = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = "panda_trajectory";
  line_strip.points.clear();
  line_strip.points.resize(ph+1);

  for(int i=0;i<7;i++)
    traj_point_q[i] = temp_joint_values(i);
  updateJointState(traj_point_q);
  geometry_msgs::Point traj_point_x;
  Eigen::Vector3d point_x_position = getEEFTransform().translation();

  traj_point_x.x = point_x_position(0);
  traj_point_x.y = point_x_position(1);
  traj_point_x.z = point_x_position(2);
  line_strip.points[0] = traj_point_x;
  for(int i=0;i<ph;i++)
  {
    int index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
      traj_point_q[j] += u[index*7+j]*Ts;
    updateJointState(traj_point_q);
    point_x_position = getEEFTransform().translation();
    traj_point_x.x = point_x_position(0);
    traj_point_x.y = point_x_position(1);
    traj_point_x.z = point_x_position(2);
    line_strip.points[i+1] = traj_point_x;
  }

  //marker_publisher.publish(line_strip);

  line_strip.color.g = 1.0;
  line_strip.color.r = 0.0;
  line_strip.id = 3;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.ns = "panda_trajectory_point";
  //marker_publisher.publish(line_strip);
}



void NmpcNlopt::velocitiesSend_thread_real(ros::Publisher publisher2real, bool* exitFlag,
                                      std::vector<double>* v_ptr, franka_core_msgs::JointCommand* msg,
                                      pthread_mutex_t *joint_velocities_mutex,
                                      bool* new_goal_got)
{
  double passsed_time_after_new_goal;
  std::vector<double> temp_v(7);
  ros::Duration d(0.01);
  double flag;
  //franka_core_msgs::JointCommand* msg;
  while(ros::ok() && !(*exitFlag))
  {
    //std::cout << (*v_ptr)[0] <<(*v_ptr)[1] <<(*v_ptr)[2] << std::endl;
    if((*v_ptr)[0]!=-200)
    {

      if(*new_goal_got)
      {
        passsed_time_after_new_goal = 0;
        *new_goal_got = false;
      }
      else
      {
        passsed_time_after_new_goal +=0.01;
        if(passsed_time_after_new_goal > 0.2)
        {
          //ROS_WARN("Joint velocities not updte for a long time, sending 0 velocities to robot.");
          std::fill(temp_v.begin(),temp_v.end(),0);
        }
        else
        {
          //ROS_WARN("===========asign joint velocites=====================");
          pthread_mutex_lock(joint_velocities_mutex);
          temp_v = *v_ptr;
          //std::cout << temp_v[0]<<"-----"<<temp_v[1]<<"------"<<temp_v[2] << "------"<<temp_v[3]<<"-----"<<temp_v[4]<<"------"<<temp_v[5]<<"------"<<temp_v[6]<< std::endl;
          pthread_mutex_unlock(joint_velocities_mutex);
        }
      }

      msg->header.seq++;
      //msg->position.clear();
      //franka_core_msgs::JointCommand msg_point;
      //msg_point.time_from_start = d;
      //msg->velocity = {-1.037792060227770554, -0.01601235411041661, 0.019782607023391807, 0.0342050140544315, 0.029840531355804868, -0.05411935298621688,-1.322};
      msg->names = {"panda_link1","panda_link2","panda_link3","panda_link4", "panda_link5","panda_link6","panda_hand"};
      
      msg->velocity=temp_v;
      msg->mode=msg->VELOCITY_MODE;
      
        publisher2real.publish(*msg);
      
      
    }
    d.sleep();
  }
}


void NmpcNlopt::velocitiesSend_thread_sim(std::vector<double>* c_ptr,ros::Publisher publisher, bool* exitFlag,
                                      std::vector<double>* v_ptr, franka_core_msgs::JointCommand* msg,
                                      pthread_mutex_t *joint_velocities_mutex, pthread_mutex_t* position_goal_mutex,
                                      bool* new_goal_got, std::vector<double>* position_goal_ptr)
{
  double passsed_time_after_new_goal;
  std::vector<double> temp_v(7);
  ros::Duration d(0.01);
  double flag;

  while(ros::ok() && !(*exitFlag))
  {
    //std::cout << (*v_ptr)[0] <<(*v_ptr)[1] <<(*v_ptr)[2] << std::endl;
    if((*v_ptr)[0]!=-200)
    {

      if(*new_goal_got)
      {
        passsed_time_after_new_goal = 0;
        *new_goal_got = false;
        //std::cout << temp_v[0]<<"-----"<<temp_v[1]<<"------"<<temp_v[2] << "------"<<temp_v[3]<<"-----"<<temp_v[4]<<"------"<<temp_v[7]<<"------"<<temp_v[8]<< std::endl;
         
      }
      else
      {
        passsed_time_after_new_goal +=0.01;
        if(passsed_time_after_new_goal > 0.2)
        {
          //ROS_WARN("Joint velocities not updte for a long time, sending 0 velocities to robot.");
          std::fill(temp_v.begin(),temp_v.end(),0.000001);
        }
        else
        {
          //ROS_WARN("===========asign joint velocites=====================");
          pthread_mutex_lock(joint_velocities_mutex);
          temp_v = *v_ptr;
          //std::cout << temp_v[0]<<"-----"<<temp_v[1]<<"------"<<temp_v[2] << "------"<<temp_v[3]<<"-----"<<temp_v[4]<<"------"<<temp_v[7]<<"------"<<temp_v[8]<< std::endl;
          //std::cout << temp_v.size() << std::endl;
          pthread_mutex_unlock(joint_velocities_mutex);
        }
      

      msg->header.seq++;
      //msg->position.clear();
      //franka_core_msgs::JointCommand msg_point;
      //msg_point.time_from_start = d;
      
      //msg->velocity = {-1.037792060227770554, -0.01601235411041661, 0.019782607023391807, 0.0342050140544315, 0.029840531355804868, -0.05411935298621688,-1.322};
      msg->names = {"panda_link1","panda_link2","panda_link3","panda_link4", "panda_link5","panda_link6","panda_hand"};
      
      //velocity mode
      //->velocity=temp_v;
      //std::cout << temp_v[0]<<"-----"<<temp_v[1]<<"------"<<temp_v[2] << "------"<<temp_v[3]<<"-----"<<temp_v[4]<<"------"<<temp_v[7]<<"------"<<temp_v[8]<< std::endl;
         
      //msg->velocity={-1.2,0,0,0,0,0,0.5};
      //msg->mode = msg->VELOCITY_MODE;
      //publisher.publish(*msg);
      //std::cout << msg->velocity[0] << std::endl;
      //msg = msg_point; 
      

    //position mode
      
      pthread_mutex_lock(position_goal_mutex);
      for(int i=0;i<7;i++)
        (*position_goal_ptr)[i] += (temp_v[i])*0.01;
      pthread_mutex_unlock(position_goal_mutex);
      msg->position = *position_goal_ptr;
      msg->mode =msg->POSITION_MODE;
      publisher.publish(*msg);
      
      }
    }
    d.sleep();
  }
  std::cout << "Velocities sending thread ended. " << std::endl;
}


void NmpcNlopt::initialize()
{
	// add a box obstacle representing robot body
	shapes::ShapeConstPtr obj_shape1(new shapes::Box(0.52,0.4,0.25));
	Eigen::Quaterniond quater(0,0,0,1);
  quater.normalize ();

	Eigen::Translation3d translation(0.05,0,body_height+0.5);
	Eigen::Affine3d obj_pose = translation*quater;


	shapes::ShapeConstPtr obj_shape4(new shapes::Box(2,2,0.01));
	translation.x()=0;translation.y()=0;translation.z()=0;
	obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("ground",obj_shape4,obj_pose,plot_obstacle);
}
  

void NmpcNlopt::controlLoop()
{
  //ROS_INFO("Control Loop ");

	if(!state_sub_started)
		return;
  // update joint states
  pthread_mutex_lock(&position_goal_mutex);
  for(int i=0;i<7;i++)
  {
    temp_joint_values(i) = curr_joint_values[i];
    position_goal[i] = curr_joint_values[i];
  }
  updateJointState(curr_joint_values,true,true);
  pthread_mutex_unlock(&position_goal_mutex);
  perception_msgs::Cylinders cylinders;
  cylinders.centers.resize(3);
  cylinders.x_axes.resize(3);
  cylinders.y_axes.resize(3);
  cylinders.z_axes.resize(3);
  for(int i=0;i<3;i++)
  {
  	cylinders.centers[i].x = centroids[i][0];cylinders.centers[i].y = centroids[i][1];cylinders.centers[i].z = centroids[i][2];
  	cylinders.x_axes[i].x = x_axes[i][0];cylinders.x_axes[i].y = x_axes[i][1];cylinders.x_axes[i].z = x_axes[i][2];
  	cylinders.y_axes[i].x = y_axes[i][0];cylinders.y_axes[i].y = y_axes[i][1];cylinders.y_axes[i].z = y_axes[i][2];
  	cylinders.z_axes[i].x = z_axes[i][0];cylinders.z_axes[i].y = z_axes[i][1];cylinders.z_axes[i].z = z_axes[i][2];
  }
  self_cylinders_pub.publish(cylinders);


  // only need right arm eef position
  const auto& eef_transform = getEEFTransform();
  const auto& translation = eef_transform.translation();
  const auto& rotation = eef_transform.linear();
  Eigen::Quaterniond quat(rotation);
  geometry_msgs::Pose eef_pose_msg;
  eef_pose_msg.position.x = translation.x();
  eef_pose_msg.position.y = translation.y();
  eef_pose_msg.position.z = translation.z();
  eef_pose_msg.orientation.w = quat.w();
  eef_pose_msg.orientation.x = quat.x();
  eef_pose_msg.orientation.y = quat.y();
  eef_pose_msg.orientation.z = quat.z();
  eef_pose_pub.publish(eef_pose_msg);

//  if(track_mode&&goal_sub_started)
  if(track_mode)
  {
    optimize(u,mincost);

		// warm start
    for(int i=0;i<(ch-1);i++)
     for(int j=0;j<7;j++)
     {
       u[i*7+j] = u[(i+1)*7+j];
     }
    for(int j=0;j<7;j++)
    {
      u[(ch-1)*7+j]=0;
    }
  }

}
