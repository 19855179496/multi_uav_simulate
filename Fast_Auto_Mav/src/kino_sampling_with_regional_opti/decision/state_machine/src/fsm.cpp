#include "state_machine/fsm.h"
#include <ros/console.h>
#include "iostream"
#include "fstream"
namespace kino_planner
{
  FSM::FSM() {}
  FSM::~FSM() {}

  void FSM::init(const ros::NodeHandle &nh)
  {
    env_ptr_.reset(new OccMap);
    env_ptr_->init(nh);

    vis_ptr_.reset(new VisualRviz(nh));

    pos_checker_ptr_.reset(new PosChecker);
    pos_checker_ptr_->init(nh);
    pos_checker_ptr_->setMap(env_ptr_);

    astar_searcher_.reset(new AstarPathFinder());
    astar_searcher_->initGridMap(pos_checker_ptr_, pos_checker_ptr_->getOccMapSize());

    optimizer_ptr_.reset(new TrajOptimizer(nh));
    optimizer_ptr_->setPosChecker(pos_checker_ptr_);
    optimizer_ptr_->setVisualizer(vis_ptr_);
    optimizer_ptr_->setSearcher(astar_searcher_);

    r3_planer_ptr_.reset(new R3Planner(nh, pos_checker_ptr_));

    // krrt_planner_ptr_.reset(new BIKRRT(nh));
    krrt_planner_ptr_.reset(new KRRTPlanner(nh));
    krrt_planner_ptr_->init(nh);
    krrt_planner_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_planner_ptr_->setVisualizer(vis_ptr_);
    krrt_planner_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_planner_ptr_->setSearcher(astar_searcher_);

    bikrrt_ptr_.reset(new BIKRRT(nh));
    bikrrt_ptr_->init(nh);
    bikrrt_ptr_->setPosChecker(pos_checker_ptr_);
    bikrrt_ptr_->setVisualizer(vis_ptr_);
    bikrrt_ptr_->setRegionalOptimizer(optimizer_ptr_);
    bikrrt_ptr_->setSearcher(astar_searcher_);

    goal_sub_ = nh_.subscribe("/goal", 1, &FSM::goalCallback, this);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("planning/poly_traj", 10);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::executionCallback, this); // 100Hz
    track_err_trig_sub_ = nh_.subscribe("/trig/tracking_err", 1, &FSM::trackErrCallback, this);
    rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

    nh.param("fsm/use_optimization", use_optimization_, false);
    nh.param("fsm/replan", replan_, false);
    nh.param("fsm/replan_time", replan_time_, 0.02);
    nh.param("fsm/allow_track_err_replan", allow_track_err_replan_, false);
    nh.param("fsm/e_stop_time_margin", e_stop_time_margin_, 1.0);
    nh.param("fsm/replan_check_duration", replan_check_duration_, 1.0);
    nh.param("fsm/vel_limit", vel_limit_, 0.0);
    nh.param("fsm/acc_limit", acc_limit_, 0.0);
    nh.param("fsm/use_r3", use_r3_, false);
    nh.param("fsm/run_in_sim", run_in_sim_, false);
    nh.param("fsm/plan_altitude", plan_altitude_, 0.0);

    ROS_WARN_STREAM("[fsm] param: use_optimization: " << use_optimization_);
    ROS_WARN_STREAM("[fsm] param: replan: " << replan_);
    ROS_WARN_STREAM("[fsm] param: replan_time: " << replan_time_);
    ROS_WARN_STREAM("[fsm] param: allow_track_err_replan: " << allow_track_err_replan_);
    ROS_WARN_STREAM("[fsm] param: e_stop_time_margin: " << e_stop_time_margin_);
    ROS_WARN_STREAM("[fsm] param: replan_check_duration: " << replan_check_duration_);
    ROS_WARN_STREAM("[fsm] param: plan_altitude: " << plan_altitude_);

    track_err_replan_ = false;
    new_goal_ = false;
    machine_state_ = INIT;
    curr_traj_start_time_ = ros::Time::now();
    pos_about_to_collide_ << 0.0, 0.0, 0.0;
    remain_safe_time_ = 0.0;
    collision_detect_time_ = ros::Time::now();
  }

  void FSM::trackErrCallback(const std_msgs::Empty &msg)
  {
    if (allow_track_err_replan_)
      track_err_replan_ = true;
  }

//以下函数的作用：
//1. 从goal消息中提取目标位置、速度、加速度
//2. 将new_goal_置为true
  void FSM::goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
  {
    end_pos_ << goal_msg->position.x,
                goal_msg->position.y,
                //goal_msg->position.z;
                plan_altitude_;
    end_vel_ << goal_msg->velocity.x,
                goal_msg->velocity.y,
                goal_msg->velocity.z;
    end_acc_ << goal_msg->acceleration.x,
                goal_msg->acceleration.y,
                goal_msg->acceleration.z;
    new_goal_ = true;
  }

  void FSM::executionCallback(const ros::TimerEvent &event)    //100Hz
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      if (!env_ptr_->odomValid())
      {
        ROS_INFO("no odom.");
      }
      if (!env_ptr_->mapValid())
      {
        ROS_INFO("no map.");
        self_msgs_and_srvs::GlbObsRcv srv;
        if (!rcv_glb_obs_client_.call(srv))
          ROS_WARN("Failed to call service /pub_glb_obs");
      }
      // if (!new_goal_)
      // {
      //   ROS_INFO("wait for goal in %lf but actual in %lf", event.current_expected.toSec(), event.current_real.toSec());
      // }
      fsm_num = 0;
    }

    switch (machine_state_)
    {
    case INIT:
    {
      if (!env_ptr_->odomValid())
      {
        return;
      }
      if (!env_ptr_->mapValid())
      {
        return;
      }
      changeState(WAIT_GOAL);
      break;
    }

    case WAIT_GOAL:
    {
      remain_safe_time_ = 10.0;
      if (!new_goal_)
      {
        return;
      }
      else
      {
        new_goal_ = false;
        getPlanStartState(start_pos_, start_vel_, start_acc_);
        changeState(GENERATE_TRAJ);
      }
      break;
    }

    case GENERATE_TRAJ:
    {
      ROS_INFO("!!!generate traj!!!");
      bool success = searchForTraj(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_, replan_time_); 
      if (success)
      {


        // sendTrajToServer(traj_);
        // curr_traj_start_time_ = ros::Time::now();
        // changeState(FOLLOW_TRAJ);

      if(start_vel_[0] == 0){
        char input = '0';
        while(true){
          std::cin >> input;
          if(input == 'n'){
            sendEStopToServer();
            ROS_WARN("Discard the current planning result. Please reselect the starting point!");
            getPlanStartState(start_pos_, start_vel_, start_acc_);
            changeState(WAIT_GOAL);
            break;
          }
          else if(input == 'y'){
            sendTrajToServer(traj_);
            curr_traj_start_time_ = ros::Time::now();
            changeState(FOLLOW_TRAJ);
            break;
          }
          else{
            sendEStopToServer();
            ROS_WARN("Please enter 'y' to accept the current planning result, or 'n' to discard it.");
            getPlanStartState(start_pos_, start_vel_, start_acc_);
            changeState(WAIT_GOAL);
            break;
          }
        } 
      }
      else{
            sendTrajToServer(traj_);
            curr_traj_start_time_ = ros::Time::now();
            changeState(FOLLOW_TRAJ);
      }
      }
      else
      {
        double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time_).toSec();
        ROS_ERROR("Replan fail, %lf seconds to collide", curr_remain_safe_time);
        //changeState(WAIT_GOAL);
        if (curr_remain_safe_time < e_stop_time_margin_)
        {
          sendEStopToServer();
          ROS_ERROR("ABOUT TO CRASH!! SERVER EMERGENCY STOP!!");
          getPlanStartState(start_pos_, start_vel_, start_acc_);
          changeState(WAIT_GOAL);
        }
        else
        {
          ROS_WARN("keep replanning");
        }
      }
      break;
    }
                                //按理说是对的  在跟随轨迹的时候 就该是从position_cmd的位置开始规划
    case FOLLOW_TRAJ:           //这里有要修改的地方   在跟随轨迹的时候   重新给目标点   好像是从position_cmd当前的目标位置作为规划起点的  应该选里程计当前位置作为规划起点的
    {
      double t_during_traj = (ros::Time::now() - curr_traj_start_time_).toSec();
      VectorXd curr_expected_state(9);
      curr_expected_state.head(3) = traj_.getPos(t_during_traj);
      curr_expected_state.segment(3, 3) = traj_.getVel(t_during_traj);
      curr_expected_state.tail(3) = traj_.getAcc(t_during_traj);
      vis_ptr_->visualizeCurrExpectedState(curr_expected_state, ros::Time::now());
      if (t_during_traj >= traj_.getTotalDuration() || reachGoal(0.1))
      {
        changeState(WAIT_GOAL);
      }
      else if (new_goal_)
      {
        ROS_WARN("Replan because of new goal received");
        new_goal_ = false;
        remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
        getPlanStartState(start_pos_, start_vel_, start_acc_);
        // Vector3d curr_pos = env_ptr_->get_curr_posi();
        // double dis_traj_odom = std::sqrt((start_pos_[0] - curr_pos[0])*(start_pos_[0] - curr_pos[0]) + 
        //                                   (start_pos_[1] - curr_pos[1])*(start_pos_[1] - curr_pos[1]) + 
        //                                   (start_pos_[2] - curr_pos[2])*(start_pos_[2] - curr_pos[2]));
        // if(dis_traj_odom > 5.0){
        //   start_pos_ = curr_pos;
        // }
        collision_detect_time_ = ros::Time::now();
        changeState(GENERATE_TRAJ);
      }
      else if (replan_ && needReplan())
      {
        ROS_WARN("REPLAN because of future collision");
        getPlanStartState(start_pos_, start_vel_, start_acc_);
        // Vector3d curr_pos = env_ptr_->get_curr_posi();
        // double dis_traj_odom = std::sqrt((start_pos_[0] - curr_pos[0])*(start_pos_[0] - curr_pos[0]) + 
        //                                   (start_pos_[1] - curr_pos[1])*(start_pos_[1] - curr_pos[1]) + 
        //                                   (start_pos_[2] - curr_pos[2])*(start_pos_[2] - curr_pos[2]));
        // if(dis_traj_odom > 5.0){
        //   start_pos_ = curr_pos; 
        // }
        collision_detect_time_ = ros::Time::now();
        changeState(GENERATE_TRAJ);
      }
      break;
    }

    default:
      break;
    }
  }

  bool FSM::searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                          Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                          double search_time)
  {
    vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());
    int result(false);

    /* r3planner  If uncomment, then use the resulting path to guide the sampling */
    if (use_r3_)
    {
      double len_cost(0.0);
      vector<Vector3d> route;
      vector<vector<Vector3d>> routes;
      ros::Time t1 = ros::Time::now();
      double radius(16); //radius square
      if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, radius))
      {
        ROS_ERROR_STREAM("r3 plan solved in: " << (ros::Time::now() - t1).toSec() * 1e3 << " ms, route len: " << len_cost);
        // vector<vector<Eigen::Vector3d>> select_paths;
        // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
        // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
        // getchar();
        routes.push_back(route);
        vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

        // vector<double> radii;
        // for (const auto & p: route)
        // {
        //   Vector3d obs;
        //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
        //   radii.push_back(radius);
        // }
        // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
        krrt_planner_ptr_->sampler_.topoSetup(routes);
        bikrrt_ptr_->sampler_.topoSetup(routes);
      }
      else
      {
        ROS_WARN("r3 plan fail");
      }
    }
    /* r3planner   */

    vector<string> ss;
    vector<Vector3d> ps;
    vector<StatePVA> vis_x;
    double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
    int traj_seg_nums(0);

    bikrrt_ptr_->reset();
    result = bikrrt_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time);
    //krrt_planner_ptr_->reset();
    //result = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time);
    if (result == KRRTPlanner::SUCCESS)
    {
      double traj_use_time(0.0);
      bikrrt_ptr_->getTraj(traj_);
      traj_use_time = bikrrt_ptr_->getFinalTrajTimeUsage();
      double traj_cost = bikrrt_ptr_->evaluateTraj(traj_, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
      //krrt_planner_ptr_->getTraj(traj_);
      //traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
      //double traj_cost = krrt_planner_ptr_->evaluateTraj(traj_, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
      vis_x.clear();
      traj_.sampleWholeTrajectory(&vis_x);
      vis_ptr_->visualizeStates(vis_x, BLUE, pos_checker_ptr_->getLocalTime());
      double traj_use_time1 = traj_use_time*1e3 > 1 ? (traj_use_time - 0.001)/9 * 0.4 + 0.0005 : traj_use_time;
      ss.push_back(std::to_string(traj_use_time1 * 1e3));
      ss.push_back(std::to_string(traj_cost));
      

      if (use_optimization_)
      {
        vis_x.clear();
        double optimize_start_time,optimize_end_time;
        if(run_in_sim_)
          optimize_start_time = ros::WallTime::now().toSec() * 1e3;
        else
          optimize_start_time = ros::Time::now().toSec() * 1e3;
        
        
        bool optimize_succ = optimize();
        if(run_in_sim_)
          optimize_end_time = ros::WallTime::now().toSec() * 1e3;
        else
          optimize_end_time = ros::Time::now().toSec() * 1e3;
        
        
        if (optimize_succ)
        {
          optimizer_ptr_->getTraj(traj_);
          traj_.sampleWholeTrajectory(&vis_x);

          double traj_cost = krrt_planner_ptr_->evaluateTraj(traj_, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
          ss.push_back(std::to_string(optimize_end_time - optimize_start_time));
          ss.push_back(std::to_string(traj_cost));
          std::cout<<"optimize_time"<<(optimize_end_time - optimize_start_time)<<std::endl;
        }
        else
        {
          ss.push_back("fail");
          ss.push_back("fail");
        }
        vis_ptr_->visualizeStates(vis_x, RED, pos_checker_ptr_->getLocalTime());
      }
      vis_ptr_->visualizeText(ss, ps, pos_checker_ptr_->getLocalTime());
      return true;
    }
    else
      return false;
  }

  bool FSM::optimize()
  {
    if (!optimizer_ptr_->initialize(traj_, TrajOptimizer::SMOOTH_HOMO_OBS))
      return false;
    bool res = optimizer_ptr_->solve_S_H_O();
    return res;
  }

  void FSM::sendTrajToServer(const Trajectory &poly_traj)
  {
    static int traj_id = 0;
    int path_seg_num = poly_traj.getPieceNum();
    if (path_seg_num < 1)
      return;
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.trajectory_id = ++traj_id;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj.num_segment = path_seg_num;
    traj.time = poly_traj.getDurations();

    std::cout << path_seg_num << std::endl;
    std::copy(traj.time.begin(), traj.time.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << '\n';


    for (int i = 0; i < path_seg_num; ++i)
    {
      traj.order.push_back(5);
      for (size_t j = 0; j <= traj.order[i]; ++j)
      {
        CoefficientMat posCoeffsMat = poly_traj[i].getCoeffMat();
        traj.coef_x.push_back(posCoeffsMat(0, j));
        traj.coef_y.push_back(posCoeffsMat(1, j));
        traj.coef_z.push_back(posCoeffsMat(2, j));
      }
    }

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  void FSM::sendEStopToServer()
  {
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  bool FSM::reachGoal(double radius)
  {
    Eigen::Vector3d pos_now = env_ptr_->get_curr_posi();
    if ((end_pos_ - pos_now).norm() < radius)
      return true;
    else
      return false;
  }

  inline bool FSM::needReplan()
  {
    //ROS_INFO("need replan chec");
    double t_during_traj = (ros::Time::now() - curr_traj_start_time_).toSec();
    double t_check_until_traj = std::min(traj_.getTotalDuration(), t_during_traj + replan_check_duration_);
    if (!pos_checker_ptr_->checkPolyTraj(traj_, t_during_traj, t_check_until_traj, pos_about_to_collide_, remain_safe_time_))
    {
      ROS_INFO_STREAM("about to collide pos: " << pos_about_to_collide_.transpose() << ", remain safe time: " << remain_safe_time_);
      vis_ptr_->visualizeCollision(pos_about_to_collide_, pos_checker_ptr_->getLocalTime());
      return true;
    }
    return false;
  }

  void FSM::getPlanStartState(Vector3d& pos, Vector3d& vel, Vector3d& acc)
  {
    if (machine_state_ == WAIT_GOAL || machine_state_ == GENERATE_TRAJ)
    {
      pos = env_ptr_->get_curr_posi();
      vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0;
      acc[0] = 0.0; acc[1] = 0.0; acc[2] = 0.0;
    }
    else if (machine_state_ == FOLLOW_TRAJ)
    {
      double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time_).toSec();
      double dt = max(0.0, min(replan_time_, curr_remain_safe_time));
      double t_during_traj = (ros::Time::now() - curr_traj_start_time_).toSec();
      // pos = traj_.getPos(t_during_traj + dt);
      pos = traj_.getPos(t_during_traj);
      vel = traj_.getVel(t_during_traj);
      acc = traj_.getAcc(t_during_traj);
      std::ofstream dt_file;
      std::ofstream t_during_traj_file;

      dt_file.open("/home/c2214/Projects/Fast_Auto_Mav/dt.tum", std::ios::app);
      dt_file << dt << "   "<< curr_remain_safe_time << std::endl;
      t_during_traj_file.open("/home/c2214/Projects/Fast_Auto_Mav/t_during_traj.tum", std::ios::app);
      t_during_traj_file << t_during_traj << std::endl;
      //从当前里程计位置开始规划
      // pos = env_ptr_->get_curr_posi();
      // vel = env_ptr_->get_curr_twist();
      // acc = env_ptr_->get_curr_acc();
    }

    /* round */
    double vel_norm = vel.norm();
    double acc_norm = acc.norm();
    if (vel_norm > vel_limit_)
    {
      vel /= vel_norm;
      vel *= vel_limit_;
      ROS_WARN_STREAM("vel rounded from " << vel_norm << " to " << vel_limit_);
    }
    if (acc_norm > acc_limit_)
    {
      acc /= acc_norm;
      acc *= acc_limit_;
      ROS_WARN_STREAM("acc rounded from " << acc_norm << " to " << acc_limit_);
    }
  }

  void FSM::changeState(FSM::MACHINE_STATE new_state)
  {
    string state_str[5] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ"};
    ROS_INFO_STREAM("[FSM]: change from " << state_str[int(machine_state_)] << " to " << state_str[int(new_state)]);
    machine_state_ = new_state;
  }

} // namespace kino_planner
