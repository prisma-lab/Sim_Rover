#include "rover_manager.h"
#include <regex>

RoverManager::RoverManager() : Node("rover_manager")
{
    this->declare_parameter<std::string>("home_pose", "home");
    this->get_parameter("home_pose", home_pose_);

    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "rover/navigate_to_pose");

    current_command = "";
    new_command = "";
    command_running = false;
    nav2_running = false;
    coverage_active = false;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "seed_pdt_rover/command", 10,
        std::bind(&RoverManager::command_callback, this, std::placeholders::_1));
        
    rover_feedback_pb_ = this->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    coverage_publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/coverage_area", 10);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&RoverManager::command_manager_callback, this));
        
    RCLCPP_INFO(this->get_logger(), "rover_manager created");
}

std::vector<std::string> RoverManager::instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    ss>>c;
    while(!ss.eof())
    {
        if(c=='"' && !isString){ isString=true; app=app+c; }
        else if(c=='"' && isString){ isString=false; app=app+c; }
        else if(isString){ app=app+c; }
        else if(c=='(' && isAtom){ isAtom=false; result.push_back(app); app=""; }
        else if(c=='(' || c=='['){ count++; app=app+c; }
        else if((c==')' || c==']') && count!=0){ count--; app=app+c; }
        else if(c!=',' || count!=0) app=app+c;
        else { result.push_back(app); app=""; }
        ss>>c;
    }
    if(isAtom) {
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            while(std::getline(ss2, substr, '\\')) result.push_back(substr);
        }
        else result.push_back(app);
    }
    else{
        if(!app.empty()) app.erase(app.size()-1);
        result.push_back(app);
    }
    return result;
}

geometry_msgs::msg::TransformStamped RoverManager::get_tf(const std::string& source_frame, const std::string& target_frame)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(200));
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform from %s to %s: %s", target_frame.c_str(), source_frame.c_str(), ex.what());
        throw;
    }

    return transform_stamped;
}

void RoverManager::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  new_command = msg->data;
}

void RoverManager::command_manager_callback()
{
    if(new_command.empty()) return;

    // se il comando è lo stesso di quello in esecuzione, non fare nulla
    if(command_running && new_command == current_command){
        return;
    }

    // se sto cambiando comando
    if(new_command != current_command){
        RCLCPP_INFO(this->get_logger(), "switching from %s to %s", current_command.c_str(), new_command.c_str());

        // cancella goal NAV2 se necessario
        if(nav2_running){
            this->client_ptr_->async_cancel_all_goals();
            nav2_running = false;
        }

        // ferma copertura se attiva
        if(coverage_active){
            coverage_active = false;
            auto empty_polygon = geometry_msgs::msg::Polygon();
            coverage_publisher_->publish(empty_polygon);
        }

        // esegui il nuovo comando
        execute_command(new_command);
        current_command = new_command;
        command_running = true;

        // pulisci new_command solo se vuoi evitare di rieseguire lo stesso comando
        new_command = "";
    }
}

bool RoverManager::parse_xyyaw(const std::string &s, double &x, double &y, double &yaw)
{
    std::vector<std::string> toks;
    std::stringstream ss(s);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        if(!tok.empty()) toks.push_back(tok);
    }
    try{
        if(toks.size() >= 2){
            x = std::stod(toks[0]);
            y = std::stod(toks[1]);
            yaw = (toks.size() >= 3) ? std::stod(toks[2]) : 0.0;
            return true;
        } else if(toks.size() == 1){
            x = std::stod(toks[0]);
            y = 0.0;
            yaw = 0.0;
            return true;
        }
    } catch(...) { return false; }
    return false;
}


bool RoverManager::send_goto_goal(double x, double y, double yaw)
{
    if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))){
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return false;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,yaw);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&RoverManager::nav2_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&RoverManager::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&RoverManager::nav2_result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    nav2_running = true;
    coverage_active = false;
    return true;
}

void RoverManager::execute_command(const std::string& cmd)
{
    cv = instance2vector(cmd);
    if(cv.empty()) return;
    std::string op = cv[0];

    if(op == "goto"){
        if(cv.size() < 2){ command_running=false; return; }
        double x=0,y=0,yaw=0;
        if(parse_xyyaw(cv[1],x,y,yaw)) send_goto_goal(x,y,yaw);
        else {
            try{
                auto tf = get_tf("map", cv[1]);
                tf2::Quaternion quat(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);
                double roll,pitch;
                tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                x=tf.transform.translation.x; y=tf.transform.translation.y;
                send_goto_goal(x,y,yaw);
            } catch(...){ command_running=false; }
        }
    }
    else if(op == "coverage"){
        if(cv.size() < 2){ command_running = false; return; }

        geometry_msgs::msg::Polygon poly;

        // regex per estrarre tutte le coppie (x,y)
        std::regex point_re("\\(*\\s*([0-9.+-]+)\\s*,\\s*([0-9.+-]+)\\s*\\)*");
        auto begin = std::sregex_iterator(cmd.begin(), cmd.end(), point_re);
        auto end   = std::sregex_iterator();

        for(auto it = begin; it != end; ++it){
            float x = std::stof((*it)[1].str());
            float y = std::stof((*it)[2].str());
            geometry_msgs::msg::Point32 p;
            p.x = x; p.y = y; p.z = 0.0f;
            poly.points.push_back(p);
        }

        if(poly.points.size() < 3){
            RCLCPP_ERROR(this->get_logger(), "Coverage command malformed, found only %zu points", poly.points.size());
            command_running = false;
            return;
        }

        coverage_publisher_->publish(poly);
        RCLCPP_INFO(this->get_logger(), "Published coverage polygon with %zu points", poly.points.size());
        coverage_active = true;
        nav2_running = false;
    }
    else if(op == "wait"){
        nav2_running=false; coverage_active=false; command_running=false; current_command="";
    }
    else if(op == "stop" || op=="emergency_stop"){
        if(client_ptr_) client_ptr_->async_cancel_all_goals();
        geometry_msgs::msg::Twist tw; tw.linear.x=tw.linear.y=tw.angular.z=0.0;
        cmd_vel_pub_->publish(tw);
        nav2_running=false; coverage_active=false; command_running=false; current_command="";
        current_command = new_command;
        command_running = false;
    }
    else if(op == "cancel" || op=="cancel_goal"){
        if(client_ptr_) client_ptr_->async_cancel_all_goals();
        nav2_running=false; command_running=false; current_command="";
        current_command = new_command;
        command_running = false;        
    }
    else if(op == "return" || op=="return_to_base"){
        double x=0,y=0,yaw=0;
        if(parse_xyyaw(home_pose_,x,y,yaw)) send_goto_goal(x,y,yaw);
        else {
            try{
                auto tf = get_tf("map", home_pose_);
                tf2::Quaternion quat(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);
                double roll,pitch;
                tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                x=tf.transform.translation.x; y=tf.transform.translation.y;
                send_goto_goal(x,y,yaw);
            } catch(...){ command_running=false; }
        }
    }
    else{ command_running=false; current_command=""; }
}

void RoverManager::nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
{
    goal_handle_ = future; // assegnazione diretta
    if(!goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
}


void RoverManager::nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}

void RoverManager::nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch(result.code){
        case rclcpp_action::ResultCode::SUCCEEDED: {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            break;
        }
        case rclcpp_action::ResultCode::ABORTED: {
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            // Se possibile, informa SEED che il goal non è raggiungibile
            if(!current_command.empty()){
                auto vec = instance2vector(current_command);
                if(vec.size() >= 2){
                    std_msgs::msg::String msg;
                    msg.data = vec[1] + ".unreachable";
                    rover_feedback_pb_->publish(msg);
                }
            }
            break;
        }
        case rclcpp_action::ResultCode::CANCELED: {
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        }
        default: {
            RCLCPP_WARN(this->get_logger(), "Unknown result code");
            break;
        }
    }

    // Reset dello stato dei comandi
    current_command = "";
    command_running = false;
    nav2_running = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

