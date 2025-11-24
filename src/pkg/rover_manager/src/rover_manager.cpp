#include "rover_manager.h"
#include <regex>
#include <sstream>

RoverManager::RoverManager() : Node("rover_manager")
{
    this->declare_parameter<std::string>("home_pose", "rover/map");
    this->get_parameter("home_pose", home_pose_);

    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "rover/navigate_to_pose");

    current_command = "";
    new_command = "";
    command_running = false;
    nav2_running = false;
    coverage_active = false;
    navigation_in_progress = false;
    swath_width = 1.0; // Aggiungi questa inizializzazione

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "seed_pdt_rover/command", 10,
        std::bind(&RoverManager::command_callback, this, std::placeholders::_1));
        
    rover_feedback_pb_ = this->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rover/cmd_vel", 10);
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
  RCLCPP_INFO(this->get_logger(), "Received command: %s", new_command.c_str());
}

void RoverManager::command_manager_callback()
{
    // se coverage è attiva, esegui un passo
    if(coverage_active){
        execute_coverage_step();
    }

    if(new_command.empty()) return;

    // se il comando è lo stesso di quello in esecuzione, non fare nulla
    // RCLCPP_INFO(this->get_logger(), "Current command: %s", current_command.c_str());
    if(command_running && (new_command == current_command)){
        return;
    }

    // se sto cambiando comando
    if(new_command != current_command){
        RCLCPP_INFO(this->get_logger(), "switching from %s to %s", current_command.c_str(), new_command.c_str());

        // cancella goal NAV2 se necessario
        if(nav2_running){
            this->client_ptr_->async_cancel_all_goals();
            nav2_running = false;
            navigation_in_progress = false;
        }

        // ferma copertura se attiva
        if(coverage_active){
            coverage_active = false;
            navigation_in_progress = false;
            auto empty_polygon = geometry_msgs::msg::Polygon();
            coverage_publisher_->publish(empty_polygon);
        }

        // esegui il nuovo comando
        execute_command(new_command);
        current_command = new_command;
        command_running = true;
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
    navigation_in_progress = true;
    return true;
}

// Nuova funzione per estrarre punti dal comando coverage
std::vector<std::pair<double, double>> RoverManager::parse_coverage_points(const std::string& cmd) {
    std::vector<std::pair<double, double>> points;
    
    // Regex per trovare coppie di coordinate tra parentesi
    std::regex point_regex(R"(\(([^,]+),([^)]+)\))");
    std::sregex_iterator iter(cmd.begin(), cmd.end(), point_regex);
    std::sregex_iterator end;
    
    while (iter != end) {
        try {
            double x = std::stod((*iter)[1]);
            double y = std::stod((*iter)[2]);
            points.push_back({x, y});
            RCLCPP_INFO(this->get_logger(), "Parsed point: (%f, %f)", x, y);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing point: %s", e.what());
        }
        ++iter;
    }
    
    return points;
}

void RoverManager::execute_command(const std::string& cmd)
{
    auto cv = instance2vector(cmd);
    if(cv.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty command");
        command_running = false;
        return;
    }
    
    std::string op = cv[0];
    RCLCPP_INFO(this->get_logger(), "Executing command: %s", op.c_str());

    if(op == "goto"){
        if(cv.size() < 2){ 
            command_running = false; 
            return; 
        }
        double x=0,y=0,yaw=0;
        if(parse_xyyaw(cv[1],x,y,yaw)) {
            send_goto_goal(x,y,yaw);
        } else {
            try{
                auto tf = get_tf("map", cv[1]);
                tf2::Quaternion quat(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);
                double roll,pitch;
                tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                x=tf.transform.translation.x; 
                y=tf.transform.translation.y;
                send_goto_goal(x,y,yaw);
            } catch(...){ 
                command_running = false; 
            }
        }
    }
    else if(op == "coverage" || op == "cover"){  // Supporta entrambi i nomi
        if(cv.size() < 2){ 
            command_running = false; 
            return; 
        }

        // Estrai i punti dal comando
        coverage_area = parse_coverage_points(cmd);
        
        if(coverage_area.size() < 3){
            RCLCPP_ERROR(this->get_logger(), "Coverage command malformed, found only %zu points", coverage_area.size());
            command_running = false; 
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting coverage with %zu points", coverage_area.size());
        generate_boustrophedon_path();
        coverage_active = true;
        current_waypoint_index = 0;
        nav2_running = false;
        navigation_in_progress = false;
    }
    else if(op == "wait"){
        nav2_running = false; 
        coverage_active = false; 
        command_running = false; 
        current_command = "";
        navigation_in_progress = false;
    }
    else if(op == "stop" || op == "emergency_stop"){
        if(client_ptr_) client_ptr_->async_cancel_all_goals();
        geometry_msgs::msg::Twist tw; 
        tw.linear.x = tw.linear.y = tw.angular.z = 0.0;
        cmd_vel_pub_->publish(tw);
        nav2_running = false; 
        coverage_active = false; 
        command_running = false; 
        current_command = "";
        navigation_in_progress = false;
    }
    else if(op == "cancel" || op == "cancel_goal"){
        if(client_ptr_) client_ptr_->async_cancel_all_goals();
        nav2_running = false; 
        command_running = false; 
        current_command = "";
        navigation_in_progress = false;
    }
    else if(op == "return" || op == "return_to_base"){
        double x=0,y=0,yaw=0;
        if(parse_xyyaw(home_pose_,x,y,yaw)) {
            send_goto_goal(x,y,yaw);
        } else {
            try{
                auto tf = get_tf("map", home_pose_);
                tf2::Quaternion quat(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);
                double roll,pitch;
                tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                x=tf.transform.translation.x; 
                y=tf.transform.translation.y;
                send_goto_goal(x,y,yaw);
            } catch(...){ 
                command_running = false; 
            }
        }
    }
    else{ 
        RCLCPP_ERROR(this->get_logger(), "Unknown command: %s", op.c_str());
        command_running = false; 
        current_command = ""; 
    }
}

void RoverManager::nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
{
    goal_handle_ = future;
    if(!goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        navigation_in_progress = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }
}

void RoverManager::nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>, 
                                         const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}

void RoverManager::nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    navigation_in_progress = false;
    
    switch(result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            new_command = "";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            new_command = "";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            new_command = "";
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown result code");
            new_command = "";
            break;
    }

    // Se coverage attivo, passiamo al prossimo waypoint
    if(coverage_active){
        current_waypoint_index++;
        RCLCPP_INFO(this->get_logger(), "Moving to next waypoint %zu/%zu", current_waypoint_index, coverage_path.size());
        
        if(current_waypoint_index >= coverage_path.size()){
            RCLCPP_INFO(this->get_logger(), "Coverage completed!");
            coverage_active = false;
            command_running = false;
            current_command = "";
        }
    } else {
        // Per comandi normali (non coverage), resettiamo lo stato
        current_command = "";
        command_running = false;
        nav2_running = false;
    }
}

std::pair<double,double> RoverManager::rotate_point(const std::pair<double,double>& p, double angle){
    double x = p.first, y = p.second;
    double cos_a = cos(angle), sin_a = sin(angle);
    return { x*cos_a - y*sin_a, x*sin_a + y*cos_a };
}

std::vector<std::pair<double,double>> RoverManager::rotate_points(const std::vector<std::pair<double,double>>& pts, double angle){
    std::vector<std::pair<double,double>> rotated;
    for(auto &p : pts) rotated.push_back(rotate_point(p, angle));
    return rotated;
}

std::vector<double> RoverManager::find_polygon_intersections(const std::vector<std::pair<double,double>>& pts, double y){
    std::vector<double> intersections;
    size_t n = pts.size();
    for(size_t i=0;i<n;i++){
        auto p1 = pts[i];
        auto p2 = pts[(i+1)%n];
        if((p1.second <= y && y <= p2.second) || (p2.second <= y && y <= p1.second)){
            if(p1.second != p2.second){
                double t = (y - p1.second)/(p2.second - p1.second);
                double x = p1.first + t*(p2.first - p1.first);
                intersections.push_back(x);
            }
        }
    }
    return intersections;
}

void RoverManager::generate_boustrophedon_path(){
    if(coverage_area.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "Not enough points for coverage area");
        return;
    }
    
    // Per semplicità, usa angolo 0 (allineato con assi)
    double angle = 0.0;
    auto rotated_pts = rotate_points(coverage_area, -angle);

    // Calcola bounding box
    double min_x = rotated_pts[0].first, max_x = rotated_pts[0].first;
    double min_y = rotated_pts[0].second, max_y = rotated_pts[0].second;
    
    for(auto &p : rotated_pts){
        min_x = std::min(min_x, p.first);
        max_x = std::max(max_x, p.first);
        min_y = std::min(min_y, p.second);
        max_y = std::max(max_y, p.second);
    }

    coverage_path.clear();
    int line_count = 0;
    
    for(double y = min_y + swath_width/2; y <= max_y; y += swath_width){
        auto intersections = find_polygon_intersections(rotated_pts, y);
        if(intersections.size() >= 2){
            std::sort(intersections.begin(), intersections.end());
            double x_start = intersections.front();
            double x_end = intersections.back();
            
            if(line_count % 2 == 0){ // Andata
                coverage_path.push_back({x_start, y});
                coverage_path.push_back({x_end, y});
            } else { // Ritorno
                coverage_path.push_back({x_end, y});
                coverage_path.push_back({x_start, y});
            }
            line_count++;
        }
    }

    // Ruota i waypoint di nuovo all'orientamento originale
    for(auto &p : coverage_path){
        p = rotate_point(p, angle);
    }

    RCLCPP_INFO(this->get_logger(), "Generated coverage path with %zu waypoints", coverage_path.size());
}

void RoverManager::execute_coverage_step(){
    if(!coverage_active || navigation_in_progress) {
        return;
    }

    if(current_waypoint_index >= coverage_path.size()){
        RCLCPP_INFO(this->get_logger(), "Coverage completed!");
        coverage_active = false;
        command_running = false;
        current_command = "";
        return;
    }

    // Invia il goal corrente a NAV2
    auto wp = coverage_path[current_waypoint_index];
    RCLCPP_INFO(this->get_logger(), "Sending coverage waypoint %zu: (%f, %f)", 
                current_waypoint_index, wp.first, wp.second);
    
    if(send_goto_goal(wp.first, wp.second, 0.0)){
        navigation_in_progress = true;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}