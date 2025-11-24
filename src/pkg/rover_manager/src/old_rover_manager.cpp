#include "rover_manager.h"

RoverManager::RoverManager() : Node("rover_manager")
{
    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "rover/navigate_to_pose");

    current_command = "";
    new_command = "";
    command_running = false;
    nav2_running = false;
    coverage_active = false;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "seed_pdt_rover/command", 1,
        std::bind(&RoverManager::command_callback, this, std::placeholders::_1));
        
    rover_feedback_pb_ = this->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 1);
    
    // Publisher per avviare la coverage
    coverage_publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("/coverage_area", 10);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RoverManager::command_manager_callback, this));
        
    std::cout<<"rover_manager created"<<std::endl;
}


/**
*
* @param schemaInstance
* @return vettore di stringhe rappresentanti i parametri dello schema
* in versione prolog-like:
* EG.
*      vec[0]="nome schema"
*      vec[1]="primo parametro"
*      vec[2]="secondo parametro"
*      etc.
*
* eventuali parametri che siano essi stessi funtori vengono
* restituiti ugualmente come elemento del vettore
* EG.
*      vec[i]="fun1(fun2(x,y),z)"
*
* NOTE: This version also consider the [ ] as a list!
*/
std::vector<std::string> RoverManager::instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
//            ss>>c;
        }
        else if(c=='(' || c=='['){
            count++;
            app=app+c;
        }
        else if( ( c==')' || c==']' ) && count!=0){
            count--;
            app=app+c;
        }
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) {
        //check the \ character and split by it (added 01/12/2020 in seed 4.0)
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            //std::cout<<"INSTANCE TO VECTOR: "<<schemaInstance<<std::endl;
            while(std::getline(ss2, substr, '\\')){
                result.push_back(substr);
                //std::cout<<"split: "<<substr<<std::endl;
            }
        }
        else
            result.push_back(app);
    }
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}



geometry_msgs::msg::TransformStamped RoverManager::get_tf(std::string source_frame, std::string target_frame)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        //transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        transform_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
        //RCLCPP_INFO(this->get_logger(), "Got transform: %f, %f, %f",
        //            transform_stamped.transform.translation.x,
        //            transform_stamped.transform.translation.y,
        //            transform_stamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
        std::cout<<"Could not transform"<<std::endl;
    }

    return transform_stamped;
}



void RoverManager::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  new_command = msg->data.c_str();
}

void RoverManager::command_manager_callback()
{
    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }
    
    if(new_command != current_command){

      if(command_running){
        std::cout<<"rover_manager: switching from "<<current_command<<" to "<<new_command<<std::endl;

        if(nav2_running && current_command != "cancelling"){
	  std::cout<<"cancelling all nav2 goals "<<std::endl;
          this->client_ptr_->async_cancel_all_goals();
          current_command = "cancelling";
        }
        // Ferma anche la coverage se attiva
        else if(coverage_active){
            std::cout<<"stopping coverage mission"<<std::endl;
            coverage_active = false;
            // Pubblica un messaggio vuoto per fermare la coverage
            auto empty_polygon = geometry_msgs::msg::Polygon();
            coverage_publisher_->publish(empty_polygon);
        }
      }
      else{
        std::cout<<"rover_manager: starting "<<new_command<<" from scratch"<<std::endl;

        execute_command(new_command);
        current_command = new_command;
        command_running = true;
      }
    }
}

void RoverManager::execute_command(std::string cmd)
{
  cv = instance2vector(cmd);

  if ( cv[0] == "goto" ) {
      RCLCPP_INFO(this->get_logger(), "Rover going to %s", cv[1].c_str());
      std::cout<<"Rover going to "<< cv[1] <<std::endl;
      
      auto target_tf = get_tf("map", cv[1]);

      //get only the yaw!
      tf2::Quaternion quat(
        target_tf.transform.rotation.x,
        target_tf.transform.rotation.y,
        target_tf.transform.rotation.z,
        target_tf.transform.rotation.w);

      // Convert quaternion to RPY
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);

      // Convert back to quaternion
      tf2::Quaternion target_quat;
      target_quat.setRPY(0.0, 0.0, yaw);

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.pose.position.x = target_tf.transform.translation.x;
      goal_msg.pose.pose.position.y = target_tf.transform.translation.y;
      goal_msg.pose.pose.position.z = 0.0; //not used
      goal_msg.pose.pose.orientation.x = target_quat.x();
      goal_msg.pose.pose.orientation.y = target_quat.y();
      goal_msg.pose.pose.orientation.z = target_quat.z();
      goal_msg.pose.pose.orientation.w = target_quat.w();


      RCLCPP_INFO(this->get_logger(), "Sending goal to nav2");
      std::cout<<"\t going to "<<goal_msg.pose.pose.position.x<<", "<<goal_msg.pose.pose.position.y<<", "<<yaw<<std::endl;
      std::cout<<"\t quat "<<goal_msg.pose.pose.orientation.x<<", "<<goal_msg.pose.pose.orientation.y<<", "<<goal_msg.pose.pose.orientation.z<<", "<<goal_msg.pose.pose.orientation.w<<std::endl;

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&RoverManager::nav2_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&RoverManager::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&RoverManager::nav2_result_callback, this, std::placeholders::_1);

      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

nav2_running = true;
      coverage_active = false;
  } else if (cv[0] == "coverage") {
      RCLCPP_INFO(this->get_logger(), "Starting coverage mission");
      std::cout<<"Starting coverage mission with area: "<< cv[1] <<std::endl;
      
      // Il formato del comando è: coverage(area_name) oppure coverage(x1,y1,x2,y2,x3,y3,x4,y4)
      if (cv.size() >= 2) {
          if (start_coverage_mission(cv[1])) {
              coverage_active = true;
              nav2_running = false;
          } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to start coverage mission");
              command_running = false;
              current_command = "";
          }
      }
      
  } else if (cv[0] == "wait"){
      RCLCPP_INFO(this->get_logger(), "Rover waiting");
      coverage_active = false;
      nav2_running = false;
  } else {
      RCLCPP_INFO(this->get_logger(), "Rover stopped (command unknown)");
      coverage_active = false;
      nav2_running = false;
  }
}

bool RoverManager::start_coverage_mission(const std::string& area_spec)
{
    try {
        auto polygon_msg = geometry_msgs::msg::Polygon();
        
        // Controlla se area_spec è un nome di TF frame o coordinate dirette
        if (area_spec.find(",") != std::string::npos) {
            // Formato: x1,y1,x2,y2,x3,y3,x4,y4
            std::vector<std::string> coords;
            std::stringstream ss(area_spec);
            std::string token;
            
            while (std::getline(ss, token, ',')) {
                coords.push_back(token);
            }
            
            if (coords.size() == 8) {
                for (int i = 0; i < 8; i += 2) {
                    auto point = geometry_msgs::msg::Point32();
                    point.x = std::stof(coords[i]);
                    point.y = std::stof(coords[i+1]);
                    point.z = 0.0;
                    polygon_msg.points.push_back(point);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid coordinates format for coverage");
                return false;
            }
        } else {
            // Assume che area_spec sia un TF frame contenente un poligono
            try {
                auto area_tf = get_tf("map", area_spec);
                
                // Crea un poligono quadrato attorno al punto TF (esempio: 2x2 metri)
                float size = 2.0; // dimensione default
                if (cv.size() > 2) {
                    size = std::stof(cv[2]);
                }
                
                for (int i = 0; i < 4; ++i) {
                    auto point = geometry_msgs::msg::Point32();
                    float angle = i * M_PI / 2.0;
                    point.x = area_tf.transform.translation.x + size * cos(angle);
                    point.y = area_tf.transform.translation.y + size * sin(angle);
                    point.z = 0.0;
                    polygon_msg.points.push_back(point);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get TF for coverage area: %s", e.what());
                return false;
            }
        }
        
        // Pubblica il poligono per avviare la coverage
        coverage_publisher_->publish(polygon_msg);
        RCLCPP_INFO(this->get_logger(), "Coverage area published successfully");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error starting coverage mission: %s", e.what());
        return false;
    }
}

//void RoverManager::nav2_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
void RoverManager::nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
{
    goal_handle_ = future.get();
    if (!goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

//void RoverManager::nav2_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>,
//                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
void RoverManager::nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>,
                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    //RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)",
    //            feedback->current_pose.pose.position.x,
    //            feedback->current_pose.pose.position.y);
    
    //this is invoked continuously during the exxecution, for now we do nothing
}

void RoverManager::nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            RCLCPP_INFO(this->get_logger(), "Goal was reached");

            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
        case rclcpp_action::ResultCode::CANCELED: {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            
            current_command = "";
            command_running = false;
            nav2_running = false;
            
            break;
        }
        case rclcpp_action::ResultCode::ABORTED: {
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            
            //advise SEED that goal is not reachable
            std_msgs::msg::String failure_msg;
            failure_msg.data = instance2vector(current_command)[1] + ".unreachable";
            rover_feedback_pb_->publish(failure_msg);
            

            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
        default: {
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            
            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
    }

    //rclcpp::shutdown();
}







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverManager>();
    rclcpp::spin(node);
    
    // not stopping the robot!
    //node->all_stop();
    //sleep(1);
    
    rclcpp::shutdown();
    return 0;
}
