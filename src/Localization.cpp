#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {
    
    odometry_.pose.pose.orientation.w = 1.0; // Robot kouká dopředu, kvaternion je platný
    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;

    // Odometry message initialization
    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    // Subscriber for joint_states
    // Propojíme náš připravený subscriber s topikem "/joint_states"
    // Říkáme mu: "Když přijdou data, zavolej funkci jointCallback"
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 
        10, // Velikost fronty (kolik zpráv si pamatuje, když nestíhá)
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );
    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

/*
void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    static rclcpp::Time last_time = msg.header.stamp;
    rclcpp::Time current_time = msg.header.stamp;
    
    double right_vel = msg.velocity[0]; 
    double left_vel = msg.velocity[1];
    
    // Výpočet skutečného dt mezi zprávami
    double dt = (current_time - last_time).seconds();
    
    // Ochrana pro první spuštění 
    if (dt <= 0.0) {
        dt = 0.01; 
    }
    
    //RCLCPP_INFO(this->get_logger(), "L: %.2f, R: %.2f, dt: %.4f", msg.velocity[0], msg.velocity[1], dt);

    // Výpočty
    updateOdometry(left_vel, right_vel, dt, msg.header.stamp);
    
    // Důležité: uložíme si čas pro příští výpočet
    last_time = current_time;

    publishOdometry();
    publishTransform();
}
*/

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    rclcpp::Time current_time = msg.header.stamp;

    // 1. Ochrana pro úplně první zprávu po startu nodu
    // Pokud je last_time_ nula (přednastavená hodnota), jen ji uložíme a končíme
    if (last_time_.nanoseconds() == 0) {
        last_time_ = current_time;
        return;
    }

    // 2. Výpočet skutečného dt
    double dt = (current_time - last_time_).seconds();

    // 3. Ochrana proti restartu simulace nebo chybám v čase
    // Pokud je dt záporné (restart) nebo podezřele malé, přeskočíme krok
    if (dt <= 0.0) {
        last_time_ = current_time;
        return;
    }
    
    // Ochrana proti extrémně velkému dt (např. lag v simulaci)
    if (dt > 0.5) { 
        dt = 0.01; // Nouzová hodnota, aby robot neodletěl pryč
    }

    double right_vel = msg.velocity[0]; 
    double left_vel = msg.velocity[1];

    // Výpočty odometrie
    updateOdometry(left_vel, right_vel, dt, msg.header.stamp);
    
    // Uložíme čas pro příští výpočet (používáme členskou proměnnou třídy)
    last_time_ = current_time;

    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt, rclcpp::Time stamp) {
    odometry_.header.stamp = stamp; 
    // Výpočet dopředné a úhlové rychlosti robota
    // robot_config::WHEEL_RADIUS je poloměr 'r'
    // robot_config::HALF_DISTANCE_BETWEEN_WHEELS je 'b/2'
    double linear = robot_config::WHEEL_RADIUS * (right_wheel_vel + left_wheel_vel) / 2.0;
    double angular = robot_config::WHEEL_RADIUS * (right_wheel_vel - left_wheel_vel) / (2.0 *        	   robot_config::HALF_DISTANCE_BETWEEN_WHEELS);
 
    // Získání aktuální orientace (theta) z poslední uložené odometrie
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    // Integrace - výpočet nové pozice a úhlu
    //double delta_x = linear * std::cos(theta) * dt;
    //double delta_y = linear * std::sin(theta) * dt;
    double delta_theta = angular * dt;
    
    // Místo std::cos(theta) použij střední úhel:
    double avg_theta = theta + (delta_theta / 2.0); 
    double delta_x = linear * std::cos(avg_theta) * dt;
    double delta_y = linear * std::sin(avg_theta) * dt;

    double new_x = odometry_.pose.pose.position.x + delta_x;
    double new_y = odometry_.pose.pose.position.y + delta_y;
    double new_theta = theta + delta_theta;

    // Normalizace úhlu (aby byl vždy v rozsahu -PI až PI)
    new_theta = std::atan2(std::sin(new_theta), std::cos(new_theta));

    // Uložení výsledků zpět do členské proměnné odometry_
    odometry_.pose.pose.position.x = new_x;
    odometry_.pose.pose.position.y = new_y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, new_theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    // Nastavení rychlostí do zprávy (aby RViz věděl, jak rychle jedeme)
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    // Čas a frame už máme nastavený z updateOdometry, 
    // ale pro jistotu je můžeme aktualizovat i tady:
    odometry_.header.stamp = odometry_.header.stamp;
    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    // Publikujeme jen tu jednu správnou zprávu
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    // DŮLEŽITÉ: čas transformace musí přesně odpovídat času v odometrii
    t.header.stamp = odometry_.header.stamp; 
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
