#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Klient pro plánovací službu z minulého cvičení
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path"); // Ověř si přesný název tvé služby!

        // Action server /go_to_goal podle zadání
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "/go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server (Čekání na službu)
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Přerušeno čekání na plánovací službu.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Čekám na spuštění plánovací služby...");
        }
    }

void MotionControlNode::checkCollision() {
    // Ochrana: Zkontrolujeme, zda vůbec máme platná data z lidaru
    if (laser_scan_.ranges.empty()) {
        return; 
    }

    double emergency_stop_threshold = 0.25; // Zvětšíme trochu rezervu na 25 cm
    bool collision_imminent = false;

    // Procházíme všechny paprsky
    for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
        double distance = laser_scan_.ranges[i];
        
        // Ignorujeme nuly a nekonečno
        if (std::isnan(distance) || std::isinf(distance) || distance <= 0.01) continue;

        // VÝPOČET ÚHLU paprsku i
        double angle = laser_scan_.angle_min + (i * laser_scan_.angle_increment);

        // Kontrolujeme jen kužel před robotem (např. -0.4 až +0.4 radiánu, což je cca +/- 23 stupňů)
        if (std::abs(angle) < 0.4) { 
            if (distance < emergency_stop_threshold) {
                collision_imminent = true;
                break; 
            }
        }
    }

    // Akce při hrozící kolizi
    if (collision_imminent) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Nouzove zastaveni! Prekazka v koliznim kurzu.");

        // OKAMŽITĚ posíláme nulu do motorů
        geometry_msgs::msg::Twist stop_twist;
        twist_publisher_->publish(stop_twist);
        
        // Pokud právě běží navigační akce, musíme ji ukončit s chybou (ABORTED)
        if (goal_handle_ && goal_handle_->is_active()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            
            // Vyčistíme trasu, aby se robot po odblokování nerozjel dál
            path_.poses.clear();
        }
    }
}

void MotionControlNode::updateTwist() {
    // Ochrana: Zkontrolujeme, jestli vůbec máme nějakou trasu k cíli
    if (path_.poses.empty()) {
        return; 
    }

    // Získáme aktuální polohu robota
    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;
    
    // Získání natočení (Yaw) robota je složitější, protože ROS používá kvaterniony.
    // Převedeme kvaternion z 'current_pose_' na klasické Eulerovy úhly (roll, pitch, yaw).
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw); // Nás zajímá jen 'robot_yaw'

    // Najdeme cílový bod na trase (tzv. Look-ahead point)
    // Pokud bychom brali hned první bod (index 0), robot by se zbytečně kroutil.
    // Vezmeme bod kousek před ním, například s indexem 5 (nebo ten úplně poslední, pokud je trasa kratší)
    int target_index = std::min(5, (int)path_.poses.size() - 1);
    double target_x = path_.poses[target_index].pose.position.x;
    double target_y = path_.poses[target_index].pose.position.y;

    // Výpočet odchylky (Error)
    // Spočítáme úhel, pod kterým leží náš cílový bod z pohledu robota
    double angle_to_target = atan2(target_y - robot_y, target_x - robot_x);
    
    // Rozdíl mezi tím, kam se robot reálně dívá, a kam by se dívat měl (naše odchylka XTE)
    double xte = angle_to_target - robot_yaw;
    
    // KRITICKÝ KROK: Normalizace úhlu na interval [-PI, PI]. 
    // Pokud je rozdíl třeba 359 stupňů, nechceme, aby se robot točil dokola, 
    // ale chceme, aby si uvědomil, že je to vlastně jen -1 stupeň.
    while (xte > M_PI) xte -= 2.0 * M_PI;
    while (xte < -M_PI) xte += 2.0 * M_PI;

    // Výpočet povelů pro motory (přesně podle nápovědy v kostře)
    double P = 1.0;     // Proporcionální konstanta - určuje agresivitu zatáčení (možná budeš muset ladit)
    double v_max = 0.2; // Rychlost jízdy vpřed [m/s]
    
    geometry_msgs::msg::Twist twist;
    twist.angular.z = P * xte;
    twist.linear.x = v_max;

    // (Volitelné) Jednoduchá stopka, pokud jsme dostatečně blízko úplného konce trasy
    double final_target_x = path_.poses.back().pose.position.x;
    double final_target_y = path_.poses.back().pose.position.y;
    double dist_to_final = hypot(final_target_x - robot_x, final_target_y - robot_y);
    
    if (dist_to_final < 0.2) { // Jsme méně než 20 cm od cíle?
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    // Odešleme vypočítanou rychlost robotovi
    twist_publisher_->publish(twist);
    
    // Mazání projetých bodů
    // Když se robot k našemu zvolenému bodu dostatečně přiblíží, umažeme projeté body ze začátku pole, 
    // aby se jako další v pořadí (target_index) vybral bod zase kousek dál po trase.
    if (hypot(target_x - robot_x, target_y - robot_y) < 0.3) {
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 1);
    }
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Zaznamenán požadavek na nový cíl.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Vždy rovnou přijmout
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Zaznamenán požadavek na zrušení cíle.");
    return rclcpp_action::CancelResponse::ACCEPT; // Povolit zrušení
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // Uložíme si handle aktuálního cíle, abychom k němu měli přístup jinde
    goal_handle_ = goal_handle;

    RCLCPP_INFO(this->get_logger(), "Cíl přijat, žádám o naplánování trasy...");

    // Vytvoříme požadavek pro službu GetPlan
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    
    // Vytvoříme novou PoseStamped zprávu
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map"; // nebo "odom", podle toho, v čem plánuješ
    start_pose.pose = current_pose_.pose; // Zde vložíme reálnou polohu robota z odometrie

    request->start = start_pose; 
    request->goal = goal_handle->get_goal()->pose;

    // Asynchronní odeslání požadavku na plánovací službu
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(1.0);
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    // Čekáme max 5 sekund na to, až nám plánovač dodá trasu do path_
    int wait_count = 0;
    rclcpp::Rate wait_rate(10.0); // 10 Hz
    while (path_.poses.empty() && rclcpp::ok() && wait_count < 50) {
        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            return;
        }
        wait_rate.sleep();
        wait_count++;
    }

    if (path_.poses.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Plánovač nedodal trasu ani po 5 vteřinách. Abortuji.");
        goal_handle_->abort(result);
        return;
    }

    // Pokud jsme se dostali sem, trasa existuje. Můžeme si uložit koncový bod.
    double final_x = path_.poses.back().pose.position.x;
    double final_y = path_.poses.back().pose.position.y;

    while (rclcpp::ok()) {
        // Kontrola, jestli nebyl cíl zrušen uživatelem (např. Ctrl+C v terminálu)
        if (goal_handle_->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Cíl byl zrušen! Zastavuji robota.");
            
            // Okamžitě pošleme do motorů nulu
            geometry_msgs::msg::Twist stop_twist;
            twist_publisher_->publish(stop_twist);
            
            // KRITICKÝ KROK: Vymažeme trasu, aby updateTwist() dál neřídil
            path_.poses.clear();
            
            // Nahlásíme Action klientovi, že jsme akci oficiálně ukončili jako zrušenou
            goal_handle_->canceled(result);
            return;
        }

        double dist_to_goal = hypot(final_x - current_pose_.pose.position.x, 
                                    final_y - current_pose_.pose.position.y);

        // Zvětšíme toleranci v execute na 0.3 m, aby byla "volnější" než v updateTwist (0.2 m)
        if (dist_to_goal < 0.3) {
            RCLCPP_INFO(this->get_logger(), "Cíl potvrzen Action Serverem.");
            
            // Zastavení a vyčištění
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            path_.poses.clear();

            // TOTO UKONČÍ TERMINÁL UŽIVATELE
            goal_handle_->succeed(result); 
            return;
        }

        // Odeslání průběžných informací (Feedback)
        auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        feedback->current_pose = current_pose_;
        feedback->distance_remaining = dist_to_goal;
        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    
    // Zkontrolujeme, zda se trasa našla a není prázdná
    if (response && response->plan.poses.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Plán úspěšně vytvořen! Obsahuje %zu bodů. Spouštím execute().", response->plan.poses.size());
        
        // Uložíme si naplánovanou trasu pro naši funkci updateTwist
        path_ = response->plan;
        
        // Spuštění smyčky pro řízení pohybu ve vlastním vlákně (podle zadání)
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Plánovač nedokázal najít cestu k cíli!");
        
        // Ukončíme akci s chybou
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // Jen si uložíme poslední sken z lidaru do třídní proměnné
    laser_scan_ = msg;
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // Uložíme si aktuální pózu robota z odometrie
    current_pose_.pose = msg.pose.pose;

    // A jak ti radí nápověda v kostře, při každém novém pohybu:
    // 1. Zkontrolujeme, jestli nenabouráme
    checkCollision(); 
    
    // 2. Přepočítáme a pošleme rychlost motorům, abychom se drželi trasy
    // (Tohle zatím necháme zakomentované, než napíšeme logiku v další fázi!)
    updateTwist(); 
}
