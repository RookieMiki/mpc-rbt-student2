#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Planning.hpp"

PlanningNode::PlanningNode() : rclcpp::Node("planning_node") {

    // Client for map (Ptáme se map serveru na mapu)
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Service for path (Tady my posloucháme, když někdo chce naplánovat trasu)
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path", 
        std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
    );
        
    // Publisher for path (Tudy budeme posílat hotovou trasu do RVizu)
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Connect to map server (Počkáme, než map server vůbec naběhne)
    while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the map service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for map service to appear...");
    }

    // Request map (Služba běží, pošleme asynchronní dotaz)
    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    
    // Asynchronní volání - jakmile mapa přijde, zavolá se funkce mapCallback
    auto future = map_client_->async_send_request(
        request, 
        std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1)
    );
    
    // Ukládání polohy robota
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Pokaždé, když robot pošle svou polohu, uložíme si ji
        current_robot_x_ = msg->pose.pose.position.x;
        current_robot_y_ = msg->pose.pose.position.y;
        has_robot_pose_ = true;
    });
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map; // Uložíme si mapu do naší proměnné map_
        RCLCPP_INFO(get_logger(), "Map successfully fetched! Resolution: %f", map_.info.resolution);
        
       // Dilatace se provede hned po načtení mapy
       dilateMap();     
    } 
    else {
        RCLCPP_ERROR(get_logger(), "Failed to fetch map!");
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO(get_logger(), "Prijat pozadavek na planovani trasy...");

    // Spustíme A* algoritmus s daty z požadavku
    aStar(request->start, request->goal);

    // Vyhladíme trasu
    smoothPath();

    // Pošleme hotovou trasu do odpovědi služby
    response->plan = path_;

    // Zároveň ji publikujeme do topicu pro RViz, abychom to viděli namalované
    path_pub_->publish(path_);
    
    RCLCPP_INFO(get_logger(), "Trasa odeslana. Pocet bodu: %zu", path_.poses.size());
}

void PlanningNode::dilateMap() {
    // Vytvoříme si kopii mapy, do které budeme zapisovat nafouknuté překážky
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    // Zjistíme rozměry mapy z jejích metadat
    int width = map_.info.width;
    int height = map_.info.height;
    float resolution = map_.info.resolution;

    // Spočítáme, o kolik buněk (pixelů) chceme překážky zvětšit
    double robot_radius = robot_config::HALF_DISTANCE_BETWEEN_WHEELS + 0.1; // Poloměr robota v metrech + nějaká rezerva
    int inflation_cells = std::ceil(robot_radius / resolution);

    // Projdeme úplně všechny buňky (pixely) v původní mapě
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            
            // Převod 2D souřadnic (x,y) na pozici v 1D poli
            int index = y * width + x;

            // Pokud na tomto políčku v původní mapě leží překážka (hodnota 100)
            if (map_.data[index] == 100) {
                
                // Uděláme kolem ní čtverec o velikosti inflation_cells a ten celý vybarvíme
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        
                        int nx = x + dx;
                        int ny = y + dy;

                        // Musíme si pohlídat, abychom nekreslili mimo okraje mapy!
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            // Opět převod 2D souřadnic na 1D index, tentokrát pro sousední buňku
                            int n_index = ny * width + nx;
                            
                            // Zápis překážky (100) do naší KOPIE mapy
                            dilatedMap.data[n_index] = 100; 
                        }
                    }
                }
            }
        }
    }

    // Hotovo! Přepíšeme originální mapu tou naší novou, nafouknutou
    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Mapa byla uspesne rozsirena (dilatovana)!");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    
    // Očistíme předchozí trasu, pokud nějaká byla
    path_.poses.clear();

    // Potřebujeme informace o mapě pro přepočet (rozlišení a kde má počátek)
    float res = map_.info.resolution;
    float origin_x = map_.info.origin.position.x;
    float origin_y = map_.info.origin.position.y;
    int width = map_.info.width;
    int height = map_.info.height;

    // Kontrola, zda už robot nějakou polohu vůbec poslal
    if (!has_robot_pose_) {
    	RCLCPP_WARN(this->get_logger(), "Zatim nemam pozici robota, nemuzu planovat!");
    	return; 
    }

    // Převod ze souřadnic reálného světa (metry) na souřadnice v mapě (indexy buněk)
    int start_x = (current_robot_x_ - origin_x) / res;
    int start_y = (current_robot_y_ - origin_y) / res;

    int goal_x = (goal.pose.position.x - origin_x) / res;
    int goal_y = (goal.pose.position.y - origin_y) / res;

    // Kontrola, jestli Start nebo Cíl nejsou úplně mimo mapu
    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        RCLCPP_ERROR(get_logger(), "Start nebo Cil je mimo mapu!");
        return;
    }

    // Vytvoříme počáteční a cílovou buňku pomocí Cell konstruktoru
    Cell cStart(start_x, start_y);
    Cell cGoal(goal_x, goal_y);

    // Příprava seznamů pro A*
    // openList = políčka, která chceme prozkoumat
    std::vector<std::shared_ptr<Cell>> openList;
    // closedList = mapa (1D pole boolů), kde si značíme políčka, kde už jsme byli
    std::vector<bool> closedList(width * height, false);

    // Začínáme na Startu! Přidáme ho do seznamu k prozkoumání.
    openList.push_back(std::make_shared<Cell>(cStart));

    RCLCPP_INFO(get_logger(), "A* pripraven! Hledam trasu z [%d, %d] do [%d, %d] v pixelech.", 
                start_x, start_y, goal_x, goal_y);
    
    // Pomocná pole pro pohyb (8 směrů: nahoru, dolu, doleva, doprava a 4 diagonály)
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};


    while (!openList.empty() && rclcpp::ok()) {      
        // Najdeme buňku s nejmenším skóre 'f' v openListu 
        auto current_it = openList.begin();
        for (auto it = openList.begin(); it != openList.end(); ++it) {
            if ((*it)->f < (*current_it)->f) {
                current_it = it;
            }
        }
        std::shared_ptr<Cell> current = *current_it;
        openList.erase(current_it); // Odebereme ji z openListu, jdeme ji prozkoumat

        // Přidáme ji do closedListu (abychom se na ni už nevraceli)
        int current_index = current->y * width + current->x;
        closedList[current_index] = true;


        // Jsme v cíli?
        if (current->x == goal_x && current->y == goal_y) {
            RCLCPP_INFO(get_logger(), "Cil nalezen! Rekonstruuji trasu...");
            
            // Nyní poskládáme trasu pozpátku (od cíle ke startu pomocí ukazatelů 'parent')
            std::vector<std::shared_ptr<Cell>> path_cells;
            std::shared_ptr<Cell> curr = current;
            while (curr != nullptr) {
                path_cells.push_back(curr);
                curr = curr->parent;
            }
            // Otočíme ji, aby byla od startu do cíle
            std::reverse(path_cells.begin(), path_cells.end());

            // Převedeme zpět pixely na reálné metry (do nav_msgs::msg::Path)
            path_.header.frame_id = "map"; // Trasa je v souřadném systému mapy
            for (auto &cell : path_cells) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                
                // Posuneme se zpět na střed buňky (+ res/2) a přičteme offset mapy
                pose.pose.position.x = (cell->x * res) + origin_x + (res / 2.0);
                pose.pose.position.y = (cell->y * res) + origin_y + (res / 2.0);
                path_.poses.push_back(pose);
            }
            
            return; 
        }


        //  Prozkoumáme sousedy do 8 směrů 
        for (int i = 0; i < 8; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            // Kontrola okrajů mapy
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            int n_index = ny * width + nx;

            // Kontrola překážek a closedListu (mapa v ROS má překážky > 50)
            if (map_.data[n_index] > 50 || closedList[n_index]) continue;


            // Výpočet skóre 
            // Pohyb rovně (i < 4) stojí 1, diagonálně (i >= 4) stojí sqrt(2)
            float step_cost = (i < 4) ? 1.0 : sqrt(2); 
            float tentative_g = current->g + step_cost;

            // Zjistíme, jestli už soused není v openListu s nějakou horší cestou
            bool in_open = false;
            for (auto &open_node : openList) {
                if (open_node->x == nx && open_node->y == ny) {
                    in_open = true;
                    // Našli jsme kratší cestu k tomuto políčku?
                    if (tentative_g < open_node->g) {
                        open_node->g = tentative_g;
                        open_node->f = open_node->g + open_node->h;
                        open_node->parent = current; // Přepíšeme kudy jsme sem přijeli
                    }
                    break;
                }
            }

            // Pokud políčko ještě vůbec není v openListu, přidáme ho
            if (!in_open) {
                std::shared_ptr<Cell> neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = tentative_g;
                
                // Heuristika 'h': Euklidovská vzdálenost k cíli vzdušnou čarou
                neighbor->h = std::sqrt(std::pow(goal_x - nx, 2) + std::pow(goal_y - ny, 2));
                neighbor->f = neighbor->g + neighbor->h;
                
                neighbor->parent = current;
                openList.push_back(neighbor);
            }       
    	}
    }
    // Pokud se cyklus vyčerpá a my cíl nenašli, vyhodíme chybu
    RCLCPP_ERROR(get_logger(), "Unable to plan path. Cil neni dostupny.");
}

void PlanningNode::smoothPath() {
    // Pokud má trasa méně než 3 body, není co vyhlazovat
    if (path_.poses.size() < 3) {
        return;
    }

    // Nastavení žehličky
    int smoothing_iterations = 25; // Kolikrát trasu projedeme (čím víc, tím kulatější)
    float weight_smooth = 0.25;    // Jak silně bod přitahujeme k sousedům 

    // Vytvoříme si kopii trasy, kterou budeme upravovat
    std::vector<geometry_msgs::msg::PoseStamped> new_path = path_.poses;

    for (int iter = 0; iter < smoothing_iterations; ++iter) {
        // Začínáme od indexu 1 a končíme před posledním (start a cíl nesmíme posunout!)
        for (size_t i = 1; i < new_path.size() - 1; ++i) {
            
            // Rovnice pro vyhlazení osy X
            new_path[i].pose.position.x = new_path[i].pose.position.x + 
                weight_smooth * (new_path[i-1].pose.position.x + new_path[i+1].pose.position.x - 2.0 * new_path[i].pose.position.x);
            
            // Rovnice pro vyhlazení osy Y
            new_path[i].pose.position.y = new_path[i].pose.position.y + 
                weight_smooth * (new_path[i-1].pose.position.y + new_path[i+1].pose.position.y - 2.0 *   new_path[i].pose.position.y);
        }
    }

    // Přepíšeme původní zubatou trasu tou naší vyhlazenou
    path_.poses = new_path;
    RCLCPP_INFO(get_logger(), "Trasa byla uspesne vyhlazena!");
}

Cell::Cell(int c, int r) {
    x = c; // cols
    y = r; // rows
    f = 0.0; // Celkové skóre -> g+h
    g = 0.0; // Cesta od startu na současné políčko
    h = 0.0; // Odhad ze současného políčka do cíle
    parent = nullptr; // Zatím nevíme, odkud jsme na toto políčko přijeli
}
