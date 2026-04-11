# Laboratorní úloha číslo 8 - Behavior Trees

Cílem tohoto cvičení je navrhnout a implementovat misi pro mobilní robot ve skladovém prostředí pomocí behavior tree (BT). Studenti se naučí pracovat s knihovnou [BehaviorTree.CPP v4](https://www.behaviortree.dev/) a její integrací s ROS 2 pomocí balíku [behaviortree_ros2](https://github.com/BehaviorTree/BehaviorTree.ROS2). Výsledkem bude BT server, který orchestruje kompletní logistickou misi: robot si vyžádá úkol od warehouse manageru, naviguje k přiřazenému manipulátoru, počká na naložení, zjistí cílový sklad a doručí náklad.

Studenti využijí navigační akci `NavigateToPose` implementovanou v předchozím cvičení (Lab 07) jako stavební blok a propojí ji s dalšími BT nody volanými přes ROS 2 služby.

## Cíl cvičení
Výsledkem cvičení je:
  1) BT server node, který načítá a spouští behavior tree definovaný v XML souboru.
  2) BT akční node `NavigateToPoseAction`, který volá existující ROS 2 akci `/go_to_goal`.
  3) BT servisní nody (`GetTaskService`, `ConfirmLoadingService`, `GetDropoffService`), které komunikují s warehouse managerem.
  4) BT synchronní node `LookupPose`, který mapuje ID manipulátoru/skladu na souřadnice.
  5) XML soubor definující kompletní logistickou misi.
  6) Vizualizace a ověření funkčnosti v RVizu.

## Scénář mise

Robot vykonává logistickou misi ve skladu se **3 manipulátory** (pickup pozice 1–3) a **4 skladovými pozicemi** (A–D), každá se 2 subskladovými pozicemi (1, 2). Celkem 8 možných dropoff pozic.

**Souřadnice manipulátorů** (všechny na x=4.5):

| ID | X | Y |
|----|---|---|
| 1 | 4.5 | 1.5 |
| 2 | 4.5 | -0.5 |
| 3 | 4.5 | -2.5 |

**Souřadnice skladů** (každý sklad má 2 pozice, y=0.5 a y=-1.5):

| ID | X | Y |
|----|---|---|
| A1 | 1.5 | 0.5 |
| A2 | 1.5 | -1.5 |
| B1 | -0.5 | 0.5 |
| B2 | -0.5 | -1.5 |
| C1 | -2.5 | 0.5 |
| C2 | -2.5 | -1.5 |
| D1 | -4.5 | 0.5 |
| D2 | -4.5 | -1.5 |

**Pipeline mise:**
1. Robot stojí na startovní pozici (0, 0).
2. Zavolá službu `/get_pickup_task` → dostane ID manipulátoru (1–3).
3. Převede ID na souřadnice (node `LookupPose`).
4. Naviguje k manipulátoru.
5. Zavolá službu `/confirm_loading` → čeká na dokončení nakládky (2–5 s).
6. Zavolá službu `/get_dropoff_location` → dostane ID skladu (např. "B2").
7. Převede ID na souřadnice.
8. Naviguje do skladu a vyloží náklad.
9. Vrátí se na startovní pozici.

## Architektura systému

Řešení se skládá z následujících komponent:

### Warehouse Manager (poskytnuto)
ROS 2 node (`warehouse_manager`), který simuluje skladový systém. Poskytuje 3 služby typu `std_srvs/srv/Trigger`:

| Služba | Chování |
|--------|---------|
| `/get_pickup_task` | Vrátí `message="1"` až `"3"` (náhodný manipulátor) |
| `/confirm_loading` | Čeká 2–5 s (náhodná doba), pak `success=true` |
| `/get_dropoff_location` | Vrátí `message="A1"` až `"D2"` (náhodný sklad) |

### BT Server
Node založený na `BT::TreeExecutionServer` z balíku `behaviortree_ros2`. Načítá BT pluginy (sdílené knihovny) a XML soubory s definicemi stromů. Přijímá požadavky na spuštění mise přes ROS 2 akci `ExecuteTree`.

### BT Pluginy (nody)
Každý BT node je zkompilován jako samostatná sdílená knihovna (.so) a dynamicky načten BT serverem:

| Node | Typ | Účel |
|------|-----|------|
| `NavigateToPoseAction` | `BT::RosActionNode` | Volá akci `/go_to_goal` |
| `GetTaskService` | `BT::RosServiceNode` | Volá `/get_pickup_task`, výstup: `manipulator_id` |
| `ConfirmLoadingService` | `BT::RosServiceNode` | Volá `/confirm_loading` |
| `GetDropoffService` | `BT::RosServiceNode` | Volá `/get_dropoff_location`, výstup: `storage_id` |
| `LookupPose` | `BT::SyncActionNode` | Převádí ID (např. "2", "B1") na souřadnice x, y |

### XML Behavior Tree
Soubor `behavior_trees/warehouse_mission.xml` definuje sekvenci mise. Data se předávají mezi nody přes **blackboard** pomocí portů v zápisu `{název_proměnné}`.

## Poskytnuté soubory
Tyto soubory jsou připravené a **nemusíte je implementovat**:

| Soubor | Popis |
|--------|-------|
| `src/bt/WarehouseManager.cpp` | Node poskytující 3 služby |
| `src/bt/warehouse_manager_node.cpp` | Entry point pro warehouse manager |
| `include/mpc_rbt_solution/bt/WarehouseManager.hpp` | Hlavičkový soubor |
| `config/bt_server.yaml` | Konfigurace BT serveru (cesty k pluginům a XML) |

## Domácí příprava

> [!WARNING]
> Zadání je spíše časově náročné, minimální nutná podmínka je teoretické pochopení behavior trees a základní znalost práce se systémem ROS 2. Doporučuji si řešení částečně připravit doma.

> [!CAUTION]
> Na konci cvičení bude práce ohodnocena až **5 body**!

### Pochopení Behavior Trees
Behavior tree je stromová struktura pro řízení rozhodování autonomních agentů. Nastudujte si základní principy:

- Co jsou uzly typu **Action**, **Condition**, **Sequence**, **Fallback** (Selector) a **Decorator**?
- Jaký je rozdíl mezi návratovými hodnotami `SUCCESS`, `FAILURE` a `RUNNING`?
- Jak funguje **tick** a v jakém pořadí jsou uzly vyhodnocovány?
- Jak se liší **Sequence** od **ReactiveSequence**?
- Co je **Blackboard** a jak se předávají data mezi uzly pomocí portů?

Doporučené zdroje:
- [BehaviorTree.CPP dokumentace](https://www.behaviortree.dev/docs/intro)
- [BT.CPP v4 tutoriály](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree)

### Integrace BehaviorTree.CPP s ROS 2
Balík `behaviortree_ros2` poskytuje šablonové třídy, které zjednodušují napojení BT nodů na ROS 2:

- **`BT::RosActionNode<ActionType>`** – wrapper pro volání ROS 2 akcí z BT. Vyžaduje implementaci metod:
  - `setGoal(Goal& goal)` – naplnění požadavku akce z input portů
  - `onResultReceived(WrappedResult& wr)` – zpracování výsledku
  - `onFailure(ActionNodeErrorCode error)` – obsluha chyby
  - `onFeedback(const Feedback* fb)` – zpracování průběžné zpětné vazby

- **`BT::RosServiceNode<ServiceType>`** – wrapper pro volání ROS 2 služeb z BT. Vyžaduje implementaci metod:
  - `setRequest(Request::SharedPtr& request)` – naplnění požadavku
  - `onResponseReceived(Response::SharedPtr& response)` – zpracování odpovědi
  - `onFailure(ServiceNodeErrorCode error)` – obsluha chyby

- **`BT::SyncActionNode`** – synchronní BT node pro okamžité operace (vyhledání v tabulce apod.).

- **`BT::TreeExecutionServer`** – hotový server, který přijímá požadavky na spuštění BT přes ROS 2 akci.

### Plugin architektura
BehaviorTree.CPP v4 podporuje dynamické načítání nodů jako pluginů (shared libraries). Každý vlastní node je zkompilovaný jako sdílená knihovna a zaregistrován pomocí makra:

```cpp
// Pro ROS 2 action/service nody:
CreateRosNodePlugin(MyActionNode, "MyActionNodeName");

// Pro ostatní BT nody (SyncActionNode apod.):
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<MyNode>("MyNodeName");
}
```

### Práce s porty a blackboardem
Porty slouží k předávání dat mezi BT nody přes sdílenou paměť (blackboard):

```cpp
// Definice portů
static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("x", "Cílová pozice X"),
        BT::OutputPort<std::string>("result_id", "Výsledné ID")
    };
}

// Čtení z input portu
auto val = getInput<double>("x");
if (!val) throw BT::RuntimeError("Missing port: ", val.error());

// Zápis do output portu
setOutput("result_id", some_string);
```

V XML se porty propojují přes blackboard proměnné pomocí `{název}`:
```xml
<GetTaskService service_name="/get_pickup_task"
                manipulator_id="{manip_id}"/>
<LookupPose location_id="{manip_id}"
            x="{goal_x}" y="{goal_y}"/>
<NavigateToPoseAction action_name="/go_to_goal"
                      x="{goal_x}" y="{goal_y}"/>
```

## Hodnocení cvičení
> [!WARNING]
> Cvičení nebude hodnoceno jen na základě funkčnosti. Bude hodnocen i zdrojový kód, návrh behavior tree a pochopení problematiky. Doporučuji se připravit na témata z domácí přípravy.

Budete upravovat připravené kostry souborů v `src/bt/` a `behavior_trees/warehouse_mission.xml`.

## Instalace závislostí

Balíky `behaviortree_cpp` a `behaviortree_ros2` nejsou dostupné přes `apt` pro ROS 2 Humble. Je potřeba je zbuildovat ze zdrojových kódů.

Naklonujte repozitář `BehaviorTree.ROS2` do workspace (pokud tam ještě není):

```bash
cd ~/mpc-rbt_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
```

> [!NOTE]
> Repozitář `BehaviorTree.ROS2` obsahuje jak `behaviortree_ros2`, tak `btcpp_ros2_interfaces`. Závislost `behaviortree_cpp` se nainstaluje automaticky jako systémový balík při buildu (je uvedena v `package.xml` daného balíku). Pokud ne, doinstalujte ji:
> ```bash
> sudo apt install ros-humble-behaviortree-cpp
> ```

Zbuildujte celý workspace:

```bash
cd ~/mpc-rbt_ws
colcon build
source install/setup.bash
```

## Doporučený postup

### 1) Úprava launch souboru

Přidejte do souboru `launch/solution.launch.py` spouštění nodů `warehouse_manager` a `bt_server`. Warehouse manager je běžný ROS 2 node, BT server potřebuje navíc načíst konfigurační soubor `config/bt_server.yaml`:

```python
warehouse_manager = Node(
    package='mpc_rbt_solution',
    executable='warehouse_manager',
    name='warehouse_manager',
    output='screen',
    parameters=[{'use_sim_time': True}]
)

bt_server = Node(
    package='mpc_rbt_solution',
    executable='bt_server',
    name='bt_server',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        os.path.join(package_dir, 'config', 'bt_server.yaml')
    ]
)
```

Nezapomeňte oba nody přidat do `LaunchDescription`.

> [!IMPORTANT]
> Klíč v souboru `config/bt_server.yaml` musí odpovídat parametru `name=` v launch souboru. Výchozí nastavení používá `bt_server` pro obojí.

### 2) Ověření warehouse manageru

Zbuildujte projekt a spusťte warehouse manager, abyste ověřili, že služby fungují:

```bash
# Terminál 1:
ros2 run mpc_rbt_solution warehouse_manager
```

```bash
# Terminál 2 – otestujte všechny 3 služby:
ros2 service call /get_pickup_task std_srvs/srv/Trigger
ros2 service call /confirm_loading std_srvs/srv/Trigger
ros2 service call /get_dropoff_location std_srvs/srv/Trigger
```

Očekávaný výstup:
- `get_pickup_task`: `success=true`, `message="1"` až `"3"`
- `confirm_loading`: `success=true` (po 2–5 s čekání), `message="Loading complete"`
- `get_dropoff_location`: `success=true`, `message="A1"` až `"D2"`

### 3) Implementace NavigateToPoseAction

Soubor: `src/bt/NavigateToPoseAction.cpp`

Tento node volá existující ROS 2 akci `/go_to_goal` (z Lab 07). Dědí z `BT::RosActionNode<nav2_msgs::action::NavigateToPose>`.

Implementujte:
- **`providedPorts()`** – definujte input porty `x` a `y` (typ `double`)
- **`setGoal(Goal& goal)`** – načtěte souřadnice z input portů a naplňte goal

> [!TIP]
> ```cpp
> bool setGoal(Goal& goal) override {
>     auto x = getInput<double>("x");
>     auto y = getInput<double>("y");
>     if (!x || !y) return false;
>     goal.pose.header.frame_id = "map";
>     goal.pose.pose.position.x = x.value();
>     goal.pose.pose.position.y = y.value();
>     goal.pose.pose.orientation.w = 1.0;
>     return true;
> }
> ```

- **`onResultReceived(WrappedResult& wr)`** – zkontrolujte kód výsledku

> [!TIP]
> ```cpp
> if (wr.code == rclcpp_action::ResultCode::SUCCEEDED)
>     return BT::NodeStatus::SUCCESS;
> return BT::NodeStatus::FAILURE;
> ```

- **`onFailure()`** a **`onFeedback()`** – vraťte `FAILURE` resp. `RUNNING`

Nezapomeňte na registraci pluginu na konci souboru:
```cpp
CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
```

### 4) Implementace GetTaskService

Soubor: `src/bt/GetTaskService.cpp`

Dědí z `BT::RosServiceNode<std_srvs::srv::Trigger>`. Volá službu `/get_pickup_task`.

Implementujte:
- **`providedPorts()`** – definujte output port `manipulator_id` (typ `std::string`)
- **`setRequest()`** – Trigger nemá žádné pole v requestu, stačí `return true;`
- **`onResponseReceived()`** – zkontrolujte `response->success` a zapište `response->message` do output portu

> [!TIP]
> ```cpp
> BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
>     if (!response->success) return BT::NodeStatus::FAILURE;
>     setOutput("manipulator_id", response->message);
>     return BT::NodeStatus::SUCCESS;
> }
> ```

Registrace: `CreateRosNodePlugin(GetTaskService, "GetTaskService");`

### 5) Implementace ConfirmLoadingService a GetDropoffService

Soubory: `src/bt/ConfirmLoadingService.cpp`, `src/bt/GetDropoffService.cpp`

Oba jsou analogické ke `GetTaskService`:

- **ConfirmLoadingService** – nemá žádný output port. Služba `/confirm_loading` čeká na dokončení nakládky.
- **GetDropoffService** – output port `storage_id` (typ `std::string`). Služba `/get_dropoff_location` vrátí ID skladu (např. "B2").

### 6) Implementace LookupPose

Soubor: `src/bt/LookupPose.cpp`

Synchronní node (`BT::SyncActionNode`), který mapuje textové ID na souřadnice.

Implementujte:
- Tabulku souřadnic (viz tabulky výše) – 3 manipulátory + 8 skladových pozic
- **`providedPorts()`** – input port `location_id`, output porty `x` a `y`
- **`tick()`** – vyhledejte ID v tabulce a zapište souřadnice do output portů

> [!TIP]
> ```cpp
> BT::NodeStatus tick() override {
>     auto id = getInput<std::string>("location_id");
>     if (!id) return BT::NodeStatus::FAILURE;
>     auto it = pose_table_.find(id.value());
>     if (it == pose_table_.end()) return BT::NodeStatus::FAILURE;
>     setOutput("x", it->second.x);
>     setOutput("y", it->second.y);
>     return BT::NodeStatus::SUCCESS;
> }
> ```

Registrace (jiný způsob než u ROS nodů):
```cpp
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<LookupPose>("LookupPose");
}
```

### 7) Implementace BT serveru

Soubor: `src/bt/bt_server.cpp`

Vytvořte třídu odvozenou z `BT::TreeExecutionServer`. Implementujte `main()` s `MultiThreadedExecutor`.

> [!TIP]
> ```cpp
> #include "behaviortree_ros2/tree_execution_server.hpp"
>
> class BTServer : public BT::TreeExecutionServer {
> public:
>     BTServer(const rclcpp::NodeOptions& options)
>         : TreeExecutionServer(options) {}
>
>     void onTreeCreated(BT::Tree& tree) override {
>         // volitelně: přidat logger
>     }
> };
>
> int main(int argc, char** argv) {
>     rclcpp::init(argc, argv);
>     rclcpp::NodeOptions options;
>     auto server = std::make_shared<BTServer>(options);
>     rclcpp::executors::MultiThreadedExecutor exec;
>     exec.add_node(server->node());
>     exec.spin();
>     rclcpp::shutdown();
>     return 0;
> }
> ```

### 8) Definice XML behavior tree

Soubor: `behavior_trees/warehouse_mission.xml`

Definujte sekvenci mise. Porty se propojují přes blackboard proměnné `{název}`. Začátek stromu vypadá takto:

> [!TIP]
> ```xml
> <root BTCPP_format="4">
>   <BehaviorTree ID="WarehouseMission">
>     <Sequence name="main_mission">
>       <GetTaskService service_name="/get_pickup_task"
>                       manipulator_id="{manipulator_id}"/>
>       <LookupPose location_id="{manipulator_id}"
>                   x="{goal_x}" y="{goal_y}"/>
>       <NavigateToPoseAction action_name="/go_to_goal"
>                             x="{goal_x}" y="{goal_y}"/>
>       <!-- Doplňte zbytek mise: potvrzení nakládky, zjištění skladu,
>            navigace do skladu a návrat na start -->
>     </Sequence>
>   </BehaviorTree>
> </root>
> ```
>
> Poznámka: `{goal_x}` a `{goal_y}` se přepisují při každém volání `LookupPose` – to je v pořádku, protože `Sequence` zajišťuje sekvenční vykonávání.

### 9) Kompilace a spuštění

```bash
cd ~/mpc-rbt_ws
colcon build
source install/setup.bash
```

Spusťte celý systém:
```bash
# Terminál 1: simulátor (pokud ještě neběží)
ros2 launch mpc_rbt_simulator simulator.launch.py

# Terminál 2: všechny nody včetně BT serveru a warehouse manageru
ros2 launch mpc_rbt_solution solution.launch.py
```

Počkejte, až se všechny nody spustí (ověřte v logu, že se načetla mapa a BT pluginy). Pak spusťte misi:

```bash
# Terminál 3: spuštění mise
ros2 action send_goal /bt_server/execute_tree \
  behaviortree_interfaces/action/ExecuteTree \
  "{target_tree: 'WarehouseMission'}"
```

Sledujte v RVizu, jak robot postupně naviguje k manipulátoru, čeká na naložení, jede do skladu a vrací se na start.

### 10) Ověření funkčnosti

Robot by měl autonomně provést celou misi definovanou v XML souboru:
1. Vyžádá si úkol od warehouse manageru.
2. Naviguje k přiřazenému manipulátoru.
3. Počká na naložení.
4. Naviguje do přiřazeného skladu.
5. Vrátí se na startovní pozici.

## Odladění

Pokud něco nefunguje:

- **"Can't find a tree with name"** – zkontrolujte, že klíč v YAML konfiguraci (`config/bt_server.yaml`) odpovídá parametru `name=` v launch souboru. Obě hodnoty musí být shodné.
- **Pluginy se nenačítají** – zkontrolujte, že každý plugin má na konci souboru registrační makro (`CreateRosNodePlugin` nebo `BT_REGISTER_NODES`).
- **Navigace selže** – ověřte, že souřadnice v `LookupPose` nejsou uvnitř překážky nebo v inflované zóně. Použijte RViz k ověření pozic.
- **Service timeout** – `confirm_loading` může trvat až 5 s. Pokud timeout nastává, zvyšte `ros_plugins_timeout` v `config/bt_server.yaml`.
