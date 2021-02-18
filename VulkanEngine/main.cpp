/**
* The Vienna Vulkan Engine
*
* (c) bei Helmut Hlavacs, University of Vienna
*
*/


#include "VEInclude.h"
#include <string>
#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <math.h>
#include <cstdlib>

#include <stdio.h>
#include <array>
#include <set>
#include <vector>
#include <algorithm>
#include <iterator>
#include <list>
#include <unordered_map>
#include <unordered_set>

#include "ViennaPhysicsEngine-main/sat.h"
#include "ViennaPhysicsEngine-main/gjk_epa.h"
#include "ViennaPhysicsEngine-main/contact.h"

using namespace std;

bool winner = false;

struct Cell {
    int x, y;
    double weight;
    Cell() {
        
    }
    
    Cell(Cell *cell) {
        x = cell->x;
        y = cell->y;
        weight = cell->weight;
    }
    
    Cell(int x_, int y_, double weight_):
    x(x_), y(y_), weight(weight_) {}
    
    Cell(int x_, int y_):
    x(x_), y(y_) {}
};

namespace std {
    template <> struct hash<Cell> {
        typedef Cell argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const Cell& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
}

bool operator == (Cell a, Cell b) {
    return a.x == b.x && a.y == b.y;
}

bool operator != (Cell a, Cell b) {
    return a.x != b.x || a.y != b.y;
}

bool operator < (Cell a, Cell b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

enum State{IDLE, MOVE, ATTACK, ESCAPE};

struct NPC {
    Cell pos;
    State state;
    int health;
    int weapon;
    bool right;
    string name;
    
    NPC() {
        
    }
    
    NPC(Cell pos_, string name_): pos(pos_), name(name_) {
        state = IDLE;
        health = 100;
        weapon = 3;
        static auto gen = std::bind(std::uniform_int_distribution<>(0,1),std::default_random_engine());
        right = gen();
    }
    
    NPC(int x, int y, string name_): name(name_) {
        pos = new Cell(x, y);
        state = IDLE;
        health = 50;
        weapon = 3;
        static auto gen = std::bind(std::uniform_int_distribution<>(0,1),std::default_random_engine());
        right = gen();
    }
    
    void craft_weapons() {
        weapon += 3;
    }
    
    void heal() {
        health += 5;
    }
    
    void move(int x, int y) {
        pos = new Cell(x, y);
    }
    
    void move(Cell x) {
        pos = new Cell(x.x, x.y);
    }
    
    void attack() {
        weapon--;
    }
    
    void getAttacked() {
        health -= 10;
    }
    
    void changeState(State new_state) {
        state = new_state;
    }
    
    void patrol() {
        if (pos.y >= 9) {
            right = false;
        }
        if (pos.y <= 0) {
            right = true;
        }
        
        if (right) {
            pos = new Cell(pos.x, pos.y + 1);
        } else {
            pos = new Cell(pos.x, pos.y - 1);
        }
    }
};

struct Grid {
    int width, height;
    vector<vector<Cell>> cells;
    std::unordered_set<Cell> walls;
    vector<NPC> teamA;
    vector<NPC> teamB;
    
    Grid(int width_, int height_)
         : width(width_), height(height_) {
             for (int i = 0; i < width_; i++) {
                 vector<Cell> temp;
                 for (int j = 0; j < height_; j++) {
                     temp.push_back(Cell(i, j, 1.0));
                 }
                 cells.push_back(temp);
             }
         }

    void add_team_A(vector<NPC> team) {
        teamA = team;
    }
    
    void  add_team_B(vector<NPC> team) {
        teamB = team;
    }
    
    void add_wall(int x, int y) {
        walls.insert(new Cell(x, y));
    }
    
    bool if_in_bounds(Cell id) {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }
    
    bool passable(Cell id) {
        return walls.find(id) == walls.end();
    }
    
    void add_weight(int x, int y, double weight) {
        if (if_in_bounds(Cell(x,y))) {
            cells[x][y] = Cell(x, y, weight);
        }
    }
    
    void print_map() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                cout << cells[i][j].weight;
                if (cells[i][j].weight == 1) {
                    cout << ".0";
                }
                if (j != height - 1) {
                    cout << " - ";
                }
            }
            cout << endl;
        }
        cout << endl;
    }
    
    vector<Cell> get_neighbours(Cell current) {
        vector<Cell> results;
        double x = current.x;
        double y = current.y;
        
        if (if_in_bounds(Cell(x, y+1)) && passable(Cell(x, y+1))) {
            Cell down(cells[x][y+1]);
            results.push_back(down);
        }
        
        if (if_in_bounds(Cell(x, y-1)) && passable(Cell(x, y-1))) {
            Cell up(cells[x][y-1]);
            results.push_back(up);
        }
        
        if (if_in_bounds(Cell(x-1, y)) && passable(Cell(x-1, y))) {
            Cell left(cells[x-1][y]);
            results.push_back(left);
        }
        
        if (if_in_bounds(Cell(x+1, y)) && passable(Cell(x+1, y))) {
            Cell right(cells[x+1][y]);
            results.push_back(right);
        }
        
        if (if_in_bounds(Cell(x-1, y-1)) && passable(Cell(x-1, y-1))) {
            Cell left_up(cells[x-1][y-1]);
            results.push_back(left_up);
        }
        
        if (if_in_bounds(Cell(x+1, y-1)) && passable(Cell(x+1, y-1))) {
            Cell right_up(cells[x+1][y-1]);
            results.push_back(right_up);
        }
        
        if (if_in_bounds(Cell(x-1, y+1)) && passable(Cell(x-1, y+1))) {
            Cell left_down(cells[x-1][y+1]);
            results.push_back(left_down);
        }
        
        if (if_in_bounds(Cell(x+1, y+1)) && passable(Cell(x+1, y+1))) {
            Cell right_down(cells[x+1][y+1]);
            results.push_back(right_down);
        }
        
        return results;
    }
};

Grid create_map() {
    Grid grid(400, 400);
    return grid;
}

inline double heuristic(Cell current, Cell find) {
  return std::abs(current.x - find.x) + std::abs(current.y - find.y);
}

template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

void a_star (Grid grid,
             Cell start,
             Cell end,
             unordered_map<Cell, Cell>& parent,
             unordered_map<Cell, double>& cost) {
    PriorityQueue<Cell, double> open;
    open.put(start, 0);

    parent[start] = start;
    cost[start] = 0;
  
    while (!open.empty()) {
        Cell current = open.get();

        if (current == end) {
            break;
        }

        for (Cell next : grid.get_neighbours(current)) {
            double new_cost = cost[current] + next.weight * 10;
            if (cost.find(next) == cost.end() || new_cost < cost[next]) {
                cost[next] = new_cost;
                double priority = new_cost + heuristic(next, end);
                open.put(next, priority);
                parent[next] = current;
            }
        }
    }
}

void print_path(
    Cell start, Cell end, unordered_map<Cell, Cell> parent,
               unordered_map<int, string> heights) {
    vector<string> path;
    Cell current = end;
    
    while (current != start) {
        path.push_back(heights[current.y] + to_string(current.x+1));
        current = parent[current];
    }
    
    reverse(path.begin(), path.end());
    
    cout << heights[start.y] + to_string(start.x+1) << endl;
    for (string line: path) {
        cout << line << endl;
    }
}

void loadWallsLogic(Grid &g) {
    //1
    for (int i = 50; i <= 150; i++) {
        for (int j = 70; j <= 80; j++) {
            g.add_wall(j, i);
        }
    }
    //2
    for (int i = 70; i <= 170; i++) {
        for (int j = 40; j <= 50; j++) {
            g.add_wall(i, j);
        }
    }
    //3
    for (int i = 230; i <= 240; i++) {
        for (int j = 150; j <= 250; j++) {
            g.add_wall(i, j);
        }
    }
    //4
    for (int i = 150; i <= 230; i++) {
        for (int j = 240; j <= 250; j++) {
            g.add_wall(i, j);
        }
    }
    //5
    for (int i = 120; i <= 130; i++) {
        for (int j = 300; j <= 400; j++) {
            g.add_wall(i, j);
        }
    }
    //6
    for (int i = 20; i <= 120; i++) {
        for (int j = 390; j <= 400; j++) {
            g.add_wall(i, j);
        }
    }
    //7
    for (int i = 250; i <= 260; i++) {
        for (int j = 300; j <= 400; j++) {
            g.add_wall(i, j);
        }
    }
    //8
    for (int i = 260; i <= 360; i++) {
        for (int j = 300; j <= 310; j++) {
            g.add_wall(i, j);
        }
    }
    //9
    for (int i = 270; i <= 280; i++) {
        for (int j = 100; j <= 200; j++) {
            g.add_wall(i, j);
        }
    }
}

Box new_box{ {2.0f, 6.0f, 2.0f}, scale( mat4(1.0f), vec3(1.0f, 10.0f, 1.0f))};
Box player = new_box;

Box new_ground{ {200.0f, 0.0f, 200.0f}, scale( mat4(1.0f), vec3(400.0f, 1.0f, 400.0f))};
Box ground = new_ground;

Box enemy1{{100.0f, 0.0f, 100.0f}, scale( mat4(1.0f), vec3(5.0f, 40.0f, 5.0f))};
Box enemy2{{200.0f, 0.0f, 200.0f}, scale( mat4(1.0f), vec3(5.0f, 40.0f, 5.0f))};
Box enemy3{{100.0f, 0.0f, 350.0f}, scale( mat4(1.0f), vec3(5.0f, 40.0f, 5.0f))};
Box enemy4{{300.0f, 0.0f, 350.0f}, scale( mat4(1.0f), vec3(5.0f, 40.0f, 5.0f))};
Box enemy5{{300.0f, 0.0f, 150.0f}, scale( mat4(1.0f), vec3(5.0f, 40.0f, 5.0f))};
Box enemies[5]= {enemy1, enemy2, enemy3, enemy4, enemy5};

vector<Box> wallsValues;

vector<Box> floors;
vector<int> killedEnemies;

Grid grid = create_map();

namespace ve {
	///simple event listener for rotating objects
	class BlinkListener : public VEEventListener {
		VEEntity *m_pEntity;
		double t_now = 0.0;
		double t_last = 0.0;
		double m_blinkDuration;

	public:
		///Constructor
		BlinkListener(std::string name, VEEntity *pEntity, double duration) :
			VEEventListener(name), m_pEntity(pEntity), m_blinkDuration(duration) {};

		///\brief let cubes blink
		void onFrameStarted(veEvent event) {
			t_now += event.dt;
			double duration = t_now - t_last;

			if (duration > m_blinkDuration) {
				m_pEntity->m_visible = m_pEntity->m_visible ? false : true;	//toggle visibility
				t_last = t_now;
			}
		}

		///\returns true if this event listener instance should be deleted also
		bool onSceneNodeDeleted(veEvent event) {
			if (m_pEntity == event.ptr) return true;
			return false;
		};

	};


	///simple event listener for loading levels
	class LevelListener : public VEEventListener {
	public:
		///Constructor
		LevelListener(std::string name) : VEEventListener(name) {};

		///load a new level when pressing numbers 1-3
		virtual bool onKeyboard(veEvent event) {
			if (event.idata3 == GLFW_RELEASE) return false;

			if (event.idata1 == GLFW_KEY_1 && event.idata3 == GLFW_PRESS) {
                // TEMP SOLUTION ON SCENE LOADING (other wise, scenes not reload, but load on top of each other).
                getEnginePointer()->initEngine();
				getEnginePointer()->loadLevel(1);
				return true;
			}

			if (event.idata1 == GLFW_KEY_2 && event.idata3 == GLFW_PRESS) {
                // TEMP SOLUTION ON SCENE LOADING (other wise, scenes not reload, but load on top of each other).
                getEnginePointer()->initEngine();
				getEnginePointer()->loadLevel(2);
				return true;
			}

//			if (event.idata1 == GLFW_KEY_3 && event.idata3 == GLFW_PRESS) {
//				getSceneManagerPointer()->deleteScene();
//				getEnginePointer()->loadLevel(3);
//				return true;
//			}
			return false;
		}
	};


	///simple event listener for switching on/off light
	class LightListener : public VEEventListener {
	public:
		///Constructor
		LightListener(std::string name) : VEEventListener(name) {};

		///\brief switch on or off a given light
		void toggleLight(std::string name) {
			VELight *pLight = (VELight*)getSceneManagerPointer()->getSceneNode(name);
			if (pLight == nullptr) return;

			pLight->m_switchedOn = pLight->m_switchedOn ? false : true;
		}

		///load a new level when pressing numbers 1-3
		virtual bool onKeyboard(veEvent event) {
			if (event.idata3 == GLFW_RELEASE) return false;

			if (event.idata1 == GLFW_KEY_8 && event.idata3 == GLFW_PRESS) {
				toggleLight("StandardDirLight");
				return true;
			}

			if (event.idata1 == GLFW_KEY_9 && event.idata3 == GLFW_PRESS) {
				toggleLight("StandardPointLight");
				return true;
			}

			if (event.idata1 == GLFW_KEY_0 && event.idata3 == GLFW_PRESS) {
				toggleLight("StandardSpotLight");
				return true;
			}

			return false;
		}
	};

    class BulletListener : public VEEventListener {
        VESceneNode *m_pObject = nullptr;
        vec3 direction;
        Box bullet;
        int counter;
    public:
        ///Constructor
        BulletListener(std::string name, VESceneNode *pObject, vec3 axis_, int c_) :
            VEEventListener(name),  m_pObject(pObject), direction(axis_), counter(c_) {
                Box new_bullet{ {0.0f, 0.0f, 0.0f}, scale( mat4(1.0f), vec3(.3f, .1f, .3f))};
                bullet = new_bullet;
                bullet.m_pos = pObject->getPosition();
        };

        void onFrameStarted(veEvent event) {
            glm::vec3 acceleration = direction * 100;
            m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), acceleration * event.dt));
            bullet.m_pos = m_pObject->getPosition();
            
            int size = *(&enemies + 1) - enemies;
            for(int i = 0; i < size; i++) {
                vec3 p;
                vec3 mtv = glm::vec3(0, 0.0f, 0);
                auto hit1 = gjk( bullet, enemies[i], mtv, p, true);
                if (hit1) {
                    if (getSceneManagerPointer()->getSceneNode("enemy" + to_string(i)) != nullptr) {
                        getEnginePointer()->deleteEventListener("enemy" + to_string(i));
//                        getSceneManagerPointer()->deleteSceneNodeAndChildren("enemy" + to_string(i));
                        getSceneManagerPointer()->getSceneNode("enemy" + to_string(i))->setTransform(translate(mat4(1), vec3(-50, 0, 0)));
                        enemies[i].m_pos = vec3(-50, 0, 0);
                        killedEnemies.push_back(i);
                    }
                }
            }
        }
    };

    class EnemyBulletListener : public VEEventListener {
        VESceneNode *m_pObject = nullptr;
        vec3 direction;
        Box bullet;
        int i;
    public:
        ///Constructor
        EnemyBulletListener(std::string name, VESceneNode *pObject, int i_) :
            VEEventListener(name),  m_pObject(pObject), i(i_) {
                Box new_bullet{ {0.0f, 0.0f, 0.0f}, scale( mat4(1.0f), vec3(.3f, 6.f, .3f))};
                bullet = new_bullet;
                bullet.m_pos = pObject->getPosition();
                direction =  (player.m_pos - vec3(0, 8, 0)) - enemies[i].m_pos;
        };

        void onFrameStarted(veEvent event) {
            glm::vec3 acceleration = direction * 1;
            m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), acceleration * event.dt));
            bullet.m_pos = m_pObject->getPosition();
            
            if (bullet.m_pos.x >= 401 || bullet.m_pos.x <= -1 ||
                bullet.m_pos.y >= 401 || bullet.m_pos.y <= -1 ||
                bullet.m_pos.z >= 401 || bullet.m_pos.z <= -1) {
                if (getSceneManagerPointer()->getSceneNode("The enemy bullet" + to_string(i)) != nullptr) {
                    getSceneManagerPointer()->deleteSceneNodeAndChildren("The enemy bullet" + to_string(i));
                    getEnginePointer()->deleteEventListener("The enemy bullet" + to_string(i));
                }
            }
            
            vec3 p;
            vec3 mtv = glm::vec3(0, 0.0f, 0);
            if (abs(distance(bullet.m_pos, player.m_pos)) <= 3) {
                auto hit1 = gjk( bullet, player, mtv, p, true);
                if (hit1) {
                    getEnginePointer()->end();
                    cout << "You Lost" << endl;
                }
            }
        }
    };

    class EnemyListener : public VEEventListener {
        VESceneNode *m_pObject = nullptr;
        int index;
        Cell playerPosSave;
        Cell oldStart;
        unordered_map<Cell, Cell> parentSaved;
        int counter = 0;
    public:
        ///Constructor
        EnemyListener(std::string name, VESceneNode *pObject, int index_) :
            VEEventListener(name),  m_pObject(pObject), index(index_) {
                playerPosSave = new Cell(floor(player.m_pos.x), floor(player.m_pos.z));
        };

        void onFrameStarted(veEvent event) {
            decide(event);
        }
        
        void decide(veEvent event) {
            Cell start = new Cell(floor(m_pObject->getPosition().x), floor(m_pObject->getPosition().z));
            Cell end = new Cell(floor(player.m_pos.x), floor(player.m_pos.z));
            vec3 p;
            vec3 mtv = glm::vec3(0, 0.0f, 0);
            auto killedPlayer = gjk( player, enemies[index], mtv, p, true);
            if (killedPlayer) {
                getEnginePointer()->end();
                cout << "You Lost" << endl;
            }
            if (abs(end.x - start.x) <= 60 && abs(end.y - start.y) <= 60 && player.m_pos.y <= 8) {
                chasePlayer(event);
            }
            if (abs(end.x - start.x) <= 100 && abs(end.y - start.y) <= 100 && player.m_pos.y > 8) {
                lookAndShoot(event);
            }
        }
        
        void lookAndShoot(veEvent event) {
            if (getSceneManagerPointer()->getSceneNode("The enemy bullet" + to_string(index)) == nullptr) {
                VESceneNode *e0;
                e0 = getSceneManagerPointer()->loadModel("The enemy bullet" + to_string(index), "media/models/test/crate0", "cube.obj", 0, getSceneManagerPointer()->getSceneNode("Level 1"));
                
                e0->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(.3f, 6.f, .3f)));
                e0->lookAt(vec3(0,0,0), player.m_pos, vec3(0,0,1));
                e0->multiplyTransform( translate(mat4(1), vec3(m_pObject->getPosition().x, 10, m_pObject->getPosition().z)));
                
                getEnginePointer()->registerEventListener(new EnemyBulletListener("The enemy bullet" + to_string(index), e0, index), { veEvent::VE_EVENT_FRAME_STARTED});
            }
        }
        
        void chasePlayer(veEvent event) {
            Cell start = new Cell(floor(m_pObject->getPosition().x), floor(m_pObject->getPosition().z));
            Cell end = new Cell(floor(player.m_pos.x), floor(player.m_pos.z));
        
            if (parentSaved.empty() || parentSaved[playerPosSave] == start) {
                unordered_map<Cell, Cell> parent;
                unordered_map<Cell, double> cost;
                if ((int)m_pObject->getPosition().x >= -5 && (int)m_pObject->getPosition().x <= 405 &&
                    (int)m_pObject->getPosition().z >= -5 && (int)m_pObject->getPosition().z <= 405) {
                    if (end.x >= 0 && end.y >= 0 && end.x <= 400 && end.y <= 400) {
                        a_star(grid, start, end, parent, cost);
                        Cell current = end;
//                        while (parent[current] != start) {
//                            current = parent[current];
//                        }
//                        m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), vec3(current.x - start.x, 0, current.y - start.y) * event.dt * 15));
//                        enemies[index].m_pos = m_pObject->getPosition();
                        playerPosSave = end;
                        parentSaved = parent;
                        oldStart = start;
                    }
                }
            } else {
                Cell current = playerPosSave;
                while (parentSaved[current] != oldStart) {
                    current = parentSaved[current];
                }
                oldStart = current;
                m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), vec3(current.x - start.x, 0, current.y - start.y) * event.dt * 15));
                enemies[index].m_pos = m_pObject->getPosition();
            }
        }
    };

    class CharacterMovementListener : public VEEventListener {
        VESceneNode *m_pObject = nullptr;
        VESceneNode *camera = nullptr;
        VEEngine *engine = nullptr;
        double F, I;
        int m;
        double force;
        glm::vec3 net_F;
        glm::vec3 gravity;
        glm::vec3 idleF;
        bool pressed = false;
        int counter = 0;
    public:
        ///Constructor
        CharacterMovementListener(std::string name, VESceneNode *pObject, VESceneNode *camera_, VEEngine *eng) :
            VEEventListener(name),  m_pObject(pObject), camera(camera_), engine(eng)  {
                net_F = glm::vec3(0.f, 400.f, 0.f);
                idleF = glm::vec3(0.f, 0.f, 0.f);
                gravity = glm::vec3(0.f, -9.8f, 0.f);
                m = 1;
        };

        bool onMouseButton(veEvent event) {
            if (event.idata3 == GLFW_RELEASE) {
                return false;
            }
            
            if (event.idata1 == GLFW_MOUSE_BUTTON_LEFT && event.idata3 == GLFW_PRESS) {
                if (counter >= 5) {
                    counter = 0;
                }
                if (getSceneManagerPointer()->getSceneNode("The bullet" + to_string(counter)) != nullptr) {
                    engine->deleteEventListener("bullet" + to_string(counter));
                    getSceneManagerPointer()->deleteSceneNodeAndChildren("The bullet" + to_string(counter));
                }
                VESceneNode *e0;
                if (getSceneManagerPointer()->getSceneNode("The bullet" + to_string(counter)) == nullptr && getSceneManagerPointer()->getSceneNode("Level 1") != nullptr) {
                    VECHECKPOINTER( e0 = getSceneManagerPointer()->loadModel("The bullet" + to_string(counter), "media/models/test/crate0", "cube.obj", 0, getSceneManagerPointer()->getSceneNode("Level 1")));
                }
                
                e0->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(.1f, .3f, .1f)));
                float angle = 90*M_PI/180;
                e0->multiplyTransform( glm::rotate(glm::mat4(1.0f), angle, glm::vec3(1.f, 0, 0)));
                e0->multiplyTransform( camera->getTransform());
                e0->multiplyTransform( m_pObject->getTransform());
                
                engine->registerEventListener(new BulletListener("bullet" + to_string(counter), e0, camera->getZAxis(), counter), { veEvent::VE_EVENT_FRAME_STARTED});
                counter++;
                
            }
            
            return false;
        }
        
        bool onKeyboard(veEvent event) {
            if (event.idata3 == GLFW_RELEASE) {
                return false;
            }

            if (event.idata1 == GLFW_KEY_SPACE && event.idata3 == GLFW_PRESS) {
                if (!pressed) {
                    net_F = glm::vec3(0.f, 100.f, 0.f);
                    pressed = true;
                }
            }
            
            glm::vec4 translate = glm::vec4(0.0, 0.0, 0.0, 1.0);
            VESceneNode *pCamera = camera;
            glm::mat4 a = glm::mat4(
                                    pCamera->getTransform()[0][0], 0, -pCamera->getTransform()[2][0], pCamera->getTransform()[3][0],
                                    0, 1, 0, pCamera->getTransform()[3][1],
                                    -pCamera->getTransform()[0][2], 0, pCamera->getTransform()[2][2], pCamera->getTransform()[3][2],
                                    pCamera->getTransform()[0][3], pCamera->getTransform()[1][3], pCamera->getTransform()[2][3], pCamera->getTransform()[3][3]
                                    );
            switch (event.idata1) {
                case GLFW_KEY_A:
                    translate = a * glm::vec4(-1.0, 0.0, 0.0, 1.0);    //left
                    break;
                case GLFW_KEY_D:
                    translate =  a * glm::vec4(1.0, 0.0, 0.0, 1.0); //right
                    break;
                case GLFW_KEY_W:
                    translate =  a * glm::vec4(0.0, 0.0, 1.0, 1.0); //forward
                    break;
                case GLFW_KEY_S:
                    translate =  a * glm::vec4(0.0, 0.0, -1.0, 1.0); //back
                    break;
            }
            
            glm::vec3 trans = 30.0f * glm::vec3(translate.x, translate.y, translate.z);
            Box testHit = player;
            testHit.m_pos = glm::translate(glm::mat4(1.0f), (float)event.dt * trans) * vec4(player.m_pos, 1);
            
            bool canWalk = true;
            for(Box wall : wallsValues) {
                vec3 p;
                vec3 mtv = glm::vec3(0, 0.0f, 0);
                auto hitWall = gjk( testHit, wall, mtv, p, true);
                if (hitWall) {
                    canWalk = false;
                }
            }
            if (canWalk) {
                player.m_pos = glm::translate(glm::mat4(1.0f), (float)event.dt * trans) * vec4(player.m_pos, 1);
                m_pObject->multiplyTransform( glm::translate(glm::mat4(1.0f), (float)event.dt * trans) );
            }
            return false;
        }
        
        void onFrameStarted(veEvent event) {
            if (pressed) {
                net_F += (gravity * (float) m);
                glm::vec3 acceleration = (net_F / (float) m) * (float) event.dt;
                auto temp = player.m_pos;
                player.m_pos = glm::translate(glm::mat4(1.0f), acceleration) * vec4(player.m_pos, 1);
                for(Box f : floors) {
                    vec3 p;
                    vec3 mtv = glm::vec3(0, 0.0f, 0);
                    auto hit1 = gjk( player, f, mtv, p, true);
                    if (hit1) {
                        acceleration = vec3(0, 0, 0);
                        pressed = false;
                    }
                }
                player.m_pos = temp;
                m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), acceleration));
                player.m_pos = glm::translate(glm::mat4(1.0f), acceleration) * vec4(player.m_pos, 1);
            } else {
                if (player.m_pos.y > 8) {
                    idleF += (gravity * (float) m);
                    glm::vec3 acceleration = (idleF / (float) m) * (float) event.dt;
                    auto temp = player.m_pos;
                    player.m_pos = glm::translate(glm::mat4(1.0f), acceleration) * vec4(player.m_pos, 1);
                    for(Box f : floors) {
                        vec3 p;
                        vec3 mtv = glm::vec3(0, 0.0f, 0);
                        auto hit1 = gjk( player, f, mtv, p, true);
                        if (hit1) {
                            acceleration = vec3(0, 0, 0);
                            idleF = vec3(0, 0, 0);
                        }
                    }
                    player.m_pos = temp;
                    m_pObject->multiplyTransform(glm::translate(glm::mat4(1.0f), acceleration));
                    player.m_pos = glm::translate(glm::mat4(1.0f), acceleration) * vec4(player.m_pos, 1);
                }
            }
            if (killedEnemies.size() >= 5) {
                getEnginePointer()->end();
                cout << "You Won!" << endl;
            }
        }
    };


	///user defined manager class, derived from VEEngine
	class MyVulkanEngine : public VEEngine {
	protected:
        bool idle = true;
	public:
		/**
		* \brief Constructor of my engine
		* \param[in] debug Switch debuggin on or off
		*/
		MyVulkanEngine( bool debug=false) : VEEngine(debug) {};
		~MyVulkanEngine() {};

		///Register an event listener to interact with the user
		virtual void registerEventListeners() {
			VEEngine::registerEventListeners();

			registerEventListener(new LevelListener("LevelListener"), { veEvent::VE_EVENT_KEYBOARD });
			registerEventListener(new LightListener("LightListener"), { veEvent::VE_EVENT_KEYBOARD });
			//registerEventListener(new VEEventListenerNuklearDebug("NuklearDebugListener"), { veEvent::VE_EVENT_DRAW_OVERLAY});
		};

		///create many lights
		void createLights(uint32_t n, VESceneNode *parent) {
			float stride = 200.0f;
			static std::default_random_engine e{ 12345 };
			static std::uniform_real_distribution<> d{ 1.0f, stride };

			for (uint32_t i = 0; i < n; i++) {
				VELight *pLight;
				VECHECKPOINTER(pLight = getSceneManagerPointer()->createLight("Light" + std::to_string(i), VELight::VE_LIGHT_TYPE_SPOT, parent));
				pLight->m_col_diffuse = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f);
				pLight->m_col_specular = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);
				pLight->m_param[0] = stride/3.0f;
				//pLight->multiplyTransform(glm::translate(glm::vec3(d(e) - stride / 2.0f, d(e)/10.0f, d(e) - stride / 2.0f)));

				pLight->lookAt(glm::vec3(d(e) - stride / 2.0f, d(e) / 30.0f, d(e) - stride / 2.0f),
					glm::vec3(d(e) - stride / 2.0f, 0.0f, d(e) - stride / 2.0f),
					glm::vec3( 0.0f, 1.0f, 0.0f ));
			}
		}
        
        void loadEnemies(VESceneNode *pScene) {
            int size = *(&enemies + 1) - enemies;
            for(int i = 0; i < size; i++) {
                VESceneNode *e2;
                VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("enemy" + to_string(i), "media/models/test/santa", "Santa.obj", 0, pScene));
                e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(0.08f, 0.08f, 0.08f)));
                float angle = -90*M_PI/180;
                e2->multiplyTransform( glm::rotate(glm::mat4(1.0f), angle, glm::vec3(1.0f, 0.0f, 0.0f)));
                e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(enemies[i].m_pos.x, enemies[i].m_pos.y, enemies[i].m_pos.z)));
                
                registerEventListener(new EnemyListener("enemy" + to_string(i), e2, i), { veEvent::VE_EVENT_FRAME_STARTED});
            }
        }
        
        void loadWalls(VESceneNode *pScene) {
            //1
            VESceneNode *e2;
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("The Cube0", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 20.0f, 100.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(75, 10.0f, 100)));
            
            Box wall1{ {75.0f, 10.0f, 100.0f}, scale( mat4(1.0f), vec3(10.0f, 20.0f, 100.0f))};
            wallsValues.push_back(wall1);
            //2
            VESceneNode *e3;
            VECHECKPOINTER( e3 = getSceneManagerPointer()->loadModel("The Cube1", "media/models/test/crate0", "cube.obj", 0, pScene));
            e3->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(100.0f, 20.0f, 10.0f)));
            e3->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(120, 10.0f, 45)));
            
            Box wall2{ {120.0f, 10.0f, 45.0f}, scale( mat4(1.0f), vec3(100.0f, 20.0f, 10.0f))};
            wallsValues.push_back(wall2);
            
            //3
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("The Cube3", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 20.0f, 100.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(235, 10.0f, 200)));
            
            Box wall3{ {235, 10.0f, 200}, scale( mat4(1.0f), vec3(10.0f, 20.0f, 100.0f))};
            wallsValues.push_back(wall3);
            //4
            VECHECKPOINTER( e3 = getSceneManagerPointer()->loadModel("The Cube4", "media/models/test/crate0", "cube.obj", 0, pScene));
            e3->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(80.0f, 20.0f, 10.0f)));
            e3->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(190, 10.0f, 245)));
            
            Box wall4{ {190.0f, 10.0f, 245.0f}, scale( mat4(1.0f), vec3(80.0f, 20.0f, 10.0f))};
            wallsValues.push_back(wall4);
            
            //5
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("The Cube5", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 20.0f, 100.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(125, 10.0f, 350)));
            
            Box wall5{ {125, 10.0f, 350}, scale( mat4(1.0f), vec3(10.0f, 20.0f, 100.0f))};
            wallsValues.push_back(wall5);
            //6
            VECHECKPOINTER( e3 = getSceneManagerPointer()->loadModel("The Cube6", "media/models/test/crate0", "cube.obj", 0, pScene));
            e3->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(100.0f, 20.0f, 10.0f)));
            e3->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(70, 10.0f, 395)));
            
            Box wall6{ {70.0f, 10.0f, 395.0f}, scale( mat4(1.0f), vec3(100.0f, 20.0f, 10.0f))};
            wallsValues.push_back(wall6);
            
            //7
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("The Cube7", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 20.0f, 100.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(255, 10.0f, 350)));
            
            Box wall7{ {255, 10.0f, 350}, scale( mat4(1.0f), vec3(10.0f, 20.0f, 100.0f))};
            wallsValues.push_back(wall7);
            //8
            VECHECKPOINTER( e3 = getSceneManagerPointer()->loadModel("The Cube8", "media/models/test/crate0", "cube.obj", 0, pScene));
            e3->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(100.0f, 20.0f, 10.0f)));
            e3->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(310, 10.0f, 305)));
            
            Box wall8{ {310, 10.0f, 305.0f}, scale( mat4(1.0f), vec3(100.0f, 20.0f, 10.0f))};
            wallsValues.push_back(wall8);
            
            //9
            VECHECKPOINTER( e3 = getSceneManagerPointer()->loadModel("The Cube9", "media/models/test/crate0", "cube.obj", 0, pScene));
            e3->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 20.0f, 100.0f)));
            e3->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(275, 10.0f, 150)));
            
            Box wall9{ {275, 10.0f, 150}, scale( mat4(1.0f), vec3(10.0f, 20.0f, 100.0f))};
            wallsValues.push_back(wall9);
        }
        
        void buildOuterWalls(VESceneNode *pScene) {
            //1
            VESceneNode *e2;
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("outerWall1", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(3.0f, 40.0f, 400.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(-1, 20.0f, 200)));
            
            Box wall1{ {-1, 20.0f, 200}, scale( mat4(1.0f), vec3(3.0f, 40.0f, 400.0f))};
            wallsValues.push_back(wall1);
            
            //2
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("outerWall2", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(3.0f, 40.0f, 400.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(401, 20.0f, 200)));
            
            Box wall2{ {401, 20.0f, 200}, scale( mat4(1.0f), vec3(3.0f, 40.0f, 400.0f))};
            wallsValues.push_back(wall2);
            
            //3
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("outerWall3", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(400.0f, 40.0f, 3.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(200, 20.0f, -1)));
            
            Box wall3{ {200, 20.0f, -1}, scale( mat4(1.0f), vec3(400.0f, 40.0f, 3.0f))};
            wallsValues.push_back(wall3);
            
            //4
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("outerWall4", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(400.0f, 40.0f, 3.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(200, 20.0f, 401)));
            
            Box wall4{ {200, 20.0f, 401}, scale( mat4(1.0f), vec3(400.0f, 40.0f, 3.0f))};
            wallsValues.push_back(wall4);
        }
        
        void buildFloors(VESceneNode *pScene) {
            //1
            VESceneNode *e2;
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("floor1", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(10.0f, 2.0f, 10.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(170, 10.0f, 100)));
            
            Box floor1{ {170, 10.0f, 100}, scale( mat4(1.0f), vec3(10.0f, 2.0f, 10.0f))};
            floors.push_back(floor1);
            wallsValues.push_back(floor1);
            
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("floor2", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(20.0f, 2.0f, 20.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(100, 10.0f, 300)));
            
            Box floor2{ {100, 10.0f, 300}, scale( mat4(1.0f), vec3(20.0f, 2.0f, 20.0f))};
            floors.push_back(floor2);
            wallsValues.push_back(floor2);
            
            VECHECKPOINTER( e2 = getSceneManagerPointer()->loadModel("floor3", "media/models/test/crate0", "cube.obj", 0, pScene));
            e2->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(20.0f, 2.0f, 20.0f)));
            e2->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(320, 10.0f, 250)));
            
            Box floor3{ {320, 10.0f, 250}, scale( mat4(1.0f), vec3(20.0f, 2.0f, 20.0f))};
            floors.push_back(floor3);
            wallsValues.push_back(floor3);
            floors.push_back(ground);
        }

        void loadLevelOne(VESceneNode *pScene) {
            VECHECKPOINTER( pScene = getSceneManagerPointer()->createSceneNode("Level 1", getRoot()) );

            VESceneNode *e1;
            VECHECKPOINTER( e1 = getSceneManagerPointer()->loadModel("The plane" + 0, "media/models/test/crate0", "cube.obj", 0, pScene));
            e1->multiplyTransform( glm::scale(glm::mat4(1.0f), glm::vec3(400.0f, 1.0f, 400.0f)));
            e1->multiplyTransform( glm::translate(glm::mat4(1.0f), glm::vec3(200.0f, -0.0f, 200.0f)));

            int counter = 0;
            
            registerEventListener(new CharacterMovementListener("Jumper", getSceneManagerPointer()->getCamera()->getParent(), getSceneManagerPointer()->getCamera(), this), { veEvent::VE_EVENT_MOUSEBUTTON, veEvent::VE_EVENT_KEYBOARD, veEvent::VE_EVENT_FRAME_STARTED});
            
            loadWallsLogic(grid);
            loadEnemies(pScene);
            loadWalls(pScene);
            buildOuterWalls(pScene);
            buildFloors(pScene);
        }


		///Load the first level into the game engine
		///The engine uses Y-UP, Left-handed
		virtual void loadLevel( uint32_t numLevel) {
            VESceneNode *pScene;
            getSceneManagerPointer()->deleteScene();
            VEEngine::loadLevel(numLevel);

            loadLevelOne(pScene);

			//createLights(10, pScene );
			//VESceneNode *pSponza = m_pSceneManager->loadModel("Sponza", "models/sponza", "sponza.dae", aiProcess_FlipWindingOrder);
			//pSponza->setTransform(glm::scale(glm::mat4(1.0f), glm::vec3(0.1f, 0.1f, 0.1f)));

		};
	};
}

using namespace ve;
using namespace std;

int main() {

	bool debug = false;
#ifdef  _DEBUG
	debug = true;
#endif

	MyVulkanEngine mve(false);	//enable or disable debugging (=callback, validation layers)

	mve.initEngine();
	mve.loadLevel(1);
	mve.run();
}

