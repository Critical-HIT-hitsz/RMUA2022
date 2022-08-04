#ifndef MAP_EXECUTOR_H
#define MAP_EXECUTOR_H

struct robot_pose
{
    int x;
    int y;
    bool alive;
};

class Map_executor
{
    public:
    Map_executor()
    {
        self.alive = false;
        teammate.alive = false;
        enemy_1.alive = false;
        enemy_2.alive = false;
    }
    ~Map_executor(){}
    void ArmorDetectionUpdate();
    void SentryUpdate();
    void LocalizationUpdate();
    void TeammateUpdate();
    robot_pose self;
    robot_pose teammate;
    robot_pose enemy_1;
    robot_pose enemy_2;
};

#endif