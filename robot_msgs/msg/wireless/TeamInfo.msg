Header     header
uint8      id

uint8 attack_enemy       #正攻击敌方机器人id
uint8 being_attacked     #受攻击方向
uint8 to_buff            #要去的buff点
uint16 pose_x_teammate
uint16 pose_y_teammate
uint16 pose_x_enemy
uint16 pose_y_enemy

bool teammate_retreat    #队友是否撤退
uint8 team_mode          #主攻还是辅助的标志
uint8 decision_enemy_id  #决策选择的进攻目标
