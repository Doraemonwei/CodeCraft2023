/*
 * @Author: zzzzztw
 * @Date: 2023-03-10 20:54:04
 * @LastEditors: Do not edit
 * @LastEditTime: 2023-03-25 16:02:16
 * @FilePath: /huawei/chusai/LinuxRelease/SDK/maincopy.cpp
 */
#include <bits/stdc++.h>
using namespace std;



int framid, money;
using pdd = pair<double,double>;
struct Node
{
    int inWorkstateId_;//处于哪个工作台
    int thingNum_; // 身上物品类型；
    double time_coefficient;
    double collision_coefficient;
    double angleVelocity;
    double linearVelocity_x;
    double linearVelocity_y;
    double toward_;
    pdd pos_;//位置
    Node(int a,int b,double c,double d, double e, double f,double g,double h,double x, double y){
        inWorkstateId_ = a;
        thingNum_ = b;
        time_coefficient = c;
        collision_coefficient = d;
        angleVelocity = e;
        linearVelocity_x = f;
        linearVelocity_y = g;
        toward_ = h;
        pos_ = {x,y};
    };
    void print(){//测试用
        cout<<inWorkstateId_<<" "<<thingNum_<<" "<<time_coefficient<<" "
        <<collision_coefficient<<" "<<angleVelocity<<" "<<linearVelocity_x<<" "<<linearVelocity_y<<" "<<toward_<<" "<<pos_.first<<" "<<pos_.second<<endl;
    }

};

vector<Node>Robot;//四个机器人

struct Workplace
{
    int placeId_;//工作台id
    int charactor_;//工作台种类
    pdd pos_; // 位置（double x, double y）
    int lefttime_;// 剩余生产时间
    int cumsumestate_; //消耗格子状态 
    int productstate_;//产品状态
    Workplace(int a,int b, double x,double y, int time,int c,int d){
        placeId_ = a;
        charactor_ = b;
        pos_ = {x,y};
        lefttime_ = time;
        cumsumestate_ = c;
        productstate_ = d;
    };
    void  print(string & s){//测试用
        
        s =  to_string(placeId_) + " " + to_string(pos_.first) +" "+ to_string(pos_.second) + " "+ to_string(lefttime_) + " " +  
        to_string(cumsumestate_) + " "+ to_string( productstate_);
        //cout<<placeId_<<" "<<pos_.first<<" "<<pos_.second<<" "<<lefttime_<<" "<<cumsumestate_<<" "<<productstate_<<endl;
    }
};
vector<Workplace>workstation;
unordered_map<int,int>mp;//记录几号工作台有几个

string mapid;
unordered_map<string,int> mapsize = {{"1555559555A777A59787944A777A6644449666644466623", 1}, {"656712233A581A4A185A332217656", 2}, {"355555124446666124436A166A9AA6443642666212434533554551", 3},
             {"7111222333565656AAAA84", 4}};
/*读取第一帧状态*/
bool Initinput() {
    cin>>framid>>money;
    int work_num;
    cin>>work_num;

    for(int i = 0;i < work_num;i++){
        /*工作台状态*/
        int ch_;
        double x,y;
        int lefttime;
        int cumsumer;
        int product;
        cin>>ch_>>x>>y>>lefttime>>cumsumer>>product;
        if(mapsize[mapid] == 1){
            if(i == 8 || i == 15 ||  i == 17 || i == 35 || i == 39)
            workstation.push_back({i,ch_,x,y,lefttime,cumsumer,product}); 
            else workstation.push_back({i,ch_,x,y,lefttime,126,product}); 
        }
        else if(mapsize[mapid] == 2){
            if(i !=2 && i!=15 && i!=24 && i != 1 && i != 6 && i!= 8 && i!= 13 && i != 14 && i!= 17 && i!= 19 )
            workstation.push_back({i,ch_,x,y,lefttime,cumsumer,product}); 
            else workstation.push_back({i,ch_,x,y,lefttime,126,product}); 
        }
        else if(mapsize[mapid] == 4){
            if(i !=11 && i!=14)
            workstation.push_back({i,ch_,x,y,lefttime,cumsumer,product}); 
            else workstation.push_back({i,ch_,x,y,lefttime,126,product}); 
        }
        else workstation.push_back({i,ch_,x,y,lefttime,cumsumer,product}); 
        mp[ch_]++;
   }
    for(int i = 0;i<4; i++){
        /*小车状态*/
        int id;
        int ch_;
        double time;
        double col;
        double ra;
        double ve_x;
        double ve_y;
        double to;
        double x,y;
        cin>>id>>ch_>>time>>col>>ra>>ve_x>>ve_y>>to>>x>>y;
        Robot.push_back({id,ch_,time,col,ra,ve_x,ve_y,to,x,y});
    }
    string end;
    cin>>end;
    if(end == "OK\n"){return true;}
    return false;
}

/*更新每一帧状态*/
bool updateFrame(){
    cin>>framid>>money;
    int work_num;
    cin>>work_num;

    for(int i = 0;i < work_num;i++){
        /*工作台状态*/
        int ch_;
        double x,y;
        int lefttime;
        int cumsumer;
        int product;
        cin>>ch_>>x>>y>>lefttime>>cumsumer>>product;
        if(mapsize[mapid] == 1){
            if(i == 8 || i == 15 || i == 17 || i == 35 || i == 39)
            workstation[i] = {i,ch_,x,y,lefttime,cumsumer,product}; 
            else workstation[i] ={i,ch_,x,y,lefttime,126,product}; 
        }
        else if(mapsize[mapid] == 2){
            if(i !=2 && i!=15 && i!=24 && i != 1 && i != 6 && i!= 8 && i!= 13 && i != 14 && i!= 17 && i!= 19 )
            workstation[i] = {i,ch_,x,y,lefttime,cumsumer,product}; 
            else workstation[i] ={i,ch_,x,y,lefttime,126,product}; 
        }
        else if(mapsize[mapid] == 4){
            if(i != 11 && i!=14)
            workstation[i] = {i,ch_,x,y,lefttime,cumsumer,product}; 
            else workstation[i] ={i,ch_,x,y,lefttime,126,product}; 
        }
        else workstation[i] = {i,ch_,x,y,lefttime,cumsumer,product}; 
   }
    for(int i = 0;i<4; i++){
        /*小车状态*/
        int id;
        int ch_;
        double time;
        double col;
        double ra;
        double ve_x;
        double ve_y;
        double to;
        double x,y;
        cin>>id>>ch_>>time>>col>>ra>>ve_x>>ve_y>>to>>x>>y;
        Robot[i] = {id,ch_,time,col,ra,ve_x,ve_y,to,x,y};
    }
    string end;
    cin>>end;
    if(end == "OK\n")return true;
    return false;
}




/*在机器人坐标系中，目标工作台的角度*/
/*double convertAngle(double xr, double yr, double xw, double yw)
{

    double xa = xw - xr;
    double ya = yw - yr;
    double x = 1.0;
    double y = 0.0;
    double fenzi = xa * y + ya * x;
    double fenmu = sqrt(xa * xa + ya*ya) * 1.0;
    double res = acos(fenzi / (fenmu));
    
    double flag = xa * y - x * ya;
    if(flag > 0)return 3.1415926  - res;
    return res;
}*/

void Log(vector<int>& s){
    FILE* fp;

    fp = fopen("./log.txt","aw");

  //  string s = "";
  //  workstation[2].print(s);
  for(int i = 0;i<s.size();i++)
    fprintf(fp, "%d  ", s[i]);
    fprintf(fp, "\n");
    fclose(fp);
}

void Log(string s){
    FILE* fp;

    fp = fopen("./log.txt","aw");

  //  string s = "";
  //  workstation[2].print(s);
    fprintf(fp, "%s\n", s.c_str());

    fclose(fp);
}



double calDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1- x2) * (x1 - x2) + (y1-y2) * (y1 - y2));
}

double anglecontro(bool is_carry, pdd robot_loc, double robot_angle, pdd bench_loc, int robot_id, double &angle_speed, int angle_co = 50)
{


    double robotX = robot_loc.first, robotY = robot_loc.second;
    double workstationX = bench_loc.first, workstationY = bench_loc.second;

    pdd robotToworkstation = {workstationX - robotX, workstationY - robotY};
    double robotToworkstation_a = atan2(robotToworkstation.second, robotToworkstation.first);

    double anglediff = abs(robot_angle - robotToworkstation_a) * angle_co;

    if(robotToworkstation_a >= 0 && robot_angle >= 0){
        if(robot_angle >= robotToworkstation_a){
            angle_speed = -1 * anglediff;
        }
        else{
            angle_speed = anglediff;
        }
    }
    else if(robotToworkstation_a < 0 &&robot_angle < 0){
        if(robot_angle >= robotToworkstation_a){
            angle_speed = -1 * anglediff;
        }
        else{
            angle_speed = anglediff;
        }
    }
    else if(robotToworkstation_a <= 0 && robot_angle >= 0){
        if(abs(robotToworkstation_a) + abs(robot_angle) <= M_PI){
            angle_speed = -anglediff;
        }
        else{
            angle_speed = anglediff;
        }
    }
    else{
        if(abs(robotToworkstation_a) + abs(robot_angle) <= M_PI){
            angle_speed = anglediff;
        }
        else{
            angle_speed = -anglediff;
        }
    }
    return angle_speed;
}

double bsizecontrol(bool is_carry, pdd robot_loc, double robot_angle, pdd bench_loc, int robot_id, double &line_speed, double to_x_l = 0.5,
                                double to_x_h = 49.5, double to_y_l = 0.5, double to_y_h = 49.5, double ang_diff = 1.5)
{
    if(mapsize[mapid] == 3){
        to_x_l = 1, to_x_h = 49, to_y_l = 1, to_y_h = 49;
    }
    
    double robotX = robot_loc.first, robotY = robot_loc.second;
    double workstationX = bench_loc.first, workstationY = bench_loc.second;

    pdd robotToworkstation = {workstationX - robotX, workstationY - robotY};
    double robotToworkstation_a = atan2(robotToworkstation.second, robotToworkstation.first);

    double distance = sqrt((robotX - workstationX) * (robotX - workstationX) + (robotY - workstationY) * (robotY - workstationY));

    if((distance <= 2 ||(robotX <= to_x_l || robotX >= to_x_h || robotY <= to_y_l || robotY >= to_y_h)) && abs(robot_angle - robotToworkstation_a)>= ang_diff){
        line_speed = 0;
    }
    return line_speed;

}

double distcontrol(bool is_carry, pdd robot_loc, double robot_angle, pdd bench_loc, int robot_id, double &line_speed)
{
    

    double robotX = robot_loc.first, robotY = robot_loc.second;
    double workstationX = bench_loc.first, workstationY = bench_loc.second;

    pdd robotToworkstation = {workstationX - robotX, workstationY - robotY};
    double robotToworkstation_a = atan2(robotToworkstation.second, robotToworkstation.first);

    double distance = sqrt((robotX - workstationX) * (robotX - workstationX) + (robotY - workstationY) * (robotY - workstationY));

    if(mapsize[mapid] == 3||mapsize[mapid] == 2)
    {
        if(distance <= 5 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 1 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 3 && abs(robot_angle - robotToworkstation_a) >= 0.3){
            line_speed = 0;
        }
    }
    else if(mapsize[mapid] == 4)
    {
        if(distance <= 3 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 1 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 1&& abs(robot_angle - robotToworkstation_a) >= 0.3){
            line_speed = 0;
        }
    }
    else
    {
        if(distance <= 3 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 2 && abs(robot_angle - robotToworkstation_a) >= 1.5){
            line_speed = 0;
        }
        if(distance <= 3 && abs(robot_angle - robotToworkstation_a) >= 0.3){
            line_speed = 0;
        }
    }

    return line_speed;
}
int Maxtimne = 0;//最大碰撞次数，次数少减速，次数多不需要减速
 
/*修正当前朝向为目标朝向*/
void makeRobotTowards(Node& a, Workplace& b, double& line_speed,double& angle_speed)
{
 
    double xa = a.pos_.first, ya = a.pos_.second;
    double xb = b.pos_.first, yb = b.pos_.second;

    double xatob = xb - xa;
    double yatob = yb - ya;

    double angle_atob = atan2(yatob, xatob);

    double distance = sqrt(xatob*xatob + yatob*yatob);
    line_speed = 6.0;
    angle_speed = 0.0;

    angle_speed = anglecontro(0,a.pos_,a.toward_,b.pos_,0,angle_speed);

    line_speed = bsizecontrol(0,a.pos_,a.toward_,b.pos_,0,line_speed);
     line_speed = distcontrol(0,a.pos_,a.toward_,b.pos_,0,line_speed);
   // if(distance >= 2)line_speed = 6;

   /* if(angle_atob >=0 && a.toward_ >=0){
        if(a.toward_ > angle_atob){
            angle_speed = -3.1415926;
        }
        else if(a.toward_ < angle_atob){
            angle_speed = 3.1415926;
        }
    }
    else if(a.toward_ < 0 && angle_atob < 0){
        if(a.toward_ > angle_atob){
            angle_speed = -3.1415926;
        }
        else if(a.toward_ < angle_atob){
            angle_speed = 3.1415926;
        }
    }
    else if(angle_atob < 0 && a.toward_ > 0){
        if(abs(angle_atob) + abs(a.toward_)<=3.1415926){
            angle_speed = -3.1415926;
        }
        else angle_speed = 3;
    }
    else{
        if(abs(angle_atob) + abs(a.toward_)<=3.1415926){
            angle_speed = 3.1415926;
        }
        else angle_speed = -3;
    }*/

    /*身上带东西时候在判断要不要检测碰撞*/
    //撞墙判断
 /* if(mapsize[mapid] == 3){
    if(a.thingNum_!=0 &&((xa <= 1 || xa >=49 )|| (ya <=1|| (ya >= 49 ))))
    {
        if(xa <= 0.6 && a.toward_ >= 3.1415926 / 4 ){
           line_speed = 5;
            angle_speed = -3.1415926;
        }
        else if(xa <= 0.6&& a.toward_ < -3.1415926 / 4 ){
          line_speed = 5 ;
            angle_speed = 3.1415926;
        }
        if(xa >=49.4 && a.toward_ < 3.1415926/4 &&a.toward_ >= 0)
        {
            line_speed = 5 ;
            angle_speed = 3.1415926;
        }
        else if(xa >=49.4&& a.toward_ > -3.1415926/4 &&a.toward_ <=0 ){
           line_speed = 5 ;
            angle_speed = -3.1415926;
        }
        if(ya <= 0.6 &&( a.toward_ > -3.1415926 / 4 && a.toward_ <= 0)){
           line_speed = 5;
            angle_speed = 3.1415926;
        }
        else if(ya <= 0.6&&(a.toward_ < -3.1415926/4)){
           line_speed = 5;
            angle_speed = -3.1415926;
        }
        if(ya >=49.4 && (a.toward_>0&&a.toward_<3.1415926 / 4))
        {
          line_speed = 5 ;
            angle_speed = -3.1415926;
        }
        if(ya  >=49.6 && (a.toward_> 3.1415926 / 4)){
            line_speed = 5 ;
            angle_speed = 3.1415926;
        }
    }*/
      
    if(mapsize[mapid]==1 ){
        for(int i = 0;i<4;i++){
            if(Robot[i].pos_.first == a.pos_.first && Robot[i].pos_.second == a.pos_.second)continue;
            if(calDistance(a.pos_.first,a.pos_.second, Robot[i].pos_.first, Robot[i].pos_.second) <= 1.1995&& a.thingNum_ != 0){
                if(a.thingNum_ != 0 ){
                    Maxtimne++;
                    if(Maxtimne < 10)
                    {
                        line_speed  = 0.9 * line_speed;
                        angle_speed = -3.1415926/2;
                    }
                    else{
                        angle_speed = -3.1415926/2;
                    }
                }
            }
        }
    }  //车之间判断

  

}

/*小车之间检测距离和边缘距离函数*/



unordered_map<int,set<int>>everystationmater = {{4,{1,2}},{5,{1,3}},{6,{2,3}},{7,{4,5,6}},{8,{7}},{9,{1,2,3,4,5,6,7}}};

vector<int> lackmaterial_ch(int workstation_ch, int materstate)
{
    set<int>Havemater;
    for(int i = 1;i<=10;i++){
        if(((materstate >> i)&1) == 1){
            Havemater.insert(i);
        }
    }
    set<int>lackmater;
    for(auto m : everystationmater[workstation_ch]){
        if(Havemater.count(m) == 0){
            lackmater.insert(m);
        }
    }

    vector<int>res(lackmater.begin(),lackmater.end());

    res.push_back(Havemater.size());
    return res;

}

vector<int>NothingsRobottarget(4,-1);
vector<int>EveryRobot_target_workstation(4,-1);


void init_frame(vector<int>&charactor_lack_id, vector<int>&nowRobotcarry,unordered_map<int,set<tuple<int, pdd, int>>>&evertmatterlack,  
        unordered_map<int,set<tuple<int, pdd>>>&finishWorkstation, vector<int>&evertmatterlack_num)
{
  //  unordered_map<int,set<tuple<int, pdd, int>>>evertmatterlack;
  //  unordered_map<int,set<tuple<int, pdd>>>finishWorkstation;

    for(int i = 0;i<workstation.size();i++){
        /*编号 1 2 3 的工作台只需要考虑产品还没好，好的加入到就绪列表*/
        if(workstation[i].charactor_ == 1 ||workstation[i].charactor_ == 2 || workstation[i].charactor_ == 3){
            if(workstation[i].productstate_ == 1){
                finishWorkstation[workstation[i].charactor_].insert({workstation[i].placeId_, workstation[i].pos_});
            }
            continue;
        }
        vector<int>temp = lackmaterial_ch(workstation[i].charactor_,workstation[i].cumsumestate_);
        vector<int>lack(temp.begin(), temp.end() - 1);
      /*  if(workstation[i].charactor_ == 4){
            Log(temp);
        }*/
        int hadlen = temp[temp.size() - 1];
        for(auto& c: lack){
            
            charactor_lack_id[c] += 1;
            evertmatterlack[c].insert({workstation[i].placeId_, workstation[i].pos_, hadlen});
        }
        if(workstation[i].charactor_ != 8 && workstation[i].charactor_ != 9)
        {
            if(workstation[i].productstate_ == 1){
                finishWorkstation[workstation[i].charactor_].insert({workstation[i].placeId_, workstation[i].pos_});
            }
        }

        
    }

 //   vector<int>nowRobotcarry(10,0);
//    vector<int>evertmatterlack_num(8,0);

    for(auto& c: evertmatterlack){

        evertmatterlack_num[c.first] = c.second.size();
    }

    for(auto& c: Robot){
        nowRobotcarry[c.thingNum_] += 1;
    }

    for(int i = 0;i<nowRobotcarry.size();i++){
        if(i != 0 && nowRobotcarry[i] !=0){
                    
            evertmatterlack_num[i] -= nowRobotcarry[i];
        }
    }
  

    for(int i = 0;i<NothingsRobottarget.size();i++){
        if(NothingsRobottarget[i] != -1){
            charactor_lack_id[workstation[NothingsRobottarget[i]].charactor_] -= 1;
        }
    }

}




void task_process2(vector<vector<double>>& ROBOTaction, vector<int>&evertmatterlack_num, unordered_map<int,set<tuple<int, pdd>>>&finishWorkstation
    ,unordered_map<int,set<tuple<int, pdd, int>>>&evertmatterlack)
{
    for(int i = 0; i<4;i++){
        ROBOTaction[i] =  {6.0,-1.0,-1};
    }


    if(Robot[0].thingNum_ == 0)
    {
        int sum = 0;
        for(auto c: evertmatterlack_num)sum += c;
        if(sum == 0){
            ROBOTaction[0] = {6,-1,-1};
        }
        else{
            vector<pair<double, int>>TargetWorkstation_id;
            for(auto c:finishWorkstation){
                if(evertmatterlack_num[c.first] != 0){
                    for(auto v: c.second){
                        TargetWorkstation_id.push_back({calDistance(Robot[0].pos_.first, Robot[0].pos_.second, get<1>(v).first, get<1>(v).second),get<0>(v)});
                    }
                }
            }

            if(TargetWorkstation_id.size()){
                sort(TargetWorkstation_id.begin(), TargetWorkstation_id.end());
                
                if(TargetWorkstation_id[0].second == Robot[0].inWorkstateId_){
                    ROBOTaction[0][2] = 0;
                    Robot[0].thingNum_ = workstation[TargetWorkstation_id[0].second].charactor_;
                }
                else{
                    double line_speed = 0.0;
                    double angle_speed = 0.0;
                    makeRobotTowards(Robot[0], workstation[TargetWorkstation_id[0].second], line_speed,angle_speed);
                    ROBOTaction[0][0] = line_speed;
                    ROBOTaction[0][1] = angle_speed;
                    NothingsRobottarget[0] = workstation[TargetWorkstation_id[0].second].placeId_;
                    evertmatterlack_num[workstation[TargetWorkstation_id[0].second].charactor_] -=1;
                }
            }
        }

    }
    else{
        vector<pair<double,int>>Need_0;

        for(auto c: evertmatterlack[Robot[0].thingNum_]){
            if(mapsize[mapid] == 4 && get<0>(c) == 17)
            Need_0.push_back({calDistance(Robot[0].pos_.first, Robot[0].pos_.second, get<1>(c).first, get<1>(c).second)/ (10000 + 1), get<0>(c)});
            else  Need_0.push_back({calDistance(Robot[0].pos_.first, Robot[0].pos_.second, get<1>(c).first, get<1>(c).second)/ (get<2>(c) + 1), get<0>(c)});
        }
        sort(Need_0.begin(), Need_0.end());
        if(Need_0[0].second == Robot[0].inWorkstateId_)
        {
            ROBOTaction[0][2] = 1; // 卖
            Robot[0].thingNum_ = 0;
        }
        else{
            
            int target_station_id = Need_0[0].second;
            double line_speed = 0.0;
            double angle_speed = 0.0;
            makeRobotTowards(Robot[0], workstation[target_station_id], line_speed,angle_speed);
            ROBOTaction[0][0] = line_speed;
            ROBOTaction[0][1] = angle_speed;
            EveryRobot_target_workstation[0] = target_station_id;
        }
    }




    if(Robot[1].thingNum_ == 0)
    {
        int sum = 0;
        for(auto c: evertmatterlack_num)sum += c;
        if(sum == 0){
            ROBOTaction[1] = {6,-1,-1};
        }
        else{

            vector<pair<double,int>>TargetWorkstation_id;
            for(auto c:finishWorkstation){
                if(evertmatterlack_num[c.first] != 0){
                    for(auto v: c.second){
                        TargetWorkstation_id.push_back({calDistance(Robot[1].pos_.first, Robot[1].pos_.second, get<1>(v).first, get<1>(v).second),get<0>(v)});
                    }
                }
            }
            if(TargetWorkstation_id.size()){
                sort(TargetWorkstation_id.begin(), TargetWorkstation_id.end());
                for(auto c : TargetWorkstation_id){
                    if(c.second != NothingsRobottarget[0]){
                        NothingsRobottarget[1] = c.second;
                        break;
                    }
                }
                if(NothingsRobottarget[1]!=-1){
                    if(NothingsRobottarget[1] == Robot[1].inWorkstateId_){
                        ROBOTaction[1][2] = 0;
                        Robot[1].thingNum_ = workstation[NothingsRobottarget[1]].charactor_;
                    }
                    else{
                        double line_speed = 0.0;
                        double angle_speed = 0.0;
                        makeRobotTowards(Robot[1], workstation[NothingsRobottarget[1]], line_speed,angle_speed);
                        ROBOTaction[1][0] = line_speed;
                        ROBOTaction[1][1] = angle_speed;
                    // NothingsRobottarget[1] = workstation[TargetWorkstation_id[0].second].placeId_;
                        evertmatterlack_num[workstation[NothingsRobottarget[1]].charactor_] -=1;
                    }
                }

            }
        }
    }
    else{
       vector<pair<double,int>>Need_1;

        for(auto c: evertmatterlack[Robot[1].thingNum_]){
            if(mapsize[mapid] == 4 && get<0>(c) == 17)
            Need_1.push_back({calDistance(Robot[1].pos_.first, Robot[1].pos_.second, get<1>(c).first, get<1>(c).second)/ (get<2>(c) + 1), get<0>(c)});
            else Need_1.push_back({calDistance(Robot[1].pos_.first, Robot[1].pos_.second, get<1>(c).first, get<1>(c).second)/ (get<2>(c) + 1), get<0>(c)});
        }
        sort(Need_1.begin(), Need_1.end());

        if(Robot[1].thingNum_ == Robot[0].thingNum_)
        {
            for(auto i : Need_1){
                if(i.second != EveryRobot_target_workstation[0]){
                    EveryRobot_target_workstation[1] = i.second;
                    break;
                }
            }
        }
        else{
            EveryRobot_target_workstation[1] = Need_1[0].second;
        }
        if(Robot[1].inWorkstateId_ == EveryRobot_target_workstation[1]){
            ROBOTaction[1][2] = 1;
            Robot[1].thingNum_ = 0;
        }
        else{
            int target_station_id = EveryRobot_target_workstation[1];
            double line_speed = 0.0;
            double angle_speed = 0.0;
            makeRobotTowards(Robot[1], workstation[target_station_id], line_speed,angle_speed);
            ROBOTaction[1][0] = line_speed;
            ROBOTaction[1][1] = angle_speed;
        }
    }


  /**/  if(Robot[2].thingNum_ == 0)
    {
        int sum = 0;
        for(auto c: evertmatterlack_num)sum += c;
        if(sum == 0){
            ROBOTaction[2] = {6,-1,-1};
        }
        else{

            vector<pair<double, int>>TargetWorkstation_id;
            for(auto c:finishWorkstation){
                if(evertmatterlack_num[c.first] != 0){
                    for(auto v: c.second){
                        TargetWorkstation_id.push_back({calDistance(Robot[2].pos_.first, Robot[2].pos_.second, get<1>(v).first, get<1>(v).second),get<0>(v)});
                    }
                }
            }

            if(TargetWorkstation_id.size()){
                sort(TargetWorkstation_id.begin(), TargetWorkstation_id.end());
                
                for(auto c: TargetWorkstation_id){
                    if(c.second != NothingsRobottarget[0] && c.second != NothingsRobottarget[1]){
                        NothingsRobottarget[2] = c.second;
                        break;
                    }
                }
            if(NothingsRobottarget[2]!=-1){
                    if(NothingsRobottarget[2] == Robot[2].inWorkstateId_){
                        ROBOTaction[2][2] = 0;
                        Robot[2].thingNum_ = workstation[NothingsRobottarget[2]].charactor_;
                    }
                    else{
                        int targetid = NothingsRobottarget[2];
                        double line_speed = 0.0;
                        double angle_speed = 0.0;
                        makeRobotTowards(Robot[2], workstation[targetid], line_speed,angle_speed);
                        ROBOTaction[2][0] = line_speed;
                        ROBOTaction[2][1] = angle_speed;
                        evertmatterlack_num[workstation[NothingsRobottarget[2]].charactor_] -=1;
                    }
                }

            }
        }
    }
    else{
       vector<pair<double,int>>Need_2;

        for(auto c: evertmatterlack[Robot[2].thingNum_]){
            if(mapsize[mapid] == 4 && get<0>(c) == 17)
            Need_2.push_back({calDistance(Robot[2].pos_.first, Robot[2].pos_.second, get<1>(c).first, get<1>(c).second)/ (10000 + 1), get<0>(c)});
            else Need_2.push_back({calDistance(Robot[2].pos_.first, Robot[2].pos_.second, get<1>(c).first, get<1>(c).second)/ (get<2>(c) + 1), get<0>(c)});
        }
        sort(Need_2.begin(),Need_2.end());

        if(Robot[2].thingNum_ ==  Robot[0].thingNum_ && Robot[2].thingNum_ != Robot[1].thingNum_){
            for(auto i: Need_2){
                if(i.second != EveryRobot_target_workstation[0]){
                    EveryRobot_target_workstation[2] = i.second;
                    break;
                }
            }
        }
        else if(Robot[2].thingNum_ != Robot[0].thingNum_ && Robot[2].thingNum_ == Robot[1].thingNum_){
            for(auto i : Need_2){
                if(i.second != EveryRobot_target_workstation[1]){
                    EveryRobot_target_workstation[2] = i.second;
                    break;
                }
            }
        }
        else if(Robot[2].thingNum_ != Robot[0].thingNum_ && Robot[2].thingNum_ != Robot[1].thingNum_){
            EveryRobot_target_workstation[2] = Need_2[0].second;
        }

        else{
            for(auto i : Need_2){
                if(i.second != EveryRobot_target_workstation[0] && i.second != EveryRobot_target_workstation[1]){
                    EveryRobot_target_workstation[2] = i.second;
                    break;
                }
            }
        }
        if(Robot[2].inWorkstateId_ == EveryRobot_target_workstation[2])
        {
            ROBOTaction[2][2] = 1; // 卖
            Robot[2].thingNum_ = 0;
        }
        else{
            int target_station_id = EveryRobot_target_workstation[2];
            double line_speed = 0.0;
            double angle_speed = 0.0;
            makeRobotTowards(Robot[2], workstation[target_station_id], line_speed,angle_speed);
            ROBOTaction[2][0] = line_speed;
            ROBOTaction[2][1] = angle_speed;
        }
    }

    if(Robot[3].thingNum_ == 0)
    {
        int sum = 0;
        for(auto c: evertmatterlack_num)sum += c;
        if(sum == 0){
            ROBOTaction[3] = {6,-1,-1};
        }
        else{

            vector<pair<double,int>>TargetWorkstation_id;
            for(auto c:finishWorkstation){
                if(evertmatterlack_num[c.first] != 0){
                    for(auto v: c.second){
                        TargetWorkstation_id.push_back({calDistance(Robot[3].pos_.first, Robot[3].pos_.second, get<1>(v).first, get<1>(v).second),get<0>(v)});
                    }
                }
            }
            
            if(TargetWorkstation_id.size()){
                sort(TargetWorkstation_id.begin(),TargetWorkstation_id.end());

                for(auto c:TargetWorkstation_id){
                    if(c.second != NothingsRobottarget[0] && c.second != NothingsRobottarget[1]
                        && c.second != NothingsRobottarget[2])
                    {

                        NothingsRobottarget[3] = c.second;
                    
                        break;
                    }
                }
                if(NothingsRobottarget[3] != -1){
                    if(NothingsRobottarget[3] ==  Robot[3].inWorkstateId_ ){
                        ROBOTaction[3][2] = 0;
                        Robot[3].thingNum_ = workstation[NothingsRobottarget[3]].charactor_;
                    }
                    else{
                        double line_speed = 0.0;
                        double angle_speed = 0.0;
                        makeRobotTowards(Robot[3], workstation[NothingsRobottarget[3]], line_speed,angle_speed);
                        ROBOTaction[3][0] = line_speed;
                        ROBOTaction[3][1] = angle_speed;
                        evertmatterlack_num[workstation[NothingsRobottarget[3]].charactor_] -=1;
                    }
                }
            }
        }
    }
    else{
        vector<pair<double,int>>Need_3;

        for(auto c: evertmatterlack[Robot[3].thingNum_]){
            if(mapsize[mapid] == 4 && get<0>(c) == 17)
            Need_3.push_back({calDistance(Robot[3].pos_.first, Robot[3].pos_.second, get<1>(c).first, get<1>(c).second)/ (10000 + 1), get<0>(c)});
            else Need_3.push_back({calDistance(Robot[3].pos_.first, Robot[3].pos_.second, get<1>(c).first, get<1>(c).second)/ (get<2>(c) + 1), get<0>(c)});
        }
        sort(Need_3.begin(), Need_3.end());

        if(Robot[3].thingNum_ != Robot[0].thingNum_ && Robot[3].thingNum_ != Robot[1].thingNum_ && Robot[3].thingNum_ != Robot[2].thingNum_)
        {
            EveryRobot_target_workstation[3] = Need_3[0].second;
        }

        else if(Robot[3].thingNum_ == Robot[0].thingNum_ &&Robot[3].thingNum_ != Robot[1].thingNum_ && Robot[3].thingNum_ != Robot[2].thingNum_)
        {
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[0]){
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }

        else if(Robot[3].thingNum_ != Robot[0].thingNum_ && Robot[3].thingNum_ == Robot[1].thingNum_ && Robot[3].thingNum_ != Robot[2].thingNum_)
        {
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[1]){
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }

        else if(Robot[3].thingNum_ != Robot[0].thingNum_ && Robot[3].thingNum_ != Robot[1].thingNum_ && Robot[3].thingNum_ == Robot[2].thingNum_)
        {
            for(auto i: Need_3){
                if(i.second != EveryRobot_target_workstation[2]){
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }

        else if(Robot[3].thingNum_ == Robot[0].thingNum_ && Robot[3].thingNum_ == Robot[1].thingNum_ &&Robot[3].thingNum_ != Robot[2].thingNum_)
        {
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[0] && i.second != EveryRobot_target_workstation[1]){
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }

        else if(Robot[3].thingNum_ == Robot[0].thingNum_ && Robot[3].thingNum_!= Robot[1].thingNum_ && Robot[3].thingNum_ == Robot[2].thingNum_)
        {
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[0] && i.second != EveryRobot_target_workstation[2]){
                    EveryRobot_target_workstation[3] = i.second;
                    
                    break;
                }
            }
        }

        else if(Robot[3].thingNum_ != Robot[0].thingNum_ && Robot[3].thingNum_ == Robot[1].thingNum_ && Robot[3].thingNum_ == Robot[2].thingNum_)
        {
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[1] && i.second != EveryRobot_target_workstation[2]){
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }
        
        else{
            for(auto i : Need_3){
                if(i.second != EveryRobot_target_workstation[0] && i.second != EveryRobot_target_workstation[1] 
                    && i.second != EveryRobot_target_workstation[2])
                {
                    EveryRobot_target_workstation[3] = i.second;
                    break;
                }
            }
        }
   
        if( Robot[3].inWorkstateId_ == EveryRobot_target_workstation[3])
        {
            ROBOTaction[3][2] = 1; // 卖
            Robot[3].thingNum_ = 0;
        }
        else{
            int target_station_id = EveryRobot_target_workstation[3];
            double line_speed = 0.0;
            double angle_speed = 0.0;
            makeRobotTowards(Robot[3], workstation[target_station_id], line_speed,angle_speed);
            ROBOTaction[3][0] = line_speed;
            ROBOTaction[3][1] = angle_speed;
        }
    }
}


int targetworkstation(int robot_id, int jiahsewupin)
{
    vector<pair<double, int>>needall1;

    for(auto c:evertmatterlack[jiahsewupin]){
        int weight  = pow(2,get<2>(c));
        needall1.push_back(calDistance(Robot[robot_id].pos_.first, Robot[robot_id].pos_.second,get<1>(c).first, get<1>(c).second)/weight, get<0>(c));
    }

    sort(needall1.begin(), needall1.end());

    int jiashemubiaogongzuotai = 10000;

    for(auto c: needall1){
        bool flag = false;
        for(int i = 0;i<4;i++){
            if(i == robot_id)continue;
            else{
                if (EveryRobot_target_workstation[i] == c.second && Robot[I].thingNum_ == jiahsewupin
                    && (workstation[c.second].charactor_ != 8 &&  workstation[c.second].charactor_ != 9){
                        flag = true;
                        break;
                    }
            }
        }
        if(flag)continue;
        else
        {
            jiashemubiaogongzuotai = c.second;
            break;
        }

    }
    return assumption_target_station_id;
}

void task_process3(vector<vector<double>>& ROBOTaction, vector<int>&evertmatterlack_num, unordered_map<int,set<tuple<int, pdd>>>&finishWorkstation
    ,unordered_map<int,set<tuple<int, pdd, int>>>&evertmatterlack)

{
    for(int i = 0; i<4;i++){
        ROBOTaction[i] =  {6.0,-1.0,-1};
        }

    for(int i = 0 ;i< 4;i++){
        if(Robot[i].thingNum_ == 0){
            if(NothingsRobottarget[i] != -1){
                if(NothingsRobottarget[i] == Robot[i].placeId_){
                    int assumption_bench  = targetworkstation(i, workstation[Robot[i].placeId_].charactor_);
                    if(framid <= 9000){
                        ROBOTaction[i][2] = 0;
                        NothingsRobottarget[i] = -1;
                        EveryRobot_target_workstation[i] = jiahsewupin;
                        Robot[i].thingNum_ = workstation[Robot[i].placeId_].charactor_;
                    }
                }
                else{
                    double line_speed = 6.0;
                    double angle_speed = 0.0;
                    makeRobotTowards(Robot[i], workstation[NothingsRobottarget[i]],line_speed, angle_speed);
                    ROBOTaction[i][0] = line_speed;
                    ROBOTaction[i][1] = angle_speed;
                }
            }
            else{
                vector<pair<double,int>>Needall;
                for(auto  c : finishWorkstation){
                    if(evertmatterlack_num[c.first] > 0){
                        for(auto v: c.second){
                            vector<int>weight = {0,1,1,1,1,1,1,1};
                            weight = weight[workstation[v.first].charactor_];
                            Needall.push_back({calDistance(Robot[i]pos_.first, Robot.pos_.second, get<1>(v).first, get<1>(v).second)/weight, get<0>(v)});

                        }
                    }
                }
                if(Needall.size()){
                    sort(Needall.begin(), Needall.end());
                    for(auto c: Needall){
                        bool flag = false;
                        for(int i = 0 ;i<NothingsRobottarget.size();i++){
                            if(c,second == NothingsRobottarget[i]){
                                flag = true;
                            }
                        }
                        if(!flag || (workstation[c.second].charactor_!= 1 && workstation[c.second].charactor_!= 2 &&workstation[c.second].charactor_!= 3))
                        {
                            NothingsRobottarget[i] = c.second;
                            break;
                        }
                    }
                    if(NothingsRobottarget[i] != -1 && (workstation[NothingsRobottarget[i].thingNum_] != 8 && workstation[NothingsRobottarget[i].thingNum_] != 9))
                    {
                        evertmatterlack_num[workstation[NothingsRobottarget[i]].thingNum_] -=1 ;
                    }
                }
            }
        }
        else{
            if(Robot[i].placeId_ != -1 && Robot[i].placeId_ == EveryRobot_target_workstation[i]){
                ROBOTaction[i][2] = 1;
                EveryRobot_target_workstation[i] = -1;
                NothingsRobottarget[i] = -1;
                wait_for_del = 10000;
                for(auto c: evertmatterlack[Robot[i].thingNum_]){
                    if(get<0>(c.second) == Robot[i].placeId_){
                        wait_for_del = c.first;
                        break;
                    }
                }
                evertmatterlack[Robot[i].thingNum_].erase(wait_for_del);
            }
            else{
                double line_speed = 6.0;
                double angle_speed = 0.0;
                makeRobotTowards(Robot[i], workstation[EveryRobot_target_workstation[i]], line_speed, angle_speed);
                ROBOTaction[i][0] = line_speed;
                ROBOTaction[i][1] = angle_speed;
            }
        }
    }

}









/*bool readUntilOK() {
    char line[1024];
    string s = "";
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
    }
    return false;
}*/


string readUntilOK() {
    char line[1024];
    string s;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            break;
        }
        for(int i = 0;i<100;i++){
            if(line[i] != '.')s += line[i];
        }
        //do something
    }
    return s;
}

int main(){

    mapid = readUntilOK();
    //Log(mapid);
    puts("OK");
    fflush(stdout);
    uint64_t j = 1;
    while(true)
    {
        if(j == 1)Initinput();
        else updateFrame();

      
        vector<int>charactor_lack_id(8,0);
        vector<int>nowRobotcarry(10,0);
        unordered_map<int,set<tuple<int, pdd, int>>>evertmatterlack;
        evertmatterlack.clear();
        unordered_map<int,set<tuple<int, pdd>>>finishWorkstation;
        finishWorkstation.clear();
        vector<int>evertmatterlack_num(8,0);
        init_frame(charactor_lack_id,nowRobotcarry,evertmatterlack,finishWorkstation,evertmatterlack_num);
        
        vector<vector<double>>ROBOTaction(4);
        if(mapsize[mapid] == 1) task_process3(ROBOTaction, evertmatterlack_num, finishWorkstation, evertmatterlack);
        else task_process2(ROBOTaction, evertmatterlack_num, finishWorkstation, evertmatterlack);
        printf("%d\n", framid);
        
  
        for(int i = 0; i <4;i++)
        {

            printf("forward %d %f\n", i, ROBOTaction[i][0]);
            printf("rotate %d %f\n", i, ROBOTaction[i][1]);
            if(mapsize[mapid] == 1)
            {
                if(framid <= 8867)
                {
                    if(ROBOTaction[i][2] == 0)printf("buy %d \n", i);
                    else if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
                else{
                    if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
            }
            else if(mapsize[mapid] == 2)
            {
                if(mapsize[mapid] == 2 &&  framid <= 8720)
                {
                    if(ROBOTaction[i][2] == 0)printf("buy %d \n", i);
                    else if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
                else{
                    if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
            }
           /* else if(mapsize[mapid] == 3)
            {
                if(mapsize[mapid] == 3 &&  framid <= 88500)
                {
                    if(ROBOTaction[i][2] == 0)printf("buy %d \n", i);
                    else if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
                else{
                    if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
            }*/
            else
            {
                if(mapsize[mapid] != 2 &&framid <= 8800)
                {
                    if(ROBOTaction[i][2] == 0)printf("buy %d \n", i);
                    else if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
                else{
                    if(ROBOTaction[i][2] == 1)printf("sell %d \n", i);
                }
            }


            
        }
        
        
        /*printf("%d\n", framid);
        for(int robotId = 0; robotId < 4; robotId++){
            printf("forward %d %f\n", robotId, 1.5);
            printf("rotate %d %f\n", robotId, -3.14);
        }*/
        printf("OK\n");
        fflush(stdout);
        j++;
    }
    return 0;
}

