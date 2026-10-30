#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include<iostream>
using std::placeholders::_1;
using namespace std;
class wheeltec_joy : public rclcpp::Node
{
	
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    std_msgs::msg::Float64 vlinear_x; //默认值
    std_msgs::msg::Float64 vlinear_z;
public:
    wheeltec_joy() 
: Node ("wheeltec_joy")
{
    RCLCPP_INFO(this->get_logger(),"wheeltec_joy111");
   //读取参数服务器中的变量值
     flag_mec=0;
    
   this->declare_parameter<int>("axis_linear", 1);//默认axes[1]接收速度
   this->declare_parameter<int>("axis_angular", 0);   //默认axes[0]接收角度
   this->declare_parameter<double>("v_linear", 0.3); //默认线速度0.3 m/s  
   this->declare_parameter<double>("v_angular", 1);// 默认角速度1 单位rad/s
 
   this->get_parameter("axis_linear", axis_linear);
   this->get_parameter("axis_angular", axis_angular);
   this->get_parameter("v_linear", v_linear);
   this->get_parameter("v_angular", v_angular);

   cur_linear_speed = v_linear;     //当前速度初始值取自参数
   cur_angular_speed = v_angular;
         
    pub =  this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub =  this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&wheeltec_joy::callback, this, _1)); //订阅手柄发来的数据
 
} 


private:
    //void callback(const sensor_msgs::msg::Joy::SharedPtr Joy);
    void callback(const sensor_msgs::msg::Joy::SharedPtr Joy)   //键值回调函数
	 {
	   double vangle_key,linera_key;
	   geometry_msgs::msg::Twist v;

	   //摇杆A：axes[1]前进后退
	   linera_key = Joy->axes[axis_linear];
	   //摇杆B：axes[3]控制转向
	   vangle_key = Joy->axes[axis_angular];

	   //计算与上次回调的时间间隔，用于长按连续加减速的节流
	   rclcpp::Time now = this->get_clock()->now();
	   double dt = 0.0;
	   if(last_time.nanoseconds() > 0)
	     dt = (now - last_time).seconds();
	   last_time = now;

	   //油门：axes[2]/axes[5]扳机，初始值1.0，按到底为-1.0 
	   //长按时每隔repeat_interval秒触发一次，每次加/减0.02 m/s
	   bool acce_pressed = (Joy->axes[2] < trigger_threshold);   //axes[2]按下=加速
	   bool decel_pressed = (Joy->axes[5] < trigger_threshold);  //axes[5]按下=减速

	   if(acce_pressed)
	   {
	     if(!last_acce_pressed) acce_timer = 0.0;        //刚按下，立即触发一次
	     else acce_timer += dt;
	     if(acce_timer >= repeat_interval || !last_acce_pressed)
	     {
	       cur_linear_speed += linear_step_inc;
	       acce_timer = 0.0;
	     }
	   }
	   else acce_timer = 0.0;

	   if(decel_pressed)
	   {
	     if(!last_decel_pressed) decel_timer = 0.0;      //刚按下，立即触发一次
	     else decel_timer += dt;
	     if(decel_timer >= repeat_interval || !last_decel_pressed)
	     {
	       cur_linear_speed -= linear_step_inc;
	       if(cur_linear_speed < 0) cur_linear_speed = 0;  
	       decel_timer = 0.0;
	     }
	   }
	   else decel_timer = 0.0;

	   last_acce_pressed = acce_pressed;
	   last_decel_pressed = decel_pressed;

	   //转向加减速：buttons[4]/buttons[5]，数字量0/1
	   //长按时每隔repeat_interval秒触发一次，每次加/减0.1 rad/s
	   bool turn_acce_pressed  = (Joy->buttons[4] == 1);
	   bool turn_decel_pressed = (Joy->buttons[5] == 1);

	   if(turn_acce_pressed)
	   {
	     if(!last_turn_acce_pressed) turn_acce_timer = 0.0;
	     else turn_acce_timer += dt;
	     if(turn_acce_timer >= repeat_interval || !last_turn_acce_pressed)
	     {
	       cur_angular_speed += angular_step_inc;
	       turn_acce_timer = 0.0;
	     }
	   }
	   else turn_acce_timer = 0.0;

	   if(turn_decel_pressed)
	   {
	     if(!last_turn_decel_pressed) turn_decel_timer = 0.0;
	     else turn_decel_timer += dt;
	     if(turn_decel_timer >= repeat_interval || !last_turn_decel_pressed)
	     {
	       cur_angular_speed -= angular_step_inc;
	       if(cur_angular_speed < 0) cur_angular_speed = 0; 
	       turn_decel_timer = 0.0;
	     }
	   }
	   else turn_decel_timer = 0.0;

	   last_turn_acce_pressed  = turn_acce_pressed;
	   last_turn_decel_pressed = turn_decel_pressed;

	   //限幅：当前线速度/角速度均不超过max_total_speed
	   if(cur_linear_speed > max_total_speed) cur_linear_speed = max_total_speed;
	   if(cur_angular_speed > max_total_speed) cur_angular_speed = max_total_speed;

	   double cur_linear = cur_linear_speed;
	   double cur_angular = cur_angular_speed;

	   //判断前进后退   
	   if(linera_key>0) 
	   {
	       dir=1;
	       vlinear_x.data=cur_linear;
	   } 
	   else if(linera_key<0)
	   {
	      dir=-1;
	      vlinear_x.data=-cur_linear;
	   }
	   else  
	   {
	    dir=1;
	    vlinear_x.data=0;
	   }
	   //判断左转右转，大于0为左转，小于0为右转
	   if(vangle_key>0)       vlinear_z.data=cur_angular;
	   else if(vangle_key<0)  vlinear_z.data=-cur_angular; 
	   else vlinear_z.data=0;
	   //处理数据
	   if(Joy->buttons[1]==1) //按下B键时，切换为麦轮车，可左右平移
	     flag_mec=1;
	   if(Joy->buttons[0]==1) //按下A键时，恢复正常转向模式
	     flag_mec=0;
	   if(flag_mec) 
	   {
	     v.linear.y=vlinear_z.data;
	     v.linear.x = vlinear_x.data;
	     v.angular.z=0;
	   }
	   else
	   {
	   v.linear.x = vlinear_x.data;
	   v.angular.z = dir*vlinear_z.data;
	   }
	   pub->publish(v);
	}


    //机器人的初始速度
    double v_linear=0.3;
    double v_angular=0.5;
    //手柄键值
    int axis_angular,axis_linear; 
    int dir,flag_mec;

    //油门/转向当前速度（初始为基础值，可通过加减档调整，下限为0，上限为max_total_speed）
    double cur_linear_speed=0.3;
    double cur_angular_speed=0.5;
    const double linear_step_inc=0.02;   //每次触发增加/减少0.02 m/s
    const double angular_step_inc=0.05;   //每次触发增加/减少0.02 rad/s
    const double trigger_threshold=0.5;  //扳机判定按下的阈值(初始1.0,按到底-1.0)
    const double repeat_interval=0.15;   //长按时每隔多少秒重复触发一次
    const double max_total_speed=1.0;    //线速度/角速度上限，均不超过1.0

    //边沿检测/长按计时用的状态
    bool last_acce_pressed=false;
    bool last_decel_pressed=false;
    bool last_turn_acce_pressed=false;
    bool last_turn_decel_pressed=false;
    double acce_timer=0.0;
    double decel_timer=0.0;
    double turn_acce_timer=0.0;
    double turn_decel_timer=0.0;
    rclcpp::Time last_time;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    printf("joy_control\n");
    rclcpp::spin(std::make_shared<wheeltec_joy>());
    return 0;
}