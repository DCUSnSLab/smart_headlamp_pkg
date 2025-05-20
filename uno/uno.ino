#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#define LAMP_PIN 3

bool g_is_high_beam = false;
int g_distance_cm = -1234;

ros::NodeHandle nh;


void servo_angle_callback(const sensor_msgs::JointState& msg) {
    // Low-beam (Angle of Servo 3 > 0) : 255
    // High-beam (Angle of Servo 3 <= 0)
        // nobody in front of lamp : 255
        // human in front of lamp (dimming with distance between human and lamp) : 255 ~ 50
} ros::Subscriber <sensor_msgs::JointState> ang_sub("joint_states", &servo_angle_callback);


void human_distance_callback(const std_msgs::Int32& msg) {
    
} ros::Subscriber <std_msgs::Int32> dis_sub("human_distance", &human_distance_callback);


void dim_lamp() {
    int brightness = 255;
    if g_is_high_beam && (g_distance_cm != -1234) {
        if g_distance_cm < 255 {
            brightness = g_distance_cm;
        }
    }
    analogWrite(LAMP_PIN, brightness);
}


void setup() {
    nh.initNode();
    nh.subscribe(ang_sub);
    nh.subscribe(dis_sub);
    pinMode(LAMP_PIN, OUTPUT);
}


void loop() {
    dim_lamp();
}