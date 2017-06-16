#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<tumtools/Math/MathDefs.h>
#include<tumtools/Math/EigenDefs.h>
#include<QApplication>


using namespace Eigen;

bool qIniReady=false;
Tum::VectorDOFd qIni;

void getInitialPosition(const sensor_msgs::JointState::ConstPtr &msg)
{

    if(!qIniReady)
    {
        ROS_INFO_STREAM("Got Message: "<<msg->position[0]<<", "<<msg->position[1]);
        if(msg->position.size()<STD_DOF)
        {
            ROS_ERROR_STREAM("Wrong JointState size: "<<msg->position.size());
            qIniReady=false;
        }
        else
        {
            for(int i=0;i<STD_DOF;i++)
            {
                qIni(i)=msg->position.at(i);
            }
            ROS_INFO_STREAM("QIni: "<<qIni.transpose());


            qIniReady=true;
            ROS_INFO_STREAM("qIniReady: "<<qIniReady);
        }
    }

}

int main(int argc, char **argv)
{
//    bool qIniReady=false;
    qIni.setZero();

    ros::init(argc,argv,"testJointCtrlPub",ros::init_options::AnonymousName);


    ros::NodeHandle n;
    ros::Rate r(125);

    ros::Publisher desiredJointStatePub=n.advertise<sensor_msgs::JointState>("/joint_desired_cmd", 1000);

    ros::Subscriber initQSubs=n.subscribe("ur10_arm_joint_states",100,getInitialPosition);

    while(!qIniReady)
    {
        if(!ros::ok())
        {
            break;
        }
        ros::spinOnce();
        ::usleep(5*1E5);
        ROS_INFO_STREAM("Waiting for QIni: "<<qIniReady);

    }


    Tum::VQString jointNames=Tum::VQString()
            << "shoulder_pan_joint"
            << "shoulder_lift_joint"
            << "elbow_joint"
            << "wrist_1_joint"
            << "wrist_2_joint"
            << "wrist_3_joint";

    sensor_msgs::JointState msg;

    msg.name.resize(STD_DOF);
    msg.position.resize(STD_DOF);
    msg.velocity.resize(STD_DOF);
    msg.effort.resize(STD_DOF);
    for(int i=0;i<STD_DOF;i++)
    {
        msg.name[i]=jointNames.at(i).toStdString();
    }



    int count=0;


    double w=2*M_PI/40.0;





    ros::Time tc;


    ROS_INFO_STREAM("qIni: "<<RAD2DEG(qIni.transpose()));



    ros::Time ti=ros::Time::now();
    while(ros::ok())
    {
        tc=ros::Time::now();
        double t=tc.toSec()-ti.toSec();



        msg.header.seq=count;
        msg.header.stamp=tc;

        std::stringstream s;
        for(unsigned int i=0;i<STD_DOF;i++)
        {

            msg.position[i]=DEG2RAD(20.0)*sin(w*t)+qIni(i);
            msg.velocity[i]=0.0;
            msg.effort[i]=0.0;

            s<<RAD2DEG(msg.position[i])<<", ";
        }

        ROS_INFO_STREAM("Qd: "<<s.str());

        desiredJointStatePub.publish(msg);



        ros::spinOnce();
        r.sleep();

        count++;
    }

    std::cout<<"main: Stopped!!!"<<std::endl;

}

