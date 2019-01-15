#include <ros/ros.h>
#include <selfie_starting_procedure/starting_procedure.h>
#include <selfie_starting_procedure/qr_decoder.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starting_procedure");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    qr_decoder qr_dec(pnh,nh);
    Starting_procedure start_proc(pnh,nh);

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        if(start_proc.button_clicked())
            qr_dec.begin_search();

        if(qr_dec.end_search())
            start_proc.drive();

        ros::spinOnce();
        loop_rate.sleep();
    }

}
